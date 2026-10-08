import esp32
import framebuf
import gc
import machine
import math
import network
import sys
import time
import urequests
import _thread

import config
import ssd1306
import gc9a01py


def km_to_deg_lat(km):
    return km / 111.0


def km_to_deg_lon(km, at_lat):
    return km / (111.0 * math.cos(math.radians(at_lat)))


def swap16(c):
    return ((c & 0xFF) << 8) | (c >> 8)


def idf_largest():
    return max(h[2] for h in esp32.idf_heap_info(esp32.HEAP_DATA))


RADIUS_DEG_LAT = km_to_deg_lat(config.RADAR_RANGE_KM)
RADIUS_DEG_LON = km_to_deg_lon(config.RADAR_RANGE_KM, config.HOME_LAT)
STALE_THRESHOLD_MS = config.POLL_INTERVAL * 3 * 1000


def connect_wifi():
    ap_if = network.WLAN(network.AP_IF)
    ap_if.active(False)
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('Connecting to WiFi...')
        sta_if.active(True)
        sta_if.connect(config.WIFI_SSID, config.WIFI_PASSWORD)
        while not sta_if.isconnected():
            time.sleep(1)
    print('Network config:', sta_if.ifconfig())


class SharedState:
    def __init__(self):
        self.lock = _thread.allocate_lock()
        self.planes = []
        self.last_update = 0
        self.error = None

    def set_planes(self, planes):
        with self.lock:
            self.planes = planes
            self.last_update = time.ticks_ms()
            self.error = None

    def set_error(self, e):
        with self.lock:
            self.error = repr(e)

    def get(self):
        with self.lock:
            return list(self.planes), self.last_update, self.error


state = SharedState()


def poll_worker():
    url = (
        "https://opensky-network.org/api/states/all?"
        "lamin={:.4f}&lomin={:.4f}&lamax={:.4f}&lomax={:.4f}"
    ).format(
        config.HOME_LAT - RADIUS_DEG_LAT, config.HOME_LON - RADIUS_DEG_LON,
        config.HOME_LAT + RADIUS_DEG_LAT, config.HOME_LON + RADIUS_DEG_LON,
    )

    print("Polling url:", url)

    while True:
        gc.collect()
        print("MP free:", gc.mem_free(), "IDF largest:", idf_largest())
        delay = config.POLL_INTERVAL
        try:
            r = urequests.get(url, timeout=20)
            if r.status_code != 200:
                code = r.status_code
                r.close()
                raise RuntimeError("HTTP {}".format(code))
            data = r.json()
            r.close()
        except Exception as e:
            print("Poll error:", repr(e))
            sys.print_exception(e)
            state.set_error(e)
            delay = 3
        gc.collect()
        time.sleep(delay)


def latlon_to_xy(lat, lon, home_lat, home_lon, max_range_km, screen_radius_px):
    dlat = lat - home_lat
    dlon = lon - home_lon
    km_per_deg_lat = 111.0
    km_per_deg_lon = 111.0 * math.cos(math.radians(home_lat))

    dx_km = dlon * km_per_deg_lon
    dy_km = dlat * km_per_deg_lat

    dist_km = math.sqrt(dx_km**2 + dy_km**2)
    if dist_km > max_range_km:
        return None

    scale = screen_radius_px / max_range_km
    x = dx_km * scale
    y = -dy_km * scale
    return x, y, dist_km


class InfoScreen:
    def __init__(self, i2c):
        self.d = ssd1306.SSD1306_I2C(128, 64, i2c)
        self.idx = 0
        self.last_switch = time.ticks_ms()

    def render(self, planes, err, is_stale=False):
        self.d.fill(0)
        if err:
            self.d.text("API error:", 0, 0)
            self.d.text(err[:16], 0, 12)
        elif is_stale:
            self.d.text("Dane nieaktualne", 0, 0)
            self.d.text("(brak odpowiedzi)", 0, 12)
        elif not planes:
            self.d.text("Brak samolotow", 0, 0)
            self.d.text("w zasiegu", 0, 12)
        else:
            if time.ticks_diff(time.ticks_ms(), self.last_switch) > 3000:
                self.idx = (self.idx+1) % len(planes)
                self.last_switch = time.ticks_ms()
            p = planes[self.idx % len(planes)]
            self.d.text(p["callsign"], 0, 0)
            self.d.text("ALT: {}m".format(int(p["alt"])), 0, 16)
            self.d.text("SPD: {}m/s".format(int(p["speed"])), 0, 28)
            self.d.text("HDG: {}deg".format(int(p["heading"])), 0, 40)
            self.d.text("{}/{}".format(self.idx+1, len(planes)), 0, 52)
        self.d.show()


class RadarScreen:
    STRIP_H = 24

    def __init__(self, spi, cs, dc, rst, range_km=config.RADAR_RANGE_KM):
        self.d = gc9a01py.GC9A01(spi, dc=dc, cs=cs, reset=rst, rotation=0)
        self.cx, self.cy = 120, 120
        self.r = 115
        self.max_range_km = range_km
        self.angle = 0.0

        self.buf = bytearray(240 * self.STRIP_H * 2)
        self.fb = framebuf.FrameBuffer(
            self.buf, 240, self.STRIP_H, framebuf.RGB565)

        c = gc9a01py.color565
        self.c_ring = swap16(c(0, 70, 0))
        self.c_plane = swap16(c(255, 255, 0))
        self.trail = [swap16(c(0, g, 0)) for g in (255, 190, 130, 80, 45, 20)]

        self.d.fill(gc9a01py.BLACK)

    def _plane_shape(self, px, py, heading):
        h = math.radians(heading)
        s, co = math.sin(h), -math.cos(h)
        nose = (px + int(6 * s), py + int(6 * co))
        left = (px + int(-4 * co - 3 * s), py + int(4 * s - 3 * co))
        right = (px + int(4 * co - 3 * s), py + int(-4 * s - 3 * co))
        return nose, left, right

    def render(self, planes, dt_s):
        cx, cy, r, H = self.cx, self.cy, self.r, self.STRIP_H
        fb = self.fb

        self.angle = (self.angle + dt_s * 90) % 360

        sweep = []
        for i, col in enumerate(self.trail):
            a = math.radians(self.angle - 90 - i * 4)
            sweep.append((cx + int(r * math.cos(a)),
                         cy + int(r * math.sin(a)), col))

        shapes = []
        for p in planes:
            pos = latlon_to_xy(p["lat"], p["lon"], config.HOME_LAT,
                               config.HOME_LON, self.max_range_km, r)
            if pos is None:
                continue
            px, py = int(cx + pos[0]), int(cy + pos[1])
            shapes.append(self._plane_shape(px, py, p["heading"]))

        for y0 in range(0, 240, H):
            fb.fill(0)
            oy = y0
            for k in (1, 2, 3):
                rr = r * k // 3
                fb.ellipse(cx, cy - oy, rr, rr, self.c_ring)
            fb.hline(cx - r, cy - oy, 2 * r, self.c_ring)
            fb.vline(cx, cy - r - oy, 2 * r, self.c_ring)

            for x2, y2, col in reversed(sweep):
                fb.line(cx, cy - oy, x2, y2 - oy, col)

            for nose, left, right in shapes:
                fb.line(nose[0], nose[1] - oy, left[0],
                        left[1] - oy, self.c_plane)
                fb.line(left[0], left[1] - oy, right[0],
                        right[1] - oy, self.c_plane)
                fb.line(right[0], right[1] - oy, nose[0],
                        nose[1] - oy, self.c_plane)

            self.d.blit_buffer(self.buf, 0, y0, 240, H)


def run():
    connect_wifi()

    spi = machine.SPI(1, baudrate=40000000, sck=machine.Pin(
        config.ROUND_SCK_PIN), mosi=machine.Pin(config.ROUND_MOSI_PIN))
    radar = RadarScreen(
        spi,
        cs=machine.Pin(config.ROUND_CS_PIN, machine.Pin.OUT),
        dc=machine.Pin(config.ROUND_DC_PIN, machine.Pin.OUT),
        rst=machine.Pin(config.ROUND_RST_PIN, machine.Pin.OUT),
    )
    radar = RadarScreen(
        spi,
        cs=machine.Pin(5, machine.Pin.OUT),
        dc=machine.Pin(2, machine.Pin.OUT),
        rst=machine.Pin(4, machine.Pin.OUT),
    )

    i2c = machine.I2C(scl=machine.Pin(config.DATA_SCL_PIN),
                      sda=machine.Pin(config.DATA_SDA_PIN))
    if 60 not in i2c.scan():
        raise RuntimeError('Cannot find display.')
    info = InfoScreen(i2c)

    _thread.start_new_thread(poll_worker, ())

    last_frame = time.ticks_ms()
    last_info_render = 0

    while True:
        now = time.ticks_ms()
        dt_s = time.ticks_diff(now, last_frame) / 1000.0
        last_frame = now

        planes, last_update, err = state.get()
        is_stale = (last_update == 0) or (
            time.ticks_diff(now, last_update) > STALE_THRESHOLD_MS)

        radar.render(planes, dt_s)

        if time.ticks_diff(now, last_info_render) > 500:
            info.render(planes, err, is_stale)
            last_info_render = now

        time.sleep_ms(20)


run()
