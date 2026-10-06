import gc
import machine
import math
import network
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
            self.error = str(e)

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
        try:
            r = urequests.get(url, timeout=6)
            data = r.json()
            r.close()

            planes = []
            for s in (data.get("states") or []):
                callsign = (s[1] or "").strip()
                lon, lat = s[5], s[6]
                heading = s[10]
                velocity = s[9]
                baro_alt = s[7]
                if lat is None or lon is None:
                    continue
                planes.append({
                    "callsign": callsign or "----",
                    "lat": lat, "lon": lon,
                    "heading": heading or 0,
                    "speed": velocity or 0,
                    "alt": baro_alt or 0
                })
            state.set_planes(planes)

        except Exception as e:
            state.set_error(e)

        gc.collect()
        time.sleep(config.POLL_INTERVAL)

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
    def __init__(self, spi, cs, dc, rst, range_km=config.RADAR_RANGE_KM):
        self.d = gc9a01py.GC9A01(
            spi, 240, 240,
            reset=rst, cs=cs, dc=dc,
            rotation=0
        )
        self.cx, self.cy = 120, 120
        self.r = 115
        self.max_range_km = range_km
        self.sweep_angle = 0.0

    def draw_static_rings(self):
        self.d.fill(gc9a01py.BLACK)
        for ring in (1, 2, 3):
            self.d.circle(self.cx, self.cy, int(self.r * ring / 3), gc9a01py.color565(0, 60, 0))
        self.d.hline(self.cx - self.r, self.cy, self.r * 2, gc9a01py.color565(0, 60, 0))
        self.d.vline(self.cx, self.cy - self.r, self.r * 2, gc9a01py.color565(0, 60, 0))

    def draw_sweep(self, dt_s):
        self.sweep_angle = (self.sweep_angle + dt_s * 90) % 360  # 90 deg/s
        rad = math.radians(self.sweep_angle - 90)
        x2 = self.cx + int(self.r * math.cos(rad))
        y2 = self.cy + int(self.r * math.sin(rad))
        self.d.line(self.cx, self.cy, x2, y2, gc9a01py.color565(0, 255, 0))

    def draw_planes(self, planes):
        for p in planes:
            pos = latlon_to_xy(p["lat"], p["lon"], config.HOME_LAT, config.HOME_LON,
                                self.max_range_km, self.r)
            if pos is None:
                continue
            x, y, _ = pos
            px, py = int(self.cx + x), int(self.cy + y)
            self.d.pixel(px, py, gc9a01py.color565(255, 255, 0))
            self.d.circle(px, py, 2, gc9a01py.color565(255, 255, 0))

    def render(self, planes, dt_s):
        self.draw_static_rings()
        self.draw_planes(planes)
        self.draw_sweep(dt_s)

def run():
    connect_wifi()

    spi = machine.SPI(2, baudrate=40000000, sck=machine.Pin(config.ROUND_SCK_PIN), mosi=machine.Pin(config.ROUND_MOSI_PIN))
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
        is_stale = (last_update == 0) or (time.ticks_diff(now, last_update) > STALE_THRESHOLD_MS)

        radar.render(planes, dt_s)

        if time.ticks_diff(now, last_info_render) > 500:
            info.render(planes, err, is_stale)
            last_info_render = now

        time.sleep_ms(20)


run()
