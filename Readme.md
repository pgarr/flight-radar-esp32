# Flight Radar

This electronic device fetches data from Open Sky about nearby flights and displays "radar" icons on the round screen and data on the rectangle screen.

## BOM

- ESP32
- SSD1306 screen
- GC9A01 round screen

## Libraries/bins

- ESP32_GENERIC-\*.bin from https://micropython.org/download/ESP32_GENERIC
- gc9a01.py from https://github.com/russhughes/gc9a01py
- ssd1306.py from https://github.com/micropython/micropython-lib/blob/master/micropython/drivers/display/ssd1306/ssd1306.py - it may or may not be included in the standard MicroPython libraries - worth checking

## Configuration

Create config.py file with below parameters and send it to ESP alongisde the others.

```
WIFI_SSID =
WIFI_PASSWORD =

DATA_SCL_PIN = 19
DATA_SDA_PIN = 18
ROUND_SCK_PIN  = 14   # SPI Clock (SCL)
ROUND_MOSI_PIN = 13   # SPI Data (MOSI) (SDA)
ROUND_CS_PIN   = 5    # Chip Select
ROUND_DC_PIN   = 2    # Data/Command
ROUND_RST_PIN  = 1    # Reset

HOME_LAT =
HOME_LON =
RADAR_RANGE_KM = 10

POLL_INTERVAL = 10
```
