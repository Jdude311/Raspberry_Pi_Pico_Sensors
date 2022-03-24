#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Misc
import time
import board
import busio
import microcontroller
# Sensor
import adafruit_sgp30
# WiFi
from adafruit_esp32spi import adafruit_esp32spi
from adafruit_esp32spi import adafruit_esp32spi_wifimanager
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
from digitalio import DigitalInOut
# MQTT
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from adafruit_io.adafruit_io import IO_MQTT

i2c = busio.I2C(board.GP21, board.GP20, frequency=100000)

# Set up sensor
sgp30 = adafruit_sgp30.Adafruit_SGP30(i2c)
print("SGP30 serial #", [hex(i) for i in sgp30.serial])

# Set up secrets
try:
    from secrets import net_secrets, mqtt_secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")

# Set up WiFi and connect
esp32_cs = DigitalInOut(board.GP13)
esp32_ready = DigitalInOut(board.GP14)
esp32_reset = DigitalInOut(board.GP15)
spi = busio.SPI(board.GP10, board.GP11, board.GP12)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)
wifi = adafruit_esp32spi_wifimanager.ESPSPI_WiFiManager(esp, net_secrets)
wifi.connect()
print("Connected to WiFi!")

# Set up MQTT and connect
MQTT.set_socket(socket, esp)
mqtt_client = MQTT.MQTT(
    broker=mqtt_secrets["broker"],
    username=mqtt_secrets["username"],
    password=mqtt_secrets["password"],
)
broker = IO_MQTT(mqtt_client)
broker.connect()
print("Connected to MQTT!")

# Load saved calibration data
sgp30.iaq_init()
base_eCO2_saved = microcontroller.nvm[0:6]
base_TVOC_saved = microcontroller.nvm[6:12]
if base_eCO2_saved != bytearray(b'\x00'*6) and base_eCO2_saved != bytearray(b'\xff'*6):
    if base_TVOC_saved != bytearray(b'\x00'*6) and base_TVOC_saved != bytearray(b'\xff'*6):
        sgp30.set_iaq_baseline(
            int(str(base_eCO2_saved)[12:18], 16), # fucking bastard code ikr
            int(str(base_TVOC_saved)[12:18], 16)  # necessary because i'm stupid and can't get the actual value from bytearray
        )
del base_eCO2_saved, base_TVOC_saved

# Saves baseline to NVM
def save_baseline():
    baseline_eCO2, baseline_TVOC = sgp30.baseline_eCO2, sgp30.baseline_TVOC
    microcontroller.nvm[0:6] = bytearray(hex(baseline_eCO2))
    microcontroller.nvm[6:12] = bytearray(hex(baseline_TVOC))
    print(
        "**** Baseline values: eCO2 = 0x%x, TVOC = 0x%x"
        % (sgp30.baseline_eCO2, sgp30.baseline_TVOC)
    )

# Takes reading and saves/prints/sends
def take_reading():
    location = "Jaden\'s Room"
    eCO2, TVOC = sgp30.eCO2, sgp30.TVOC
    topic = "test"
    msg = f"{location},{TVOC},{eCO2}"
    broker.publish(topic, msg)
    print("eCO2: %d ppm \t TVOC: %d ppb" % (eCO2, TVOC))



# Warm up sensor
for i in range(20):
    time.sleep(1)
    eCO2 = sgp30.eCO2
    TVOC = sgp30.TVOC

# Main loop
read_sec = 0
base_sec = 0
while True:
    time.sleep(1)
    read_sec += 1
    base_sec += 1
    try:
        # Take reading every 30 seconds
        if read_sec > 30:
            take_reading()
            read_sec = 0

        # Save baseline hourly
        if base_sec > 3600:
            save_baseline()
            base_sec = 0
    except (ValueError, RuntimeError) as e:
        print("Failed. Retrying.")
        wifi.reset()
        wifi.connect()
        broker.reconnect()
