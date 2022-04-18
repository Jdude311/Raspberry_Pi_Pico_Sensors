#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Misc
import time
import board
import busio
import microcontroller
# Sensor
import adafruit_scd30
# WiFi
from adafruit_esp32spi import adafruit_esp32spi
from adafruit_esp32spi import adafruit_esp32spi_wifimanager
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
from digitalio import DigitalInOut
# MQTT
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from adafruit_io.adafruit_io import IO_MQTT

i2c = busio.I2C(board.GP27, board.GP26, frequency=100000)

# Set up sensor
scd30 = adafruit_scd30.SCD30(i2c)

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

# Takes reading and saves/prints/sends
def take_reading():
    iaq = [
        scd30.CO2,
        scd30.temperature,
        scd30.relative_humidity
    ]
    CO2, TVOC, temperature, relative_humidity = iaq[0], None, iaq[1], iaq[2] # eCO2, TVOC
    CO2_reads.append(CO2)
    TVOC_reads.append(TVOC)
    temperature_reads.append(temperature)
    relative_humidity_reads.append(relative_humidity)

# Evaluates median. Utility function
def median(l):
    l.sort()
    mid = len(l)//2
    if len(l) % 2 == 0:
        return (l[mid]+l[mid+1])//2
    else:
        return l[mid]

# Sends readings
def send_reading():
    CO2, TVOC, temperature, relative_humidity = median(CO2_reads), None, median(temperature_reads), median(relative_humidity_reads)
    location = "RDHS 312 (SCD30 TEST 1)"
    topic = "test"
    msg = f"{location},0,{CO2},{temperature},{relative_humidity}"
    broker.publish(topic, msg)
    print(msg)



# Warm up sensor
while True:
    time.sleep(1)
    if scd30.data_available:
        CO2 = scd30.CO2
        break

# Main loop
read_sec = 0
base_sec = 0
CO2_reads, TVOC_reads, temperature_reads, relative_humidity_reads = [], [], [], []
while True:
    time.sleep(1)
    read_sec += 1
    base_sec += 1
    try:
        # Take reading every 30 seconds
        if read_sec >= 30:
            send_reading()
            CO2_reads, TVOC_reads, temperature_reads, relative_humidity_reads = [], [], [], []
            read_sec = 0
        elif scd30.data_available:
            take_reading()
            #print("CO2: %d ppm \t TVOC: %d ppb" % (CO2_reads[-1], TVOC_reads[-1]))

    except (ValueError, RuntimeError) as e:
        print("Failed. Retrying.")
        wifi.reset()
        wifi.connect()
        broker.reconnect()
