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

#sgp30.set_iaq_relative_humidity(celcius=21.606, relative_humidity=41.5253)

print(
    "**** Baseline values: eCO2 = 0x%x, TVOC = 0x%x"
    % (sgp30.baseline_eCO2, sgp30.baseline_TVOC)
)
del base_eCO2_saved, base_TVOC_saved

# Takes reading and saves/prints/sends
def take_reading():
    iaq = sgp30.iaq_measure()
    eCO2, TVOC = iaq[0], iaq[1] # eCO2, TVOC
    eCO2_reads.append(eCO2)
    TVOC_reads.append(TVOC)

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
    eCO2, TVOC = median(eCO2_reads), median(TVOC_reads)
    location = "RDHS 312 (SGP30 TEST 1)"
    msg = f"{location},{TVOC},{eCO2}"
    print("eCO2: %d ppm \t TVOC: %d ppb" % (eCO2, TVOC))



# Warm up sensor
for i in range(20):
    time.sleep(1)
    eCO2 = sgp30.eCO2
    TVOC = sgp30.TVOC

# Main loop
read_sec = 0
base_sec = 0
eCO2_reads, TVOC_reads = [], []
while True:
    time.sleep(1)
    read_sec += 1
    base_sec += 1
    # Take reading every 30 seconds
    if read_sec >= 2:
        send_reading()
        eCO2_reads, TVOC_reads = [], []
        read_sec = 0
    else:
        take_reading()
        print("eCO2: %d ppm \t TVOC: %d ppb" % (eCO2_reads[-1], TVOC_reads[-1]))

    if base_sec % 30 == 0:
        base = sgp30.get_iaq_baseline() # eCO2, TVOC
        baseline_eCO2, baseline_TVOC = base[0], base[1]
        print("*** Baseline: " + str(baseline_eCO2) + "\t" + str(baseline_TVOC))
