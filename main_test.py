# SPDX-FileCopyrightText: Brent Rubell for Adafruit Industries
# SPDX-License-Identifier: MIT
# Modified by Jaden Cermak-Hosein for research project
# Original file URL: https://learn.adafruit.com/pages/21765/elements/3084926/download

import time
import storage
from adafruit_ntp import NTP
from microcontroller import cpu
import board
import busio
from digitalio import DigitalInOut
from adafruit_esp32spi import adafruit_esp32spi
from adafruit_esp32spi import adafruit_esp32spi_wifimanager
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from adafruit_io.adafruit_io import IO_MQTT
import adafruit_sgp30
import calibration

### WiFi ###

# Get wifi details and more from a secrets.py file
try:
    from secrets import net_secrets, mqtt_secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    raise

# Raspberry Pi RP2040
esp32_cs = DigitalInOut(board.GP13)
esp32_ready = DigitalInOut(board.GP14)
esp32_reset = DigitalInOut(board.GP15)

spi = busio.SPI(board.GP10, board.GP11, board.GP12)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)

wifi = adafruit_esp32spi_wifimanager.ESPSPI_WiFiManager(esp, net_secrets)

# Configure the RP2040 Pico LED Pin as an output
led_pin = DigitalInOut(board.LED)
led_pin.switch_to_output()

# Define callback functions which will be called when certain events happen.
# pylint: disable=unused-argument
def connected(client):
    # Connected function will be called when the client is connected to Adafruit IO.
    print("Connected to Adafruit IO! ")


def subscribe(client, userdata, topic, granted_qos):
    # This method is called when the client subscribes to a new feed.
    print("Subscribed to {0} with QOS level {1}".format(topic, granted_qos))


# pylint: disable=unused-argument
def disconnected(client):
    # Disconnected function will be called when the client disconnects.
    print("Disconnected from Adafruit IO!")


def on_led_msg(client, topic, message):
    # Method called whenever user/feeds/led has a new value
    print("New message on topic {0}: {1} ".format(topic, message))
    if message == "ON":
        led_pin.value = True
    elif message == "OFF":
        led_pin.value = False
    else:
        print("Unexpected message on LED feed.")


# Connect to WiFi
print("Connecting to WiFi...")
wifi.connect()
print("Connected!")

# Initialize MQTT interface with the esp interface
MQTT.set_socket(socket, esp)

# Initialize a new MQTT Client object
mqtt_client = MQTT.MQTT(
    broker=mqtt_secrets["broker"],
    username=mqtt_secrets["username"],
    password=mqtt_secrets["password"],
)

# Initialize an MQTT Client
io = IO_MQTT(mqtt_client)

# Connect the callback methods defined above
io.on_connect = connected
io.on_disconnect = disconnected
io.on_subscribe = subscribe

# Set up a callback for the led feed
io.add_feed_callback("led", on_led_msg)

# Connect to MQTT broker
print("Connecting to MQTT broker...")
io.connect()

# Set up SGP30
i2c = busio.I2C(board.GP21, board.GP20, frequency=100000)
sgp30 = adafruit_sgp30.Adafruit_SGP30(i2c)
sgp30.iaq_init()

# Set up time via NTP
ntp = NTP(esp)
ntp.set_time()

# Restore calibration data
if calibration.calibration["co2eq_base"] is not None and calibration.calibration['tvoc_base'] is not None:
    sgp30.set_iaq_baseline(calibration.calibration['co2eq_base'], calibration.calibration['tvoc_base'])

# Warm up the sensor
print("Sensor warming up...")
for i in range(10):
    time.sleep(1)
    print("eCO2: %d ppm" % sgp30.eCO2)
print("Sensor warmed up!\n")

count = 0
prv_refresh_time = time.monotonic()
calibration_refresh_time = time.monotonic()
while True:
    try:
        if (time.monotonic() - prv_refresh_time) > 30:
            data = [
                'Jaden\'s Room', # location
                str(time.time()), # time
                sgp30.eCO2 # CO2 reading
            ]
            msg = f"{data[0]},{data[1]},{data[2]}"
            # publish it to io
            print("Publishing %s..." % msg)
            print("eCO2: %d ppm" % data[2])
            io.publish("/test", msg)
            print("Published!")
            prv_refresh_time = time.monotonic()
            count += 1
        if (time.monotonic() - calibration_refresh_time) > 3600:
            try:
                co2eq_base, tvoc_base = sgp30.baseline_eCO2, sgp30.baseline_TVOC
                storage.remount("/", readonly=False)
                file = open('/lib/calibration.py', 'w')
                file.write(f"calibration=\{'co2eq_base': {co2eq_base}, 'tvoc_base': {tvoc_base}\}")
            except:
                io.publish("/error", "SENSOR COULD NOT SAVE CALIBRATION DATA!!!")
            calibration_refresh_time = time.monotonic()

    except (ValueError, RuntimeError) as e:
        print("Failed to connect. Retrying...")
        wifi.reset()
        wifi.connect()
        io.reconnect()
        continue
