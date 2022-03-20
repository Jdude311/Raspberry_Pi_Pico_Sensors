# SPDX-FileCopyrightText: Brent Rubell for Adafruit Industries
# SPDX-License-Identifier: MIT
# Modified by Jaden Cermak-Hosein for research project
# Original file URL: https://learn.adafruit.com/pages/21765/elements/3084926/download

import time
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

# Subscribe to all messages on the led feed
#io.subscribe("led")

prv_refresh_time = 0.0
count = 0
while True:
    # Send a new temperature reading to IO every 30 seconds
    try:
        if (time.monotonic() - prv_refresh_time) > 2:
            # take the cpu's temperature
            msg = f"Message number: {count}"
            # publish it to io
            print("Publishing %s..." % msg)
            io.publish("test", msg)
            print("Published!")
            prv_refresh_time = time.monotonic()
            count += 1
    except (ValueError, RuntimeError) as e:
        print("Failed to connect. Retrying...")
        wifi.reset()
        wifi.connect()
        io.reconnect()
        continue
