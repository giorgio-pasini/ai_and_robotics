"""
dongle.py  –  micro:bit #2 (plugged into PC via USB)
======================================================
Receives single-byte commands from the browser over USB serial
and re-broadcasts them to the robot over the micro:bit radio.

Flash with python.microbit.org — just paste and flash.
No wiring needed, just USB.
"""

from microbit import uart, display, Image, sleep
import radio

# Must match robot_radio.py exactly
RADIO_CHANNEL = 7
RADIO_GROUP   = 1

radio.config(channel=RADIO_CHANNEL, group=RADIO_GROUP, power=7)
radio.on()
uart.init(baudrate=115200)

VALID = set(b"FBLRSfblrs+-")

display.show(Image.ARROW_E)   # ready, waiting for serial connection

while True:
    data = uart.read(1)
    if data and data[0:1] in [bytes([b]) for b in VALID]:
        cmd = data.decode().upper()
        radio.send(cmd)
        # Quick flash to confirm relay
        if   cmd == "F": display.show(Image.ARROW_N)
        elif cmd == "B": display.show(Image.ARROW_S)
        elif cmd == "L": display.show(Image.ARROW_W)
        elif cmd == "R": display.show(Image.ARROW_E)
        elif cmd == "S": display.show(Image.SQUARE_SMALL)
        elif cmd in ("+", "-"): display.show(Image.HEART_SMALL)
    sleep(5)