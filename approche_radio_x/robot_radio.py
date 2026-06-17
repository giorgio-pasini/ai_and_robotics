"""
robot_radio.py  –  micro:bit #1 (on the Maqueen Plus v1 robot)
===============================================================
Listens for radio commands from the dongle and drives the motors.

Commands:
  F  → forward          B  → backward
  L  → spin left        R  → spin right
  S  → stop
  +  → speed up (+20)   -  → speed down (-20)

Flash with python.microbit.org — just paste and flash.
"""

from microbit import display, Image, sleep, i2c, running_time
import radio

# Must match dongle.py exactly
RADIO_CHANNEL = 7
RADIO_GROUP   = 1

radio.config(channel=RADIO_CHANNEL, group=RADIO_GROUP, power=7)
radio.on()

# ── Motor board (Maqueen Plus v1) ────────────────────────────────────────
ADDR      = 0x10
MAX_SPEED = 200
MIN_SPEED = 40
speed     = 120

# Wait for motor board to be ready
while ADDR not in i2c.scan():
    sleep(100)

def set_motors(left, right):
    ld = 1 if left  > 0 else 2 if left  < 0 else 0
    rd = 1 if right > 0 else 2 if right < 0 else 0
    ls = min(MAX_SPEED, int(abs(left)))
    rs = min(MAX_SPEED, int(abs(right)))
    i2c.write(ADDR, bytes([0, ld, ls, rd, rs]))

def stop_motors():
    i2c.write(ADDR, bytes([0, 0, 0, 0, 0]))

def forward():
    set_motors(speed, speed)
    display.show(Image.ARROW_N)

def backward():
    set_motors(-speed, -speed)
    display.show(Image.ARROW_S)

def turn_left():
    set_motors(-speed, speed)
    display.show(Image.ARROW_W)

def turn_right():
    set_motors(speed, -speed)
    display.show(Image.ARROW_E)

def stop():
    stop_motors()
    display.show(Image.SQUARE_SMALL)

# ── Watchdog: stop motors if no command received for this long (ms) ───────
# Protects against the USB cable being unplugged or the tab being closed
# while an arrow key is held down.
WATCHDOG_MS = 500
last_cmd_time = running_time()

stop()
display.scroll("RC", delay=80)
display.show(Image.SQUARE_SMALL)

while True:
    msg = radio.receive()
    if msg:
        last_cmd_time = running_time()
        if   msg == "F": forward()
        elif msg == "B": backward()
        elif msg == "L": turn_left()
        elif msg == "R": turn_right()
        elif msg == "S": stop()
        elif msg == "+": speed = min(MAX_SPEED, speed + 20)
        elif msg == "-": speed = max(MIN_SPEED, speed - 20)

    # Watchdog: if no message for 500 ms, stop for safety
    if running_time() - last_cmd_time > WATCHDOG_MS:
        stop_motors()

    sleep(10)