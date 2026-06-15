# test_ultrason.py
# Test de lecture du capteur ultrason. Affiche en boucle la distance en cm.
# Pointe le capteur vers un mur : le nombre doit correspondre a la distance reelle.
#
# Si l'ecran affiche une croix (X) ou des valeurs absurdes, les broches sont
# probablement inversees ou differentes : essaie pin1/pin2 au lieu de pin13/pin14.
from microbit import display, Image, pin13, pin14, sleep
import machine, utime

TRIG = pin13     # broche de declenchement
ECHO = pin14     # broche d'echo

def distance_cm():
    TRIG.write_digital(0)
    utime.sleep_us(2)
    TRIG.write_digital(1)
    utime.sleep_us(10)
    TRIG.write_digital(0)
    duree = machine.time_pulse_us(ECHO, 1, 30000)   # timeout 30 ms
    if duree <= 0:
        return -1
    return duree / 58        # conversion microsecondes -> cm

while True:
    d = distance_cm()
    if d < 0:
        display.show(Image.NO)        # pas d'echo : mauvaise broche ou hors portee
    else:
        display.scroll(str(int(d)))
    sleep(300)
