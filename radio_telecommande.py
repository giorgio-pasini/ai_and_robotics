# =============================================================================
#  Robot Maqueen Plus v1 + micro:bit (MicroPython)
#  ETAPE 9 : RADIO -- cote TELECOMMANDE (emetteur).
#
#  A FLASHER SUR LA micro:bit TENUE EN MAIN.
#
#  COMMANDES :
#     Bouton A              -> "L" tourner a GAUCHE (pivot)
#     Bouton B              -> "R" tourner a DROITE (pivot)
#     Boutons A + B         -> "F" AVANCER
#     incliner vers l'AVANT -> "F" AVANCER
#     incliner vers l'ARRIERE -> "B" RECULER
#     rien                  -> "S" STOP
#
#  On envoie en CONTINU (toutes les 50 ms) : des que tu relaches, le robot
#  s'arrete (le watchdog du robot coupe les moteurs ~400 ms apres le dernier
#  message). Reaction en temps reel.
#
#  /!\ channel + group DOIVENT etre IDENTIQUES a radio_robot.py.
# =============================================================================
from microbit import accelerometer, button_a, button_b, display, Image, sleep
import radio

# =============================================================================
#  Configuration RADIO  (IDENTIQUE au robot)
# =============================================================================
RADIO_CHANNEL = 7
RADIO_GROUP   = 77
radio.config(channel=RADIO_CHANNEL, group=RADIO_GROUP, power=7, queue=1, length=4)
radio.on()

# =============================================================================
#  Reglages
# =============================================================================
SEUIL = 350   # inclinaison mini (accelerometre ~ -1024..1024) pour avancer/reculer

# =============================================================================
#  Boucle principale
# =============================================================================
while True:
    a = button_a.is_pressed()
    b = button_b.is_pressed()
    y = accelerometer.get_y()

    if a and b:
        cmd, img = "F", Image.ARROW_N        # A + B        -> AVANCER
    elif a:
        cmd, img = "L", Image.ARROW_W        # A            -> GAUCHE
    elif b:
        cmd, img = "R", Image.ARROW_E        # B            -> DROITE
    elif y < -SEUIL:
        cmd, img = "F", Image.ARROW_N        # incline avant -> AVANCER
    elif y > SEUIL:
        cmd, img = "B", Image.ARROW_S        # incline arriere -> RECULER
    else:
        cmd, img = "S", Image.SQUARE_SMALL   # rien         -> STOP

    radio.send(cmd)
    display.show(img)
    sleep(50)        # 50 ms : reaction rapide (temps reel)
