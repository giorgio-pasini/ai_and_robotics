# =============================================================================
#  Robot Maqueen Plus + micro:bit (MicroPython)
#  ETAPE 9 : RADIO -- cote TELECOMMANDE (emetteur).
#
#  A FLASHER SUR LA micro:bit TENUE EN MAIN / POSEE PRES DE L'ORDI.
#
#  COMMANDES (tout aux boutons, marche a plat sur le bureau) :
#     Bouton A             -> "L" tourner a GAUCHE
#     Bouton B             -> "R" tourner a DROITE
#     Boutons A + B ensemble  ->  "F" AVANCER
#     incline vers l'avant -> "F" AVANCER (au cas ou tu la tiens en main)
#     rien                 -> "S" STOP
#
#  On envoie en CONTINU : des que tu relaches, le robot s'arrete (temps reel).
#
#  /!\ Le GROUPE radio doit etre IDENTIQUE a celui de radio_robot.py.
# =============================================================================
from microbit import accelerometer, button_a, button_b, display, Image, sleep
import radio

# =============================================================================
#  Configuration RADIO  (MEME groupe que le robot)
# =============================================================================
GROUPE = 77
radio.config(group=GROUPE, queue=1, length=8)
radio.on()

# =============================================================================
#  Reglages
# =============================================================================
SEUIL   = 350    # inclinaison mini (accelerometre ~ -1024..1024) pour avancer
VITESSE = 128    # vitesse CONSTANTE envoyee au robot (0-255). 128 = moyenne.

def envoyer(cmd):
    radio.send(cmd + "{:03d}".format(VITESSE))   # ex "L128"

# =============================================================================
#  Boucle principale
# =============================================================================
while True:
    a = button_a.is_pressed()
    b = button_b.is_pressed()

    if a and b:
        cmd, img = "F", Image.ARROW_N            # A + B -> AVANCER
    elif a:
        cmd, img = "L", Image.ARROW_W            # A -> GAUCHE
    elif b:
        cmd, img = "R", Image.ARROW_E            # B -> DROITE
    elif accelerometer.get_y() < -SEUIL:
        cmd, img = "F", Image.ARROW_N            # incline devant -> AVANCER
    else:
        cmd, img = "S", Image.SQUARE_SMALL       # rien -> STOP

    envoyer(cmd)
    display.show(img)
    sleep(50)        # 50 ms : reaction rapide (temps reel)
