# =============================================================================
#  Robot Maqueen Plus v1 + micro:bit (MicroPython)
#  ETAPE 9 : RADIO -- cote TELECOMMANDE (emetteur).
#
#  A FLASHER SUR LA micro:bit TENUE EN MAIN.
#
#  COMMANDES :
#     incliner vers l'AVANT   -> "F" AVANCER   (vitesse = a quel point t'inclines)
#     incliner vers l'ARRIERE -> "B" RECULER   (idem : plus tu penches, plus vite)
#     Bouton A                -> "L" tourner a GAUCHE (arc)
#     Bouton B                -> "R" tourner a DROITE (arc)
#     Boutons A + B           -> "F" AVANCER A FOND (burst pleine vitesse)
#     rien / a plat           -> "S" STOP
#
#  >>> REGLAGE DE VITESSE : proportionnel a l'inclinaison <<<
#  Penche un peu = lent, penche beaucoup = rapide. Plus besoin de vitesse fixe.
#
#  On envoie en CONTINU (toutes les 50 ms) : des que tu relaches / remets a plat,
#  le robot s'arrete (watchdog du robot ~400 ms apres le dernier message).
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
radio.config(channel=RADIO_CHANNEL, group=RADIO_GROUP, power=7, queue=1, length=8)
radio.on()

# =============================================================================
#  Reglages vitesse / inclinaison  (accelerometre ~ -1024..1024)
# =============================================================================
SEUIL        = 200    # inclinaison mini pour commencer a bouger (zone morte a plat)
TILT_MAX     = 750    # inclinaison consideree comme "a fond"
VITESSE_MIN  = 110    # vitesse a l'inclinaison SEUIL (assez pour demarrer)
VITESSE_MAX  = 245    # vitesse a l'inclinaison TILT_MAX (rapide)
VITESSE_TOURNE = 175  # vitesse (roue exterieure) dans les virages A / B
VITESSE_BURST  = 255  # A + B : avancer a fond

def map_vitesse(tilt):
    # |inclinaison| dans [SEUIL, TILT_MAX] -> vitesse dans [VITESSE_MIN, VITESSE_MAX]
    if tilt > TILT_MAX:
        tilt = TILT_MAX
    frac = (tilt - SEUIL) / (TILT_MAX - SEUIL)
    return int(VITESSE_MIN + frac * (VITESSE_MAX - VITESSE_MIN))

def envoyer(cmd, vitesse):
    radio.send(cmd + "{:03d}".format(vitesse))   # ex "F207"

# =============================================================================
#  Boucle principale
# =============================================================================
while True:
    a = button_a.is_pressed()
    b = button_b.is_pressed()
    y = accelerometer.get_y()

    if a and b:
        cmd, v, img = "F", VITESSE_BURST, Image.ARROW_N      # A+B -> a fond
    elif a:
        cmd, v, img = "L", VITESSE_TOURNE, Image.ARROW_W     # A   -> gauche
    elif b:
        cmd, v, img = "R", VITESSE_TOURNE, Image.ARROW_E     # B   -> droite
    elif y < -SEUIL:
        cmd, v, img = "F", map_vitesse(-y), Image.ARROW_N    # avant proportionnel
    elif y > SEUIL:
        cmd, v, img = "B", map_vitesse(y), Image.ARROW_S     # arriere proportionnel
    else:
        cmd, v, img = "S", 0, Image.SQUARE_SMALL             # rien -> STOP

    envoyer(cmd, v)
    display.show(img)
    sleep(50)        # 50 ms : reaction rapide (temps reel)
