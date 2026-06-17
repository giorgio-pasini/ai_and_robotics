# =============================================================================
#  Robot Maqueen Plus + micro:bit (MicroPython)
#  Etape 5 : "Danser" -- choregraphie rock'n'roll.
#
#  Contrairement aux etapes precedentes (1 m, 90 deg, carre) qui visent la
#  PRECISION par odometrie + PID, une danse cherche le RYTHME et le spectacle.
#  On pilote donc les moteurs en boucle ouverte (commandes temporisees) :
#  c'est plus nerveux, plus simple a choregraphier, et les LEDs accompagnent
#  chaque figure.
#
#  Registres I2C utilises (cf. Adresses.pdf, registre 0x10) :
#      0x00 dir / 0x01 vitesse  -> moteur gauche
#      0x02 dir / 0x03 vitesse  -> moteur droit
#      0x0B couleur             -> LED RGB gauche
#      0x0C couleur             -> LED RGB droite
#
#  Le BUZZER (broche pin0 de la micro:bit) joue une musique rock en fond
#  pendant toute la danse. /!\ L'interrupteur physique "Buzzer Switch" sur le
#  robot doit etre sur ON, sinon aucun son ne sortira.
#
#  Commande : A ou B -> lance la choregraphie une fois, puis revient au repos.
# =============================================================================
from microbit import i2c, sleep, running_time, button_a, button_b, display, Image
from random import randint
import music

# =============================================================================
#  Communication I2C (adresse 0x10)
# =============================================================================
ADDR = 0x10
STOP, FORWARD, BACKWARD = 0, 1, 2

def left_motor(direction, speed):  i2c.write(ADDR, bytes([0x00, direction, speed]))
def right_motor(direction, speed): i2c.write(ADDR, bytes([0x02, direction, speed]))
def stop():                        left_motor(STOP, 0); right_motor(STOP, 0)
def set_internal_pid(on):          i2c.write(ADDR, bytes([0x0A, 1 if on else 0]))

# =============================================================================
#  LEDs RGB
#  Couleurs 0-7 : 0 eteint, 1 rouge, 2 vert, 3 jaune, 4 bleu, 5 violet,
#                 6 cyan, 7 blanc.
# =============================================================================
OFF, RED, GREEN, YELLOW, BLUE, VIOLET, CYAN, WHITE = 0, 1, 2, 3, 4, 5, 6, 7
PALETTE = [RED, GREEN, YELLOW, BLUE, VIOLET, CYAN, WHITE]

def set_leds(gauche, droite):
    i2c.write(ADDR, bytes([0x0B, gauche]))
    i2c.write(ADDR, bytes([0x0C, droite]))

def leds_off():
    set_leds(OFF, OFF)

def couleur_aleatoire():
    return PALETTE[randint(0, len(PALETTE) - 1)]

# =============================================================================
#  Musique (buzzer sur pin0)
#  On joue une boucle rock EN FOND (wait=False) : la fonction rend la main
#  immediatement, le buzzer continue tout seul pendant que les moteurs dansent.
#  Notation micro:bit : "NOTE[octave][:duree]" (duree en multiples du tempo).
# =============================================================================
MELODIE_ROCK = [
    "e4:2", "e4:2", "r:2", "e4:2", "r:2", "c4:2", "e4:4",
    "g4:4", "r:4", "g3:4", "r:4",
    "c4:3", "g3:2", "r:2", "e3:3", "a3:2", "b3:2", "a3:2", "g3:3",
]

def demarrer_musique():
    music.set_tempo(bpm=160)
    music.play(MELODIE_ROCK, wait=False, loop=True)

def arreter_musique():
    music.stop()

# =============================================================================
#  Helpers de mouvement
#  spd_l / spd_r : vitesses gauche / droite, SIGNEES.
#      > 0 -> roue en avant, < 0 -> roue en arriere, 0 -> roue a l'arret.
#  En jouant sur la difference des deux vitesses on obtient toutes les figures :
#      avancer, reculer, pivoter sur place, decrire un cercle ou un arc.
# =============================================================================
def clamp_speed(v):
    if v > 255:  return 255
    if v < -255: return -255
    return int(v)

def set_wheels(spd_l, spd_r):
    spd_l = clamp_speed(spd_l)
    spd_r = clamp_speed(spd_r)
    left_motor(FORWARD if spd_l >= 0 else BACKWARD, abs(spd_l))
    right_motor(FORWARD if spd_r >= 0 else BACKWARD, abs(spd_r))

def bouger(spd_l, spd_r, duree_ms, clignote_ms=120):
    """Applique (spd_l, spd_r) pendant duree_ms en faisant clignoter les LEDs.
    Renvoie False si A ou B est presse (arret d'urgence de la choregraphie)."""
    set_wheels(spd_l, spd_r)
    fin = running_time() + duree_ms
    prochain_clignotement = 0
    while running_time() < fin:
        if button_a.was_pressed() or button_b.was_pressed():
            return False
        if running_time() >= prochain_clignotement:
            set_leds(couleur_aleatoire(), couleur_aleatoire())
            prochain_clignotement = running_time() + clignote_ms
        sleep(10)
    return True

def pause(duree_ms):
    stop()
    return bouger(0, 0, duree_ms, clignote_ms=80)

# =============================================================================
#  Figures de danse (primitives rock'n'roll)
# =============================================================================
def tour_sur_soi(sens=1, vitesse=200, duree_ms=700):
    # Tour sur place : une roue en avant, l'autre en arriere.
    # sens = +1 anti-horaire (gauche), -1 horaire (droite).
    return bouger(-sens * vitesse, sens * vitesse, duree_ms, clignote_ms=70)

def pirouette(sens=1, tours=3, v_min=150, v_max=255, duree_ms=320):
    # Toupie "vivante" : on enchaine plusieurs tours en faisant pulser la vitesse
    # (accelere / ralentit) pour que la rotation ne soit pas plate. Bien plus
    # "roue" qu'une simple avancee. duree_ms = duree d'une pulsation.
    for i in range(tours):
        v = v_max if i % 2 == 0 else v_min
        if not bouger(-sens * v, sens * v, duree_ms, clignote_ms=60):
            return False
    return True

def avancee_brusque(vitesse=220, duree_ms=180):
    # Petit coup en avant : court et sec (on ne s'eloigne pas).
    return bouger(vitesse, vitesse, duree_ms, clignote_ms=60)

def recul_brusque(vitesse=220, duree_ms=180):
    # Petit coup en arriere : court et sec.
    return bouger(-vitesse, -vitesse, duree_ms, clignote_ms=60)

def cercle(sens=1, vitesse=220, serrage=0.6, duree_ms=1100):
    # Cercle SERRE (serrage proche de 1 -> petit rayon, ca tourne vraiment).
    # serrage 0 -> ligne droite, proche de 1 -> cercle tres serre.
    interne = vitesse * (1.0 - serrage)
    if sens >= 0:   # cercle vers la gauche : roue gauche = interne
        return bouger(interne, vitesse, duree_ms)
    else:           # cercle vers la droite : roue droite = interne
        return bouger(vitesse, interne, duree_ms)

def huit(vitesse=220, serrage=0.6, duree_ms=1000):
    # Figure en "8" : un cercle serre dans un sens, puis l'autre.
    if not cercle(sens=1,  vitesse=vitesse, serrage=serrage, duree_ms=duree_ms):
        return False
    return cercle(sens=-1, vitesse=vitesse, serrage=serrage, duree_ms=duree_ms)

def shimmy(repetitions=4, vitesse=200, duree_ms=120):
    # Petits a-coups gauche/droite tres rapides : le robot "tremble" en rythme.
    for i in range(repetitions):
        sens = 1 if i % 2 == 0 else -1
        if not bouger(-sens * vitesse, sens * vitesse, duree_ms, clignote_ms=duree_ms):
            return False
    return True

# =============================================================================
#  Choregraphie
#  Enchainement des figures demandees. Chaque appel renvoie False si un bouton
#  a interrompu la danse -> on propage l'arret immediatement.
# =============================================================================
def danser():
    sequence = (
        # 1. Entree : petit recul sec, puis il part en TOUPIE.
        lambda: recul_brusque(vitesse=220, duree_ms=200),
        lambda: pirouette(sens=1, tours=4, duree_ms=300),
        lambda: pause(100),

        # 2. A-coups avant/arriere COURTS (il reste sur place) entre deux toupies.
        lambda: avancee_brusque(duree_ms=160),
        lambda: recul_brusque(duree_ms=160),
        lambda: pirouette(sens=-1, tours=3, duree_ms=300),
        lambda: pause(100),

        # 3. Tremblement rock (shimmy).
        lambda: shimmy(repetitions=6, vitesse=210, duree_ms=110),
        lambda: pause(100),

        # 4. Cercles serres d'un sens puis de l'autre.
        lambda: cercle(sens=1,  vitesse=230, serrage=0.65, duree_ms=1000),
        lambda: cercle(sens=-1, vitesse=230, serrage=0.65, duree_ms=1000),
        lambda: pause(100),

        # 5. Figure en "8".
        lambda: huit(vitesse=220, serrage=0.6, duree_ms=900),
        lambda: pause(100),

        # 6. Toupies alternees rapides (gauche/droite) en accelerant.
        lambda: tour_sur_soi(sens=1,  vitesse=200, duree_ms=400),
        lambda: tour_sur_soi(sens=-1, vitesse=230, duree_ms=400),
        lambda: tour_sur_soi(sens=1,  vitesse=255, duree_ms=400),
        lambda: pause(100),

        # 7. Reprise : a-coups + grande pirouette dans l'autre sens.
        lambda: recul_brusque(duree_ms=160),
        lambda: avancee_brusque(duree_ms=160),
        lambda: pirouette(sens=-1, tours=4, duree_ms=300),
        lambda: pause(100),

        # 8. Deuxieme figure en "8", plus serree.
        lambda: huit(vitesse=230, serrage=0.7, duree_ms=850),
        lambda: pause(100),

        # 9. Shimmy plus long pour relancer.
        lambda: shimmy(repetitions=8, vitesse=220, duree_ms=100),
        lambda: pause(100),

        # 10. Final : petit coup avant + grande toupie a fond.
        lambda: avancee_brusque(duree_ms=160),
        lambda: pirouette(sens=1,  tours=6, v_min=220, v_max=255, duree_ms=280),
    )
    demarrer_musique()
    for figure in sequence:
        if not figure():
            arreter_musique()
            return False
    arreter_musique()
    return True

# =============================================================================
#  Programme principal
#  A ou B -> lance la danse une fois. Un appui pendant la danse l'interrompt.
# =============================================================================
set_internal_pid(False)
stop()
leds_off()
display.show(Image.MUSIC_QUAVER)

while True:
    if button_a.was_pressed() or button_b.was_pressed():
        display.show(Image.HEART)
        fini = danser()
        stop()
        arreter_musique()
        leds_off()
        display.show(Image.YES if fini else Image.NO)
        sleep(500)
        display.show(Image.MUSIC_QUAVER)
    sleep(50)
