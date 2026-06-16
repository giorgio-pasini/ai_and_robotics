# =============================================================================
#  Robot Maqueen Plus + micro:bit (MicroPython)
#  ETAPE 9 : SUIVI DE LIGNE -- avec les 2 CAPTEURS EXTERNES (L3 et R3) UNIQUEMENT.
#
#  Idee : la ligne noire passe normalement ENTRE les deux capteurs externes
#  (les plus ecartes). Tant que les deux voient du blanc, le robot est centre
#  et va tout droit. Des qu'un capteur externe touche la ligne, c'est que le
#  robot a derive de ce cote -> on braque vers la ligne (roue OPPOSEE acceleree).
#
#     L3 et R3 sur le BLANC      -> robot centre        -> TOUT DROIT
#     L3 (gauche) touche le NOIR -> ligne a gauche      -> TOURNER A GAUCHE
#     R3 (droite) touche le NOIR -> ligne a droite      -> TOURNER A DROITE
#     L3 et R3 sur le NOIR       -> croisement / ligne large -> TOUT DROIT
#
#  Les 4 autres capteurs (L2 L1 R1 R2) sont volontairement IGNORES.
#
#  Commandes :
#   - Bouton A : demarre / arrete le suivi.
#   - Bouton B : mode TEST -> montre l'etat des 2 capteurs externes (a faire d'abord).
# =============================================================================
from microbit import i2c, sleep, button_a, button_b, display

# =============================================================================
#  Communication I2C (adresse 0x10)
# =============================================================================
ADDR = 0x10
STOP, FORWARD, BACKWARD = 0, 1, 2

def _read(reg, n):
    i2c.write(ADDR, bytes([reg]))
    return i2c.read(ADDR, n)

def left_motor(direction, speed):  i2c.write(ADDR, bytes([0x00, direction, speed]))
def right_motor(direction, speed): i2c.write(ADDR, bytes([0x02, direction, speed]))
def stop():                        left_motor(STOP, 0); right_motor(STOP, 0)
def set_internal_pid(on):          i2c.write(ADDR, bytes([0x0A, 1 if on else 0]))

# =============================================================================
#  Capteurs -- registre d'etat 0x1D. On n'utilise QUE les 2 externes :
#      bit5 = 0x20 -> L3 (externe GAUCHE)
#      bit0 = 0x01 -> R3 (externe DROITE)
#
#  POLARITE (verifiee au mode TEST) : sur ce robot, NOIR -> bit a 1, BLANC -> 0
#  (conforme au PDF "1 pour du noir"). Si un jour les blocs s'allument a
#  l'envers, change LIGNE_EST_BIT_0 (True <-> False).
# =============================================================================
L3 = 0x20      # capteur externe gauche
R3 = 0x01      # capteur externe droit

LIGNE_EST_BIT_0 = False      # noir -> bit a 1 (comme le PDF)

def lire_capteurs():
    return _read(0x1D, 1)[0]

def sur_la_ligne(etat, bit):
    """True si ce capteur voit la ligne NOIRE (gere la polarite du robot)."""
    bit_allume = (etat & bit) != 0
    if LIGNE_EST_BIT_0:
        return not bit_allume    # noir = bit a 0
    return bit_allume            # noir = bit a 1

# =============================================================================
#  Vitesses (a ajuster). Comme les capteurs externes detectent la derive
#  TARD (ligne deja loin), on braque FORT : roue interieure tres lente.
# =============================================================================
RAPIDE = 55      # roue exterieure / marche tout droit (baisse: evite de depasser)
LENT   = 8       # roue interieure dans un virage (braquage serre)
LOOP_MS = 5      # on relit les capteurs tres souvent

# =============================================================================
#  Boucle de suivi de ligne -- 2 capteurs externes
# =============================================================================
def suivre_ligne():
    while not button_a.was_pressed():
        etat = lire_capteurs()
        gauche = sur_la_ligne(etat, L3)     # ligne sous l'externe GAUCHE ?
        droite = sur_la_ligne(etat, R3)     # ligne sous l'externe DROIT ?

        if gauche and not droite:
            # Ligne touchee a GAUCHE -> tourner a gauche (roue gauche lente).
            left_motor(FORWARD, LENT)
            right_motor(FORWARD, RAPIDE)
        elif droite and not gauche:
            # Ligne touchee a DROITE -> tourner a droite (roue droite lente).
            left_motor(FORWARD, RAPIDE)
            right_motor(FORWARD, LENT)
        else:
            # Les deux sur le blanc (centre) OU les deux sur le noir (croisement)
            # -> tout droit.
            left_motor(FORWARD, RAPIDE)
            right_motor(FORWARD, RAPIDE)

        sleep(LOOP_MS)
    stop()

# =============================================================================
#  Mode TEST : affiche SEULEMENT les 2 capteurs externes, en gros blocs.
#   - bloc des 2 colonnes de GAUCHE  = L3
#   - bloc des 2 colonnes de DROITE  = R3
#  Passe du noir sous l'externe gauche -> le bloc gauche doit s'allumer (et lui
#  seul). Sinon, change LIGNE_EST_BIT_0 en haut du fichier.
# =============================================================================
def mode_test():
    while not button_b.was_pressed():
        etat = lire_capteurs()
        display.clear()
        if sur_la_ligne(etat, L3):              # bloc gauche (colonnes 0 et 1)
            for x in (0, 1):
                for y in range(5):
                    display.set_pixel(x, y, 9)
        if sur_la_ligne(etat, R3):              # bloc droit (colonnes 3 et 4)
            for x in (3, 4):
                for y in range(5):
                    display.set_pixel(x, y, 9)
        print("0x1D =", "{:06b}".format(etat & 0x3F),
              " L3:", sur_la_ligne(etat, L3), " R3:", sur_la_ligne(etat, R3))
        sleep(100)
    display.clear()

# =============================================================================
#  Programme principal
# =============================================================================
set_internal_pid(False)
stop()

while True:
    if button_a.was_pressed():
        suivre_ligne()
    elif button_b.was_pressed():
        mode_test()
    sleep(50)
