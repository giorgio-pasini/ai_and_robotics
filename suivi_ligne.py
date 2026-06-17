# =============================================================================
#  Robot Maqueen Plus + micro:bit (MicroPython)
#  ETAPE 9 : SUIVI DE LIGNE -- avec 2 capteurs (paire CONFIGURABLE).
#
#  Idee : la ligne noire passe normalement ENTRE les deux capteurs choisis.
#  Tant que les deux voient du blanc, le robot est centre et va tout droit.
#  Des qu'un capteur touche la ligne, c'est que le robot a derive de ce cote
#  -> on braque vers la ligne (roue OPPOSEE acceleree).
#
#     gauche et droite sur le BLANC -> robot centre   -> TOUT DROIT
#     capteur GAUCHE touche le NOIR -> ligne a gauche  -> TOURNER A GAUCHE
#     capteur DROITE touche le NOIR -> ligne a droite  -> TOURNER A DROITE
#     les deux sur le NOIR          -> croisement      -> TOUT DROIT
#
#  /!\ LE MAPPING DES BITS ET LES CAPTEURS QUI MARCHENT DEPENDENT DU ROBOT.
#      Toujours lancer le mode TEST (bouton B) sur un nouveau robot.
#      Sur CE robot : mapping inverse de l'ancien, et R3 est HS -> on utilise
#      la paire L2 / R2.
#
#  Commandes :
#   - Bouton A : demarre / arrete le suivi.
#   - Bouton B : mode TEST / diagnostic des 6 capteurs (a faire en premier).
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
#  Capteurs -- registre d'etat 0x1D (1 octet, 1 bit par capteur).
#
#  MAPPING DES BITS *POUR CE ROBOT* (verifie au diagnostic) :
#      bit0 = 0x01 -> L3 (externe GAUCHE)   bit3 = 0x08 -> R1
#      bit1 = 0x02 -> L2                     bit4 = 0x10 -> R2
#      bit2 = 0x04 -> L1                     bit5 = 0x20 -> R3 (externe DROITE, HS)
#  (Attention : un autre robot peut avoir l'ordre inverse -> re-tester.)
#
#  POLARITE : ici NOIR -> bit a 1, BLANC -> 0. Si l'affichage est inverse,
#  change LIGNE_EST_BIT_0.
# =============================================================================
L3, L2, L1 = 0x01, 0x02, 0x04
R1, R2, R3 = 0x08, 0x10, 0x20

# Tous les capteurs, ordre physique gauche -> droite (pour le diagnostic).
TOUS = (("L3", L3), ("L2", L2), ("L1", L1), ("R1", R1), ("R2", R2), ("R3", R3))

LIGNE_EST_BIT_0 = False      # noir -> bit a 1

# --- PAIRE DE CAPTEURS UTILISEE POUR LE SUIVI --------------------------------
# R3 est HS sur ce robot : on prend une paire symetrique qui fonctionne.
# (Tu peux mettre L3/R3 si les deux externes marchent, ou L1/R1 pour plus serre.)
CAPTEUR_GAUCHE = L2
CAPTEUR_DROITE = R2

def lire_capteurs():
    return _read(0x1D, 1)[0]

def sur_la_ligne(etat, bit):
    """True si ce capteur voit la ligne NOIRE (gere la polarite du robot)."""
    bit_allume = (etat & bit) != 0
    if LIGNE_EST_BIT_0:
        return not bit_allume    # noir = bit a 0
    return bit_allume            # noir = bit a 1

# =============================================================================
#  Filtre anti-glitch (resilience aux ombres / reflets passagers)
#  On ne change l'etat "vu / pas vu" d'un capteur que s'il reste pareil pendant
#  N lectures de suite. Une ombre qui ne dure qu'une lecture est donc ignoree.
# =============================================================================
class Filtre:
    def __init__(self, n=3):
        self.n = n           # nb de lectures concordantes pour basculer
        self.etat = False    # etat "stable" courant
        self.compte = 0      # depuis combien de lectures la nouvelle valeur dure

    def maj(self, lecture):
        if lecture != self.etat:
            self.compte += 1
            if self.compte >= self.n:
                self.etat = lecture
                self.compte = 0
        else:
            self.compte = 0
        return self.etat

# =============================================================================
#  Vitesses (a ajuster)
# =============================================================================
RAPIDE = 55      # roue exterieure / marche tout droit
LENT   = 8       # roue interieure dans un virage (braquage serre)
LOOP_MS = 5      # on relit les capteurs tres souvent

# =============================================================================
#  Boucle de suivi de ligne -- 2 capteurs (avec filtre anti-glitch)
# =============================================================================
def suivre_ligne():
    fg = Filtre(3)
    fd = Filtre(3)
    while not button_a.was_pressed():
        etat = lire_capteurs()
        gauche = fg.maj(sur_la_ligne(etat, CAPTEUR_GAUCHE))   # ligne a gauche ?
        droite = fd.maj(sur_la_ligne(etat, CAPTEUR_DROITE))   # ligne a droite ?

        if gauche and not droite:
            # Ligne a GAUCHE -> tourner a gauche (roue gauche lente).
            left_motor(FORWARD, LENT)
            right_motor(FORWARD, RAPIDE)
        elif droite and not gauche:
            # Ligne a DROITE -> tourner a droite (roue droite lente).
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
#  Mode TEST / DIAGNOSTIC : verifie LES 6 CAPTEURS un par un.
#
#  Mode d'emploi :
#   1. Branche la micro:bit en USB et ouvre la console serie (REPL) de l'editeur.
#   2. Pose le robot sur le BLANC : la ligne doit etre [. . . . . .].
#   3. Passe lentement du NOIR sous chaque capteur, de gauche a droite.
#      -> le capteur correspondant doit passer a "#". S'il ne change JAMAIS,
#         ce capteur est mort / sale / mal cable.
#
#  Ecran 5x5 : ligne du HAUT = capteurs GAUCHE (L3 L2 L1),
#              ligne du BAS  = capteurs DROITE (R1 R2 R3).
# =============================================================================
def mode_test():
    pos = {"L3": (0, 0), "L2": (1, 0), "L1": (2, 0),
           "R1": (2, 4), "R2": (3, 4), "R3": (4, 4)}
    while not button_b.was_pressed():
        etat = lire_capteurs()
        display.clear()
        ligne_txt = ""
        for nom, bit in TOUS:
            vu = sur_la_ligne(etat, bit)
            ligne_txt += ("#" if vu else ".") + " "
            if vu:
                x, y = pos[nom]
                display.set_pixel(x, y, 9)
        print("L3 L2 L1 R1 R2 R3 =", ligne_txt, " (0x1D =",
              "{:06b}".format(etat & 0x3F), ")")
        sleep(150)
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
