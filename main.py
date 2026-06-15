# main.py
# -----------------------------------------------------------------------------
# OBJECTIF 1 : faire avancer le Maqueen Plus d'EXACTEMENT 1.00 metre,
#              en utilisant un regulateur PID base sur les codeurs de roue.
#
# Principe :
#   - les codeurs comptent combien les roues ont tourne -> on en deduit la
#     distance parcourue.
#   - un PID compare la distance parcourue a la cible (1 m) et calcule la
#     vitesse a envoyer aux moteurs.
#   - plus on approche de 1 m, plus l'erreur est petite, plus on ralentit,
#     et on s'arrete pile a la cible.
# -----------------------------------------------------------------------------
from microbit import sleep, running_time, button_a, display, Image
import maqueen

# =============================================================================
#  1) PARAMETRES A REGLER  (la "calibration" du projet)
# =============================================================================

# Combien de "counts" (unites des codeurs) pour parcourir 1 metre ?
# >>> VALEUR DE DEPART, A REMPLACER par ce que tu trouves avec calibrer.py <<<
COUNTS_PER_METER = 7400

TARGET_M    = 1.0      # distance a parcourir, en metres
TOLERANCE_M = 0.01     # on considere "arrive" quand il reste < 1 cm

# Gains du PID (a ajuster, voir la section "Reglage" du README)
KP = 300.0             # proportionnel : reagit a l'erreur actuelle
KI = 0.0               # integral      : corrige l'erreur qui dure (on commence a 0)
KD = 40.0              # derive        : freine quand on approche (amortit)

MAX_SPEED = 120        # vitesse max envoyee aux moteurs (0-255)
MIN_SPEED = 25         # vitesse mini pour que le robot bouge (sinon il cale)

KP_STRAIGHT = 0.05     # petite correction pour rouler droit (gauche vs droite)

LOOP_MS    = 20        # on recalcule toutes les 20 ms (50 fois par seconde)
TIMEOUT_MS = 8000      # securite : on coupe tout apres 8 s quoi qu'il arrive


# =============================================================================
#  2) LE REGULATEUR PID
# =============================================================================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = running_time()

    def update(self, error):
        # dt = temps ecoule depuis le dernier calcul, en secondes
        now = running_time()
        dt = (now - self.prev_time) / 1000.0
        if dt <= 0:
            dt = LOOP_MS / 1000.0

        # Les 3 termes du PID
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # on memorise pour le prochain tour
        self.prev_error = error
        self.prev_time = now
        return output


def clamp(value, low, high):
    """Garde value entre low et high."""
    return max(low, min(high, value))


# =============================================================================
#  3) LA FONCTION QUI FAIT LE 1 METRE
# =============================================================================
def distance_parcourue(start_l, start_r):
    """Renvoie (distance_en_metres, delta_gauche, delta_droite)."""
    dl = maqueen.enc_left()  - start_l
    dr = maqueen.enc_right() - start_r
    moyenne = (dl + dr) / 2.0
    return moyenne / COUNTS_PER_METER, dl, dr


def avancer_1m():
    # position de depart des codeurs (on travaille en "ecart" par rapport a ca)
    start_l = maqueen.enc_left()
    start_r = maqueen.enc_right()

    pid = PID(KP, KI, KD)
    t0 = running_time()

    while True:
        dist, dl, dr = distance_parcourue(start_l, start_r)
        erreur = TARGET_M - dist     # distance qu'il reste a parcourir

        # Conditions d'arret
        if erreur <= TOLERANCE_M:
            break                     # arrive !
        if running_time() - t0 > TIMEOUT_MS:
            break                     # securite

        # Le PID calcule la vitesse de base a partir de l'erreur
        vitesse = pid.update(erreur)
        vitesse = clamp(vitesse, MIN_SPEED, MAX_SPEED)

        # Correction "rouler droit" : si la roue gauche a plus avance que la
        # droite (dl > dr), on ralentit la gauche et on accelere la droite.
        corr = KP_STRAIGHT * (dl - dr)
        v_gauche = clamp(vitesse - corr, 0, MAX_SPEED)
        v_droite = clamp(vitesse + corr, 0, MAX_SPEED)

        maqueen.left_motor(maqueen.FORWARD,  int(v_gauche))
        maqueen.right_motor(maqueen.FORWARD, int(v_droite))

        sleep(LOOP_MS)

    maqueen.stop()
    return distance_parcourue(start_l, start_r)[0]


# =============================================================================
#  4) PROGRAMME PRINCIPAL
# =============================================================================
maqueen.set_internal_pid(False)   # c'est NOTRE PID qui pilote
maqueen.stop()

# On attend un appui sur le bouton A pour demarrer
display.show(Image.ARROW_E)
while not button_a.is_pressed():
    sleep(50)

display.show(Image.YES)
parcouru = avancer_1m()

# On affiche la distance reelle parcourue (en cm) pour verifier la precision
display.scroll(" {} cm".format(int(round(parcouru * 100))))
