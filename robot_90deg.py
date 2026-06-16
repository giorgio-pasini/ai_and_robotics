# =============================================================================
#  Robot Maqueen Plus + micro:bit (MicroPython)
#  Rotation sur place de 90 deg par modele UNICYCLE + regulateur PID.
#
#  Modele unicycle (robot differentiel a 2 roues) :
#      v = (v_droite + v_gauche) / 2      (vitesse d'avance)
#      w = (v_droite - v_gauche) / L      (vitesse de rotation)
#  Pour tourner SUR PLACE : v = 0  ->  v_droite = -v_gauche
#  (une roue avance, l'autre recule : le robot pivote autour de son centre).
#
#  Mesure de l'angle (odometrie) :
#      theta = (pos_droite - pos_gauche) / WHEEL_BASE_COUNTS   [radians]
#  pos_* = somme des deplacements d'encodeur signes par la direction.
#  WHEEL_BASE_COUNTS = entraxe exprime directement en counts d'encodeur.
#
#  Comportement : A -> pivote de +90 deg (gauche) ; B -> pivote de -90 deg
#  (droite). Le robot tourne, se stabilise sur la cible, puis s'arrete.
#  Approche et reglages repris du programme "carre" du groupe (deja testes).
# =============================================================================
from microbit import i2c, sleep, running_time, button_a, button_b, display, Image
import math

# --- Calibration -------------------------------------------------------------
# Entraxe en counts d'encodeur (~ COUNTS_PER_METER * entraxe_m ~ 591 * 0.10 m).
# Valeur testee sur ce robot par le groupe (cf. square_pattern.py).
WHEEL_BASE_COUNTS = 59

# --- Cible et tolerance ------------------------------------------------------
TURN_TARGET   = math.pi / 2     # 90 deg en radians
TOLERANCE_RAD = 0.01            # ~0.57 deg : robot considere "sur la cible"
STABLE_COUNT  = 5               # cycles consecutifs dans la tolerance avant d'arreter

# --- Gains du PID (rotation) -------------------------------------------------
KP_ROT = 150.0
KI_ROT = 4.0
KD_ROT = 20.0

# --- Limites moteur ----------------------------------------------------------
MAX_SPEED = 110
MIN_SPEED = 40
LOOP_MS   = 15                  # periode de la boucle (ms)

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

def clear_encoders():
    i2c.write(ADDR, bytes([0x04, 0, 0, 0, 0]))   # remet 0x04..0x07 a zero

# =============================================================================
#  Suivi de position de chaque roue (counts signes)
#  L'encodeur (0x04/0x06) ne donne qu'une magnitude ; on le signe avec le sens
#  lu dans le registre de direction (0x00/0x02 : 1 avant, 2 arriere).
# =============================================================================
prev_g = 0; prev_d = 0
pos_g  = 0; pos_d  = 0

def reset_pos():
    global prev_g, prev_d, pos_g, pos_d
    clear_encoders()
    sleep(5)
    e = _read(0x04, 4)
    prev_g = (e[0] << 8) | e[1]
    prev_d = (e[2] << 8) | e[3]
    pos_g = 0; pos_d = 0

def update_pos():
    global prev_g, prev_d, pos_g, pos_d
    dirs = _read(0x00, 4)            # [dir_gauche, _, dir_droite, _]
    e    = _read(0x04, 4)            # [encG_hi, encG_lo, encD_hi, encD_lo]
    g = (e[0] << 8) | e[1]
    d = (e[2] << 8) | e[3]
    if   dirs[0] == FORWARD:  pos_g += g - prev_g
    elif dirs[0] == BACKWARD: pos_g -= g - prev_g
    if   dirs[2] == FORWARD:  pos_d += d - prev_d
    elif dirs[2] == BACKWARD: pos_d -= d - prev_d
    prev_g = g; prev_d = d

def angle_rad():
    # Modele unicycle : l'angle vient de la DIFFERENCE des deux roues.
    # theta > 0 -> rotation anti-horaire (roue droite en avant, gauche en arriere).
    return (pos_d - pos_g) / float(WHEEL_BASE_COUNTS)

# =============================================================================
#  Commande de ROTATION sur place
#  w > 0 -> tourne a gauche (anti-horaire) : droite avance, gauche recule.
#  w < 0 -> tourne a droite (horaire).
#  Les commandes trop faibles sont relevees a MIN_SPEED (sinon le moteur cale).
# =============================================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def moteur_pivot(w_cmd):
    spd = int(clamp(abs(w_cmd), 0, MAX_SPEED))
    if 0 < spd < MIN_SPEED:
        spd = MIN_SPEED
    if w_cmd >= 0:
        left_motor(BACKWARD, spd); right_motor(FORWARD,  spd)
    else:
        left_motor(FORWARD,  spd); right_motor(BACKWARD, spd)

# =============================================================================
#  Regulateur PID
# =============================================================================
class PID:
    def __init__(self, kp, ki, kd, max_speed=MAX_SPEED):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_max = max_speed / ki if ki > 0 else 0
        self.reset(0.0)

    def reset(self, error):
        self.integral = 0.0
        self.prev_error = error
        self.prev_time = running_time()

    def update(self, error):
        now = running_time()
        dt = (now - self.prev_time) / 1000.0
        if dt <= 0:
            dt = LOOP_MS / 1000.0
        p = self.kp * error
        self.integral += error * dt
        if self.i_max:
            self.integral = clamp(self.integral, -self.i_max, self.i_max)
        i = self.ki * self.integral
        d = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        self.prev_time = now
        return p + i + d

# =============================================================================
#  Pivot sur place d'un angle donne (radians), avec confirmation de stabilite.
#  On ne s'arrete que lorsque l'angle reste DANS la tolerance pendant
#  STABLE_COUNT cycles consecutifs : evite de declarer "fini" en passant a
#  pleine vitesse devant la cible.
#  Retourne False si interrompu par un bouton.
# =============================================================================
def pivoter(cible_rad):
    reset_pos()
    pid = PID(KP_ROT, KI_ROT, KD_ROT)
    display.show(Image.ARROW_W if cible_rad >= 0 else Image.ARROW_E)
    stable = 0
    while not (button_a.was_pressed() or button_b.was_pressed()):
        update_pos()
        erreur = cible_rad - angle_rad()
        if abs(erreur) <= TOLERANCE_RAD:
            stop()
            stable += 1
            if stable >= STABLE_COUNT:     # vraiment stabilise sur la cible
                display.show(Image.YES)
                return True
        else:
            stable = 0                     # sorti de la zone -> on recommence a compter
            moteur_pivot(pid.update(erreur))
        sleep(LOOP_MS)
    stop()
    return False

# =============================================================================
#  Programme principal
#  A -> pivote de +90 deg (gauche)   |   B -> pivote de -90 deg (droite)
# =============================================================================
set_internal_pid(False)
stop()
display.show(Image.ARROW_N)
while True:
    if button_a.was_pressed():
        pivoter(+TURN_TARGET)
        sleep(300)
        display.show(Image.ARROW_N)
    elif button_b.was_pressed():
        pivoter(-TURN_TARGET)
        sleep(300)
        display.show(Image.ARROW_N)
    sleep(50)

# =============================================================================
#  CALIBRATION de WHEEL_BASE_COUNTS (a faire une fois, sur le robot reel)
#  --------------------------------------------------------------------
#  1. Marquer le cap de depart du robot (un trait au sol vers l'avant).
#  2. Lancer le programme, appuyer sur A : le robot tourne "de 90 deg".
#  3. Mesurer l'angle REEL effectue (rapporteur, ou repere visuel).
#  4. Corriger :
#         WHEEL_BASE_COUNTS = WHEEL_BASE_COUNTS * (90.0 / angle_reel_mesure)
#     - tourne TROP (>90 deg)  -> angle_reel grand -> WHEEL_BASE_COUNTS diminue
#     - tourne PAS ASSEZ (<90) -> angle_reel petit -> WHEEL_BASE_COUNTS augmente
#  5. Repeter jusqu'a tomber juste sur 90 deg.
# =============================================================================
