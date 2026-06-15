# =============================================================================
#  Robot Maqueen Plus + micro:bit (MicroPython)
#  Maintien de position par regulateur PID (Proportionnel + Integral + Derive).
#
#  Position = somme des deplacements d'encodeur signes par la direction.
#  L'encodeur (registre 0x04/0x06) ne donne qu'une magnitude ; le sens vient du
#  registre de direction (0x00/0x02), lisible : 0 stop, 1 avant, 2 arriere.
#
#  Comportement : appuyer sur A ou B -> le robot rejoint la cible (1 m) puis la
#  maintient ; deplace a la main, il y revient. Nouvel appui : arret.
# =============================================================================
from microbit import i2c, sleep, running_time, button_a, button_b, display, Image

# --- Calibration -------------------------------------------------------------
COUNTS_PER_METER = 591      # counts d'encodeur correspondant a 1 metre

# --- Cible et tolerance ------------------------------------------------------
TARGET_M    = 1.0
TOLERANCE_M = 0.02

# --- Gains du PID ------------------------------------------------------------
KP = 180.0
KI = 6.0
KD = 30.0

# --- Limites moteur ----------------------------------------------------------
MAX_SPEED = 110
MIN_SPEED = 40
LOOP_MS   = 15      # periode de la boucle (ms) : plus petit = plus reactif

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
#  Suivi de position
#  L'encodeur (0x04/0x06) ne donne qu'une magnitude : il faut le signer avec le
#  sens de rotation lu dans le registre de direction (0x00/0x02 : 1 avant,
#  2 arriere). On accumule ainsi la position, meme quand le robot est pousse.
# =============================================================================
prev_g = 0; prev_d = 0       # dernieres valeurs d'encodeur
pos_g  = 0; pos_d  = 0       # position cumulee, signee (counts)

def reset_pos():
    # remet les encodeurs et la position a zero
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
    # ajoute le deplacement de chaque roue, signe par son sens de rotation reel
    if   dirs[0] == FORWARD:  pos_g += g - prev_g
    elif dirs[0] == BACKWARD: pos_g -= g - prev_g
    if   dirs[2] == FORWARD:  pos_d += d - prev_d
    elif dirs[2] == BACKWARD: pos_d -= d - prev_d
    prev_g = g; prev_d = d

def position_m():
    # moyenne des deux roues, convertie en metres
    cpm = COUNTS_PER_METER if COUNTS_PER_METER > 0 else 1
    return (pos_g + pos_d) / 2.0 / cpm

# =============================================================================
#  Commande moteur : x > 0 avance, x < 0 recule
# =============================================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def moteur(x):
    x = clamp(x, -MAX_SPEED, MAX_SPEED)
    if x >= 0:
        left_motor(FORWARD, int(x));   right_motor(FORWARD, int(x))
    else:
        left_motor(BACKWARD, int(-x)); right_motor(BACKWARD, int(-x))

def applique_min(x):
    if 0 < x < MIN_SPEED:   return MIN_SPEED
    if -MIN_SPEED < x < 0:  return -MIN_SPEED
    return x

# =============================================================================
#  Regulateur PID
# =============================================================================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_max = MAX_SPEED / ki if ki > 0 else 0
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
#  Maintien de position
# =============================================================================
def maintenir_position(cible_m):
    reset_pos()
    pid = PID(KP, KI, KD)
    while not (button_a.was_pressed() or button_b.was_pressed()):
        update_pos()
        erreur = cible_m - position_m()
        if abs(erreur) <= TOLERANCE_M:
            stop()
            pid.reset(erreur)
            display.show(Image.YES)
        else:
            moteur(applique_min(pid.update(erreur)))
            display.show(Image.ARROW_N)
        sleep(LOOP_MS)
    stop()

# =============================================================================
#  Programme principal
# =============================================================================
set_internal_pid(False)
stop()
display.show(Image.ARROW_E)
while True:
    if button_a.was_pressed() or button_b.was_pressed():
        maintenir_position(TARGET_M)
        display.show(Image.ARROW_E)
    sleep(50)
