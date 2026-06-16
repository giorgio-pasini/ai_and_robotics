# robot_move.py
# =============================================================================
#  Robot Maqueen Plus + micro:bit (MicroPython)
#  Primitive move(distance_m, angle_rad) basee sur le modele unicycle.
# =============================================================================
from microbit import i2c, sleep, running_time, button_a, button_b, display, Image
import math

# --- Calibration -------------------------------------------------------------
COUNTS_PER_METER  = 591
WHEEL_BASE_M      = 0.10
WHEEL_BASE_COUNTS = int(WHEEL_BASE_M * COUNTS_PER_METER)  # ~59 counts

# --- Tolerances --------------------------------------------------------------
TOLERANCE_M   = 0.01
TOLERANCE_RAD = 0.02

# --- Gains PID (par roue) ----------------------------------------------------
KP = 120.0
KI = 10.0
KD = 5.0

# --- Limites moteur ----------------------------------------------------------
MAX_SPEED = 110
MIN_SPEED = 40
LOOP_MS   = 15

# --- Stabilisation -----------------------------------------------------------
STABLE_ITERATIONS = 1

# =============================================================================
#  I2C
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
def clear_encoders():              i2c.write(ADDR, bytes([0x04, 0, 0, 0, 0]))

# =============================================================================
#  Position tracking (signed counts per wheel)
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
    dirs = _read(0x00, 4)
    e    = _read(0x04, 4)
    g = (e[0] << 8) | e[1]
    d = (e[2] << 8) | e[3]
    if   dirs[0] == FORWARD:  pos_g += g - prev_g
    elif dirs[0] == BACKWARD: pos_g -= g - prev_g
    if   dirs[2] == FORWARD:  pos_d += d - prev_d
    elif dirs[2] == BACKWARD: pos_d -= d - prev_d
    prev_g = g; prev_d = d

def pos_g_m(): return pos_g / float(COUNTS_PER_METER)
def pos_d_m(): return pos_d / float(COUNTS_PER_METER)

# =============================================================================
#  Motor helpers
# =============================================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _apply_motor(cmd, motor_fn):
    cmd = clamp(cmd, -MAX_SPEED, MAX_SPEED)
    spd = int(abs(cmd))
    if 0 < spd < MIN_SPEED:
        spd = MIN_SPEED
    if cmd >= 0:
        motor_fn(FORWARD,  spd)
    else:
        motor_fn(BACKWARD, spd)

# =============================================================================
#  PID
# =============================================================================
class PID:
    def __init__(self, kp, ki, kd, max_i=MAX_SPEED):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_i = max_i
        self.reset(0.0)

    def reset(self, error=0.0):
        self.integral   = 0.0
        self.prev_error = error
        self.prev_time  = running_time()

    def update(self, error):
        now = running_time()
        dt  = (now - self.prev_time) / 1000.0
        if dt <= 0:
            dt = LOOP_MS / 1000.0
        self.integral = clamp(
            self.integral + error * dt, -self.max_i, self.max_i
        )
        d = (error - self.prev_error) / dt
        self.prev_error = error
        self.prev_time  = now
        return self.kp * error + self.ki * self.integral + self.kd * d

# =============================================================================
#  move(v, w)
#
#  v : distance lineaire (metres, + = avant, - = arriere)
#  w : angle total       (radians, + = gauche/CCW, - = droite/CW)
#
#  Cas :
#    w = 0            => ligne droite
#    v = 0            => pivot pur sur place
#    v != 0, w != 0   => arc de cercle (R = v/w)
# =============================================================================
def move(v, w):
    L = WHEEL_BASE_M

    if abs(w) < 1e-6:
        target_g = v
        target_d = v
    elif abs(v) < 1e-6:
        half     = w * L / 2.0
        target_g = -half
        target_d =  half
    else:
        R        = v / w
        target_g = w * (R - L / 2.0)
        target_d = w * (R + L / 2.0)

    reset_pos()
    pid_g  = PID(KP, KI, KD)
    pid_d  = PID(KP, KI, KD)
    stable = 0

    if abs(w) < 1e-6:
        display.show(Image.ARROW_N)
    elif w > 0:
        display.show(Image.ARROW_W)
    else:
        display.show(Image.ARROW_E)

    while not (button_a.was_pressed() or button_b.was_pressed()):
        update_pos()

        err_g = target_g - pos_g_m()
        err_d = target_d - pos_d_m()

        if abs(err_g) <= TOLERANCE_M and abs(err_d) <= TOLERANCE_M:
            stable += 1
            if stable >= STABLE_ITERATIONS:
                stop()
                return True          # no sleep here
        else:
            stable = 0
            _apply_motor(pid_g.update(err_g), left_motor)
            _apply_motor(pid_d.update(err_d), right_motor)

        sleep(LOOP_MS)

    stop()
    return False

# =============================================================================
#  Square pattern
# =============================================================================
def square_pattern(side_m=0.20, turn_direction=1):
    angle = (math.pi / 2) * turn_direction
    while True:
        if not move(side_m, 0):   return
        sleep(30)
        if not move(0, angle):    return
        sleep(30)
# =============================================================================
#  turn_and_hold(w)
#
#  Pivote sur place de w radians (+ = gauche/CCW, - = droite/CW) puis
#  MAINTIENT activement cette position avec le PID (resiste aux poussees).
#  Ne s'arrete que sur appui bouton A ou B.
# =============================================================================
def turn_and_hold(w):
    L = WHEEL_BASE_M

    # Pivot pur sur place : roues en sens oppose, meme amplitude.
    half     = w * L / 2.0
    target_g = -half
    target_d =  half

    reset_pos()
    pid_g = PID(KP, KI, KD)
    pid_d = PID(KP, KI, KD)

    if abs(w) < 1e-6:
        display.show(Image.YES)      # deja a destination, on tient juste la pos
    elif w > 0:
        display.show(Image.ARROW_W)
    else:
        display.show(Image.ARROW_E)

    # Boucle de maintien : pas de sortie sur "stable", on corrige en continu.
    while not (button_a.was_pressed() or button_b.was_pressed()):
        update_pos()

        err_g = target_g - pos_g_m()
        err_d = target_d - pos_d_m()

        if abs(err_g) <= TOLERANCE_M and abs(err_d) <= TOLERANCE_M:
            # Dans la tolerance : on coupe les moteurs mais on continue de
            # surveiller, pour repartir si on nous pousse hors position.
            stop()
        else:
            _apply_motor(pid_g.update(err_g), left_motor)
            _apply_motor(pid_d.update(err_d), right_motor)

        sleep(LOOP_MS)

    stop()
    return False
    
# =============================================================================
#  Main
# =============================================================================
set_internal_pid(False)
stop()
display.show(Image.ARROW_E)

while True:
    if button_a.was_pressed():
        turn_and_hold(2.35)
    sleep(50)