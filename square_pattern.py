# robot_square.py
# =============================================================================
#  Robot Maqueen Plus + micro:bit (MicroPython)
#  Deplacement en carre : avance -> tourne 90° -> avance -> tourne 90° -> ...
# =============================================================================
from microbit import i2c, sleep, running_time, button_a, button_b, display, Image
import math

# --- Calibration -------------------------------------------------------------
COUNTS_PER_METER  = 591
WHEEL_BASE_COUNTS = 59

# --- Cible et tolerance ------------------------------------------------------
SIDE_M        = 0.05         # longueur d'un cote du carre (metres)
TOLERANCE_M   = 0.01        # m
TURN_TARGET   = math.pi / 2 # 90°
TOLERANCE_RAD = 0.01        

# --- Gains PID (translation) -------------------------------------------------
KP = 50.0
KI = 15.0
KD = 0.0

# --- Gains PID (rotation) ----------------------------------------------------
KP_ROT = 150.0
KI_ROT = 4.0
KD_ROT = 20.0

# --- Limites moteur ----------------------------------------------------------
MAX_SPEED = 110
MIN_SPEED = 40
LOOP_MS   = 15

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

def clear_encoders():
    i2c.write(ADDR, bytes([0x04, 0, 0, 0, 0]))

# =============================================================================
#  Position tracking
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

def position_m():
    return (pos_g + pos_d) / 2.0 / COUNTS_PER_METER

# =============================================================================
#  Motor helpers
# =============================================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def moteur(x):
    x = clamp(x, -MAX_SPEED, MAX_SPEED)
    if x >= 0:
        left_motor(FORWARD,  int(x)); right_motor(FORWARD,  int(x))
    else:
        left_motor(BACKWARD, int(-x)); right_motor(BACKWARD, int(-x))

def applique_min(x):
    if 0 < x < MIN_SPEED:  return MIN_SPEED
    if -MIN_SPEED < x < 0: return -MIN_SPEED
    return x

def moteur_pivot(w_cmd):
    spd = int(clamp(abs(w_cmd), 0, MAX_SPEED))
    if 0 < spd < MIN_SPEED:
        spd = MIN_SPEED
    if w_cmd >= 0:
        left_motor(BACKWARD, spd); right_motor(FORWARD,  spd)
    else:
        left_motor(FORWARD,  spd); right_motor(BACKWARD, spd)

# =============================================================================
#  PID
# =============================================================================
class PID:
    def __init__(self, kp, ki, kd, max_speed=MAX_SPEED):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_max = max_speed / ki if ki > 0 else 0
        self.reset(0.0)

    def reset(self, error):
        self.integral   = 0.0
        self.prev_error = error
        self.prev_time  = running_time()

    def update(self, error):
        now = running_time()
        dt  = (now - self.prev_time) / 1000.0
        if dt <= 0:
            dt = LOOP_MS / 1000.0
        p = self.kp * error
        self.integral += error * dt
        if self.i_max:
            self.integral = clamp(self.integral, -self.i_max, self.i_max)
        i = self.ki * self.integral
        d = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        self.prev_time  = now
        return p + i + d

# =============================================================================
#  Primitives : avancer / pivoter
#  Both return False if a button was pressed (abort signal).
# =============================================================================
def avancer_vers(cible_m):
    """Advance to cible_m from current position. Returns False if aborted."""
    reset_pos()
    pid = PID(KP, KI, KD)
    display.show(Image.ARROW_N)
    iteration_de_correction = 0
    max_iteration = 5

    while not (button_a.was_pressed() or button_b.was_pressed()):
        update_pos()
        erreur = cible_m - position_m()
        if abs(erreur) <= TOLERANCE_M:
            if abs(erreur) <= TOLERANCE_M:
                if(iteration_de_correction >= max_iteration) :
                    stop()
                    sleep(200)
                    return True
            iteration_de_correction += 1
            stop()
            sleep(200)
            return True
        moteur(applique_min(pid.update(erreur)))
        sleep(LOOP_MS)

    stop()
    return False

def pivoter(angle_rad):
    """
    Rotate by angle_rad in place.
    Positive = left (CCW), negative = right (CW).
    Returns False if aborted.
    """
    reset_pos()
    pid_rot = PID(KP_ROT, KI_ROT, KD_ROT)
    display.show(Image.ARROW_W if angle_rad > 0 else Image.ARROW_E)
    iteration_de_correction = 0
    max_iteration = 5

    while not (button_a.was_pressed() or button_b.was_pressed()):
        update_pos()
        theta     = (pos_d - pos_g) / float(WHEEL_BASE_COUNTS)
        erreur    = angle_rad - theta
        if abs(erreur) <= TOLERANCE_RAD:
            if(iteration_de_correction >= max_iteration) :
                stop()
                sleep(200)
                return True
            iteration_de_correction += 1
        w_cmd = clamp(pid_rot.update(erreur), -MAX_SPEED, MAX_SPEED)
        moteur_pivot(w_cmd)
        sleep(LOOP_MS)

    stop()
    return False

# =============================================================================
#  Square pattern
#  Repeats indefinitely: advance SIDE_M -> turn 90° -> ... until button press.
#  turn_direction: +1 = left (CCW), -1 = right (CW)
# =============================================================================
def square_pattern(side_m=SIDE_M, turn_direction=1):
    """
    Drive the robot in a square pattern indefinitely.
    Each iteration: advance side_m, then turn 90° in turn_direction.
    Press A or B to stop.
    """
    angle = TURN_TARGET * turn_direction

    while True:
        # --- Advance one side ---
        if not avancer_vers(side_m):
            return

        # --- Turn 90° ---
        if not pivoter(angle):
            return

# =============================================================================
#  Main
# =============================================================================
set_internal_pid(False)
stop()
display.show(Image.ARROW_E)

while True:
    if button_a.was_pressed():
        square_pattern(side_m=SIDE_M, turn_direction=1)   # A → left turns
        display.show(Image.ARROW_E)
    elif button_b.was_pressed():
        square_pattern(side_m=SIDE_M, turn_direction=-1)  # B → right turns
        display.show(Image.ARROW_E)
    sleep(50)