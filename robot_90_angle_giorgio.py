# robot_1m_turn.py
# =============================================================================
#  Robot Maqueen Plus + micro:bit (MicroPython)
#  Maintien de position par regulateur PID (Proportionnel + Integral + Derive).
#
#  Comportement :
#    1. Avance vers TARGET_M / 2  (0,5 m) et s'y stabilise.
#    2. Pivote de 90° SUR PLACE via le modele unicycle (roues en sens opposes).
#    3. Avance vers TARGET_M / 2  (0,5 m supplementaires) et maintient la pos.
#    Appuyer sur A ou B pendant n'importe quelle phase arrete le robot.
#
#  Modele unicycle :
#    v  = (v_d + v_g) / 2   -- vitesse lineaire (non utilisee en pivot pur)
#    w  = (v_d - v_g) / L   -- vitesse angulaire
#    => pivot pur : v_g = -v_pivot, v_d = +v_pivot  (tourne a gauche)
#    Angle integre via encodeurs :
#      dtheta = (delta_d - delta_g) / WHEEL_BASE_COUNTS
#    Cible : pi/2 rad (90°)
# =============================================================================
from microbit import i2c, sleep, running_time, button_a, button_b, display, Image
import math

# --- Calibration -------------------------------------------------------------
COUNTS_PER_METER = 591      # counts d'encodeur = 1 metre

# Entraxe exprime en counts d'encodeur.
# Formule : WHEEL_BASE_COUNTS = (pi * entraxe_m) / (pi * roue_m) * COUNTS_PER_METER
#           => WHEEL_BASE_COUNTS = (entraxe_m / roue_m) * COUNTS_PER_METER
# Maqueen Plus : entraxe ~12,8 cm, diametre roue ~6,5 cm
# => ratio ~1,969  => ~1163 counts
# Ajustez WHEEL_BASE_COUNTS si la rotation n'est pas exactement 90°.
WHEEL_BASE_COUNTS = 56    # counts pour une rotation complete / (2*pi) * 2*pi = idem

# --- Cible et tolerance ------------------------------------------------------
TARGET_M      = 0.2         # distance totale a parcourir
HALF_M        = TARGET_M / 2.0
TOLERANCE_M   = 0.02        # m
TURN_TARGET   = math.pi / 2 # 90° en radians
TOLERANCE_RAD = 0.04        # ~2,3°

# --- Gains du PID (translation) ----------------------------------------------
KP = 180.0
KI = 6.0
KD = 30.0

# --- Gains du PID (rotation) -------------------------------------------------
KP_ROT = 150.0
KI_ROT = 4.0
KD_ROT = 20.0

# --- Limites moteur ----------------------------------------------------------
MAX_SPEED     = 110
MIN_SPEED     = 40
TURN_SPEED    = 50          # vitesse de base pendant le pivot
LOOP_MS       = 15

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
    i2c.write(ADDR, bytes([0x04, 0, 0, 0, 0]))

# =============================================================================
#  Suivi de position (translation)
#  pos_g / pos_d : deplacements signes en counts depuis le dernier reset_pos()
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
    cpm = COUNTS_PER_METER if COUNTS_PER_METER > 0 else 1
    return (pos_g + pos_d) / 2.0 / cpm

# =============================================================================
#  Commande moteur (translation) : x > 0 avance, x < 0 recule
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

# =============================================================================
#  Commande moteur (rotation pure, sens trigo = gauche)
#  w > 0 => tourne a gauche  (roue gauche recule, roue droite avance)
#  w < 0 => tourne a droite
# =============================================================================
def moteur_pivot(w_cmd):
    """
    Pivot pur en place.
    w_cmd est une commande angulaire normalisee [-MAX_SPEED, MAX_SPEED].
    Roue gauche : BACKWARD si w>0 (tourne a gauche), FORWARD sinon.
    Roue droite : FORWARD  si w>0,                   BACKWARD sinon.
    """
    spd = int(clamp(abs(w_cmd), 0, MAX_SPEED))
    if spd < MIN_SPEED and spd > 0:
        spd = MIN_SPEED
    if w_cmd >= 0:
        left_motor(BACKWARD, spd); right_motor(FORWARD,  spd)
    else:
        left_motor(FORWARD,  spd); right_motor(BACKWARD, spd)

# =============================================================================
#  Regulateur PID generique
# =============================================================================
class PID:
    def __init__(self, kp, ki, kd, max_speed=MAX_SPEED):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_max = max_speed / ki if ki > 0 else 0
        self.reset(0.0)

    def reset(self, error):
        self.integral    = 0.0
        self.prev_error  = error
        self.prev_time   = running_time()

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
#  Modele unicycle - pivot 90° sur place
#  Angle mesure via encodeurs :
#    dtheta = (delta_droite - delta_gauche) / WHEEL_BASE_COUNTS  [radians]
#  Les encodeurs sont resetes avant d'entrer dans cette fonction.
#  Sens positif = rotation a gauche (CCW vu du dessus).
# =============================================================================
def pivoter_90():
    """
    Fait pivoter le robot de +90° (sens direct, gauche) en utilisant le
    modele unicycle et un regulateur PID sur l'angle mesure par encodeurs.
    Retourne False si l'utilisateur a appuye sur A ou B pendant la rotation.
    """
    reset_pos()                     # remet pos_g / pos_d a zero
    pid_rot = PID(KP_ROT, KI_ROT, KD_ROT)
    display.show(Image.ARROW_W)

    while not (button_a.was_pressed() or button_b.was_pressed()):
        update_pos()

        # --- Modele unicycle : angle integre via encodeurs ---
        # delta_d (pos_d) positif si la roue droite avance
        # delta_g (pos_g) positif si la roue gauche avance
        # theta = (delta_d - delta_g) / L_counts  (rad)
        theta = (pos_d - pos_g) / float(WHEEL_BASE_COUNTS)

        erreur_rot = TURN_TARGET - theta   # reste a tourner (rad)

        if abs(erreur_rot) <= TOLERANCE_RAD:
            stop()
            sleep(200)             # courte pause de stabilisation
            return True            # rotation terminee

        # Commande PID en rad/s, scalee vers [MIN_SPEED, MAX_SPEED]
        w_cmd = pid_rot.update(erreur_rot)
        w_cmd = clamp(w_cmd, -MAX_SPEED, MAX_SPEED)
        moteur_pivot(w_cmd)
        sleep(LOOP_MS)

    stop()
    return False                   # interrompu par bouton

# =============================================================================
#  Phase de translation avec PID (meme logique que l'original)
#  Retourne False si interrompu par bouton.
# =============================================================================
def avancer_vers(cible_m):
    reset_pos()
    pid = PID(KP, KI, KD)
    display.show(Image.ARROW_N)

    while not (button_a.was_pressed() or button_b.was_pressed()):
        update_pos()
        erreur = cible_m - position_m()
        if abs(erreur) <= TOLERANCE_M:
            stop()
            pid.reset(erreur)
            display.show(Image.YES)
            sleep(300)
            return True
        moteur(applique_min(pid.update(erreur)))
        sleep(LOOP_MS)

    stop()
    return False

# =============================================================================
#  Sequence complete : avancer -> pivoter -> avancer -> maintenir
# =============================================================================
def sequence_avec_virage():
    # --- Phase 1 : avancer jusqu'a mi-chemin ---
    display.show(Image.ARROW_N)
    if not avancer_vers(HALF_M):
        return

    # --- Phase 2 : pivot 90° (modele unicycle) ---
    if not pivoter_90():
        return

    # --- Phase 3 : avancer la seconde moitie et maintenir la position ---
    display.show(Image.ARROW_N)
    reset_pos()
    pid = PID(KP, KI, KD)

    while not (button_a.was_pressed() or button_b.was_pressed()):
        update_pos()
        erreur = HALF_M - position_m()
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
        sequence_avec_virage()
        display.show(Image.ARROW_E)
    sleep(50)