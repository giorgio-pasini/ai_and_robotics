# maqueen.py
# -----------------------------------------------------------------------------
# Petit "pilote" du robot Maqueen Plus, piloté par une carte micro:bit.
# On parle au robot via le bus I2C (voir la derniere page du PDF du cours).
#
# Idee : le Maqueen est un "esclave" I2C a l'adresse 0x10. Pour agir dessus,
# on ecrit dans des "registres" (des petites cases memoire numerotees).
# Ex : pour la vitesse du moteur gauche, on ecrit dans le registre 0x01.
# -----------------------------------------------------------------------------
from microbit import i2c

# Adresse I2C du Maqueen Plus (le "0x10" de la derniere page du PDF)
ADDR = 0x10

# --- Numeros des registres (recopies de la table du PDF) ---------------------
REG_LEFT_DIR    = 0x00   # moteur gauche : direction
REG_LEFT_SPEED  = 0x01   # moteur gauche : vitesse 0-255
REG_RIGHT_DIR   = 0x02   # moteur droit  : direction
REG_RIGHT_SPEED = 0x03   # moteur droit  : vitesse 0-255
REG_ENC_LEFT    = 0x04   # codeur roue gauche  (2 octets : 0x04 + 0x05)
REG_ENC_RIGHT   = 0x06   # codeur roue droite  (2 octets : 0x06 + 0x07)
REG_PID_SWITCH  = 0x0A   # PID INTERNE du Maqueen (0 = off, 1 = on)

# Valeurs de direction (table du PDF : 0 arret, 1 avant, 2 arriere)
STOP     = 0
FORWARD  = 1
BACKWARD = 2


def _write(reg, *values):
    """Ecrit dans le robot : 1 octet d'adresse de registre + des données.
    Ex : _write(0x00, 1, 120) -> registre 0x00 = 1, registre 0x01 = 120
    (les registres se remplissent l'un apres l'autre automatiquement)."""
    i2c.write(ADDR, bytes([reg]) + bytes(values))


def _read_u16(reg):
    """Lit 2 octets a partir d'un registre et les assemble en un seul nombre.
    Sert a lire les codeurs (le compteur est sur 2 octets)."""
    i2c.write(ADDR, bytes([reg]))      # 1) on dit "je veux lire a partir de reg"
    data = i2c.read(ADDR, 2)           # 2) on lit 2 octets
    return (data[0] << 8) | data[1]    # 3) octet de poids fort + octet de poids faible


# --- Moteurs -----------------------------------------------------------------
def left_motor(direction, speed):
    _write(REG_LEFT_DIR, direction, speed)

def right_motor(direction, speed):
    _write(REG_RIGHT_DIR, direction, speed)

def drive(direction, speed):
    """Les deux moteurs en meme temps."""
    left_motor(direction, speed)
    right_motor(direction, speed)

def stop():
    drive(STOP, 0)


# --- Codeurs (encodeurs) de roue ---------------------------------------------
def enc_left():
    return _read_u16(REG_ENC_LEFT)

def enc_right():
    return _read_u16(REG_ENC_RIGHT)


# --- PID interne du Maqueen (optionnel) --------------------------------------
def set_internal_pid(on):
    """Active (True) ou desactive (False) le PID de VITESSE integre au robot.
    Pour l'Objectif 1 on le laisse desactive : c'est NOTRE PID qui pilote."""
    _write(REG_PID_SWITCH, 1 if on else 0)
