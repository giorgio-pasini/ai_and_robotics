# test_direction.py
# Affiche en continu le registre de direction de la roue gauche (0 stop / 1 avant
# / 2 arriere) et l'encodeur. Aucun moteur n'est commande.
#
# Manipulation : tourne la roue gauche A LA MAIN, en avant puis en arriere, et
# observe le chiffre de direction.
#   - s'il passe a 1 quand tu tournes en avant et a 2 en arriere -> la direction
#     est detectee : on peut savoir le sens d'une poussee.
#   - s'il reste a 0 -> seule la commande moteur fixe la direction (le sens d'une
#     poussee main ne peut venir que du programme).
from microbit import i2c, display, button_a, sleep

ADDR = 0x10

def lire4(reg):
    i2c.write(ADDR, bytes([reg]))
    return i2c.read(ADDR, 4)

while True:
    d = lire4(0x00)          # directions (octet0 = gauche, octet2 = droite)
    e = lire4(0x04)          # encodeurs
    enc_g = (e[0] << 8) | e[1]
    # appui sur A : montre l'encodeur ; sinon montre la direction gauche
    if button_a.is_pressed():
        display.scroll(str(enc_g))
    else:
        display.show(str(d[0]))
    sleep(150)
