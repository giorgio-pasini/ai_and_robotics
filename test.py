# test.py
# =============================================================================
#  TEST DE TELEVERSEMENT - le plus simple possible.
#  Aucun moteur, aucun I2C : juste l'ecran de la micro:bit.
#  But : verifier que le code part bien sur la carte.
#
#  Si ca marche, tu dois voir :
#     1) le mot "OK" defiler des le demarrage,
#     2) puis un coeur qui clignote en boucle,
#     3) si tu appuies sur A : une coche ; sur B : une croix.
# =============================================================================
from microbit import display, Image, button_a, button_b, sleep

# 1) Au demarrage : on fait defiler "OK" (preuve que le code tourne)
display.scroll("OK")

# 2) Ensuite, boucle infinie
while True:
    if button_a.is_pressed():
        display.show(Image.YES)        # coche si bouton A
    elif button_b.is_pressed():
        display.show(Image.NO)         # croix si bouton B
    else:
        display.show(Image.HEART)      # sinon : coeur qui bat
        sleep(400)
        display.show(Image.HEART_SMALL)
        sleep(400)
