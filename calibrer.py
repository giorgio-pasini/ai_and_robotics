# calibrer.py
# -----------------------------------------------------------------------------
# ETAPE OBLIGATOIRE AVANT main.py : trouver COUNTS_PER_METER pour TON robot.
#
# Pourquoi ? Les codeurs comptent des "counts", pas des metres. Le lien entre
# les deux depend du diametre des roues et de l'electronique. Il faut donc le
# mesurer une fois.
#
# Mode d'emploi :
#   1. Pose le robot, marque la position de l'avant (scotch / regle au sol).
#   2. Televerse ce fichier comme main.py et appuie sur A.
#   3. Le robot avance ~3 secondes en ligne droite puis s'arrete.
#   4. Il affiche "C:xxxx" = le nombre de counts comptes.
#   5. Mesure a la regle la distance reellement parcourue (en cm).
#   6. Calcule :   COUNTS_PER_METER = counts / (distance_cm / 100)
#      Ex : 740 counts pour 10 cm  ->  740 / 0.10 = 7400 counts/metre.
#   7. Recopie ce nombre dans main.py (variable COUNTS_PER_METER).
# -----------------------------------------------------------------------------
from microbit import sleep, running_time, button_a, display, Image
import maqueen

SPEED    = 80       # vitesse constante pendant la mesure
DUREE_MS = 3000     # duree de la marche en avant

maqueen.set_internal_pid(False)
maqueen.stop()

display.show(Image.ARROW_E)
while not button_a.is_pressed():
    sleep(50)
display.show(Image.YES)

start_l = maqueen.enc_left()
start_r = maqueen.enc_right()

maqueen.drive(maqueen.FORWARD, SPEED)
t0 = running_time()
while running_time() - t0 < DUREE_MS:
    sleep(20)
maqueen.stop()

dl = maqueen.enc_left()  - start_l
dr = maqueen.enc_right() - start_r
counts = (dl + dr) // 2

# "C:" = counts moyens. Mesure la distance reelle et fais le calcul ci-dessus.
display.scroll("C:" + str(counts))
