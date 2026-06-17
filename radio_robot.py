# =============================================================================
#  Robot Maqueen Plus + micro:bit (MicroPython)
#  ETAPE 9 : RADIO -- cote ROBOT (recepteur).
#
#  A FLASHER SUR LA micro:bit MONTEE SUR LE ROBOT.
#
#  Au demarrage, l'ecran propose : "A=GO  B=TEST"
#     - Bouton A -> MODE RADIO   : pilotage par la telecommande.
#     - Bouton B -> MODE TEST    : le robot annonce et execute chaque mouvement
#                                  TOUT SEUL (sans telecommande, sans cable),
#                                  pour verifier le sens des moteurs.
#
#  >>> ON NE DEVINE PLUS LE CABLAGE : LE MODE TEST LE MONTRE. <<<
#  Lance le MODE TEST, regarde le robot, puis regle les 2 interrupteurs
#  ci-dessous si besoin, et re-flashe.
#
#  Protocole radio : 1 lettre + vitesse (3 chiffres).
#     "F128" avancer   "L128" gauche   "R128" droite   "S000" stop
# =============================================================================
from microbit import i2c, display, Image, button_a, button_b, sleep, running_time
import radio

# =============================================================================
#  I2C Maqueen (adresse 0x10) -- valeurs BRUTES du PDF : 1 = avant, 2 = arriere
# =============================================================================
ADDR = 0x10
STOP, FORWARD, BACKWARD = 0, 1, 2

def _ecrire(data):
    # Ecriture I2C "blindee" : un glitch passager (OSError) ne doit PAS tuer
    # le programme (sinon l'ecran affiche "Line X / error" et le robot se fige).
    try:
        i2c.write(ADDR, bytes(data))
    except OSError:
        pass

def left_motor(direction, speed):  _ecrire([0x00, direction, speed])
def right_motor(direction, speed): _ecrire([0x02, direction, speed])
def stop():                        left_motor(STOP, 0); right_motor(STOP, 0)
def set_internal_pid(on):          _ecrire([0x0A, 1 if on else 0])

# =============================================================================
#  >>> LES 2 SEULS REGLAGES A TOUCHER (avec le MODE TEST, bouton B) <<<
#
#  1) Lance le MODE TEST. Quand l'ecran dit "AVANT", regarde le robot :
#        - il avance pour de vrai  -> laisse INVERSER_AVANT_ARRIERE = False
#        - il RECULE               -> mets  INVERSER_AVANT_ARRIERE = True
#  2) Quand l'ecran dit "GAUCHE", regarde le robot :
#        - il tourne a gauche      -> laisse INVERSER_GAUCHE_DROITE = False
#        - il tourne a DROITE      -> mets  INVERSER_GAUCHE_DROITE = True
#  Puis re-flashe le robot. (Les 2 reglages sont independants.)
# =============================================================================
INVERSER_AVANT_ARRIERE = False
INVERSER_GAUCHE_DROITE = False

# Sens REELS apres correction (calcules a partir des 2 interrupteurs).
AVANT   = BACKWARD if INVERSER_AVANT_ARRIERE else FORWARD
ARRIERE = FORWARD  if INVERSER_AVANT_ARRIERE else BACKWARD

# =============================================================================
#  Configuration RADIO  (MEME groupe que la telecommande, DIFFERENT des voisins)
# =============================================================================
GROUPE       = 77
TIMEOUT_MS   = 300    # securite : stop si rien recu depuis ce delai (ms)
VITESSE_TEST = 120    # vitesse utilisee par le MODE TEST
RATIO_VIRAGE = 0.35   # roue INTERIEURE en virage = 35% de la vitesse (0=pivot serre, 1=tout droit)

radio.config(group=GROUPE, queue=3, length=8)
radio.on()

# =============================================================================
#  Mouvements de base (utilises par le radio ET par le test)
# =============================================================================
# Petit point central : "lien radio vivant mais ordre = stop".
POINT = Image("00000:00000:00900:00000:00000")

def clamp(v):
    return max(0, min(255, v))

def avancer(v):
    left_motor(AVANT, v); right_motor(AVANT, v)

def tourner_gauche(v):
    # COURBE a gauche : les 2 roues AVANCENT, la roue gauche (interieure) plus lente.
    lent = clamp(int(v * RATIO_VIRAGE))
    if INVERSER_GAUCHE_DROITE:
        left_motor(AVANT, v);    right_motor(AVANT, lent)
    else:
        left_motor(AVANT, lent); right_motor(AVANT, v)

def tourner_droite(v):
    # COURBE a droite : roue droite (interieure) plus lente.
    lent = clamp(int(v * RATIO_VIRAGE))
    if INVERSER_GAUCHE_DROITE:
        left_motor(AVANT, lent); right_motor(AVANT, v)
    else:
        left_motor(AVANT, v);    right_motor(AVANT, lent)

def appliquer(cmd, vitesse):
    vitesse = clamp(vitesse)
    if cmd == "F":
        avancer(vitesse);        display.show(Image.ARROW_N)
    elif cmd == "L":
        tourner_gauche(vitesse); display.show(Image.ARROW_W)
    elif cmd == "R":
        tourner_droite(vitesse); display.show(Image.ARROW_E)
    else:                        # "S" ou inconnu : recu mais a l'arret
        stop();                  display.show(POINT)

# =============================================================================
#  MODE RADIO -- pilotage par la telecommande (bouton B pour ressortir)
# =============================================================================
def mode_radio():
    stop()
    display.show(Image.NO)        # ✗ tant qu'on n'a rien recu
    dernier_recu = running_time()
    while not button_b.was_pressed():
        # On VIDE la file et on ne garde que le DERNIER message recu :
        # ainsi le robot obeit toujours a l'ordre le plus recent (pas de retard
        # qui le laisse "coince" sur un vieux virage).
        msg = radio.receive()
        recent = None
        while msg is not None:
            recent = msg
            msg = radio.receive()

        if recent is not None:
            try:
                vitesse = int(recent[1:])
            except ValueError:
                vitesse = 0
            appliquer(recent[0], vitesse)
            dernier_recu = running_time()
        elif running_time() - dernier_recu > TIMEOUT_MS:
            stop()
            display.show(Image.NO)        # plus de signal -> stop de securite
            dernier_recu = running_time()
        sleep(5)
    stop()

# =============================================================================
#  MODE TEST -- le robot annonce puis fait chaque mouvement, TOUT SEUL.
#  Sert a regler les 2 interrupteurs en regardant le robot (bouton A pour sortir)
# =============================================================================
def jouer(label, fonction):
    display.scroll(label, delay=70)     # annonce a l'ecran le mouvement ATTENDU
    fonction(VITESSE_TEST)
    sleep(1500)                         # ... puis on le fait pour de vrai
    stop()
    sleep(700)

def mode_test():
    while not button_a.was_pressed():
        jouer("AVANT", avancer)
        jouer("GAUCHE", tourner_gauche)
        jouer("DROITE", tourner_droite)
        display.show(Image.YES); sleep(400)
    stop()

# =============================================================================
#  Programme principal
#  -> On demarre DIRECTEMENT en mode radio (pas de menu bloquant).
#     Bouton B : passer en mode TEST.   Bouton A : revenir au mode radio.
#
#  Indications a l'ecran en mode radio (DEBUG reception) :
#     ↑ ← →  = ordre recu (avancer / gauche / droite)
#     point  = recu mais "stop" (le lien radio est vivant)
#     ✗      = rien recu (probleme radio ou telecommande eteinte)
# =============================================================================
set_internal_pid(True)   # PID interne ON : plus de couple / vitesse reguliere
stop()

while True:
    mode_radio()    # tourne jusqu'a l'appui sur B
    mode_test()     # tourne jusqu'a l'appui sur A, puis on revient au radio
