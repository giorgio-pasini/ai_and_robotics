# =============================================================================
#  Robot Maqueen Plus v1 + micro:bit (MicroPython)
#  ETAPE 9 : RADIO -- cote ROBOT (recepteur).
#
#  A FLASHER SUR LA micro:bit MONTEE SUR LE ROBOT.
#
#  Demarre directement en MODE RADIO (pilotage par la telecommande).
#     - Bouton B -> MODE TEST  : le robot annonce + execute chaque mouvement
#                                TOUT SEUL (sans telecommande) pour verifier
#                                le sens des moteurs.
#     - Bouton A -> retour MODE RADIO.
#
#  Protocole radio : 1 lettre (direction) + vitesse (3 chiffres).
#     "F128" avancer  "B128" reculer  "L128" gauche  "R128" droite  "S000" stop
#  La vitesse est calculee par la telecommande (proportionnelle a l'inclinaison)
#  -> on peut accelerer/ralentir en temps reel.
#
#  Virages : en ARC (les 2 roues avancent, la roue interieure plus lente,
#  cf. RATIO_VIRAGE) -> trajectoire courbe, pas un pivot sur place.
#
#  Techniques reprises de l'approche du camarade (approche_radio_x/) :
#     - 1 seule ecriture I2C pour les 2 moteurs ([0x00, dG, vG, dD, vD]) ;
#     - attente i2c.scan() au boot ;
#     - radio.config(channel=..., power=7) pour une liaison fiable ;
#     - set_motors(left, right) a vitesse SIGNEE.
# =============================================================================
from microbit import display, Image, button_a, button_b, sleep, i2c, running_time
import radio

# =============================================================================
#  Configuration RADIO  (DOIT etre IDENTIQUE a radio_telecommande.py)
# =============================================================================
RADIO_CHANNEL = 7
RADIO_GROUP   = 77
radio.config(channel=RADIO_CHANNEL, group=RADIO_GROUP, power=7, queue=3, length=8)
radio.on()

# =============================================================================
#  Carte moteur Maqueen Plus v1 (adresse I2C 0x10)
#     registres : 0x00 dir G | 0x01 vit G | 0x02 dir D | 0x03 vit D
#     direction (PDF) : 0 = stop, 1 = avant, 2 = arriere
# =============================================================================
ADDR         = 0x10
WATCHDOG_MS  = 400    # securite : stop si rien recu depuis ce delai (ms)
VITESSE_TEST = 150    # vitesse utilisee par le MODE TEST
RATIO_VIRAGE = 0.40   # roue INTERIEURE en virage en COURBE = 40% de la vitesse.
                      #   plus petit -> courbe plus SERREE (plus de courbure)
                      #   plus grand  -> courbe plus large (proche tout droit)

# =============================================================================
#  >>> MODE MATCH / PERFORMANCE <<<
#  Pour la competition : on privilegie vitesse de pointe + reactivite.
# =============================================================================
PID_INTERNE  = False  # False = PUISSANCE BRUTE : + de vitesse de pointe et + reactif
                      #         (recommande MATCH). True = vitesse regulee (+ droit, - rapide).
VIRAGE_PIVOT = True   # True = virage SUR PLACE : reorientation instantanee (recommande MATCH).
                      #        False = virage en COURBE (utilise RATIO_VIRAGE, conduite douce).

# =============================================================================
#  >>> LES 2 SEULS REGLAGES A TOUCHER (avec le MODE TEST, bouton B) <<<
#  1) MODE TEST dit "AVANT" : le robot avance -> False ; il RECULE -> True.
#  2) MODE TEST dit "GAUCHE": il tourne a gauche -> False ; a DROITE -> True.
#  Les 2 reglages sont independants. Re-flashe apres modification.
# =============================================================================
INVERSER_AVANT_ARRIERE = False
INVERSER_GAUCHE_DROITE = False

# Petit point central : "lien radio vivant mais ordre = stop".
POINT = Image("00000:00000:00900:00000:00000")

# =============================================================================
#  Attendre que la carte moteur reponde (evite un plantage au demarrage).
# =============================================================================
while ADDR not in i2c.scan():
    display.show(Image.NO)
    sleep(100)

def set_internal_pid(on):
    # PID interne du robot (0x0A) : vitesse plus reguliere / plus de couple.
    try:
        i2c.write(ADDR, bytes([0x0A, 1 if on else 0]))
    except OSError:
        pass

# =============================================================================
#  Pilotage moteurs : UNE seule ecriture I2C pour les 2 moteurs.
#  left / right : vitesse SIGNEE (>0 avant, <0 arriere, 0 stop).
#  Les inversions sont appliquees ICI, donc partout (radio ET test).
# =============================================================================
def set_motors(left, right):
    if INVERSER_GAUCHE_DROITE:
        left, right = right, left
    if INVERSER_AVANT_ARRIERE:
        left, right = -left, -right
    ld = 1 if left  > 0 else 2 if left  < 0 else 0
    rd = 1 if right > 0 else 2 if right < 0 else 0
    ls = min(255, abs(int(left)))
    rs = min(255, abs(int(right)))
    try:
        # Un glitch I2C passager (OSError) ne doit PAS figer le robot.
        i2c.write(ADDR, bytes([0x00, ld, ls, rd, rs]))
    except OSError:
        pass

# Mouvements de base (utilises par le radio ET par le test).
def avancer(v):  set_motors(v,  v)
def reculer(v):  set_motors(-v, -v)
def stop():      set_motors(0,  0)

def tourner_gauche(v):
    if VIRAGE_PIVOT:
        set_motors(-v, v)              # PIVOT sur place : reorientation rapide (match)
    else:
        set_motors(int(v * RATIO_VIRAGE), v)   # ARC : les 2 roues avancent, gauche plus lente

def tourner_droite(v):
    if VIRAGE_PIVOT:
        set_motors(v, -v)              # PIVOT sur place
    else:
        set_motors(v, int(v * RATIO_VIRAGE))   # ARC : droite plus lente

def appliquer(cmd, vitesse):
    if   cmd == "F": avancer(vitesse);        display.show(Image.ARROW_N)
    elif cmd == "B": reculer(vitesse);        display.show(Image.ARROW_S)
    elif cmd == "L": tourner_gauche(vitesse); display.show(Image.ARROW_W)
    elif cmd == "R": tourner_droite(vitesse); display.show(Image.ARROW_E)
    else:            stop();                  display.show(POINT)   # "S"/inconnu

# =============================================================================
#  MODE RADIO -- pilotage par la telecommande (bouton B pour ressortir).
#  On VIDE la file et on n'execute que le DERNIER message : le robot obeit
#  toujours a l'ordre le plus recent (pas de retard sur un vieux virage).
# =============================================================================
def mode_radio():
    stop()
    display.show(Image.NO)        # X tant qu'on n'a rien recu
    dernier_recu = running_time()
    while not button_b.was_pressed():
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
        elif running_time() - dernier_recu > WATCHDOG_MS:
            stop()
            display.show(Image.NO)       # plus de signal -> stop de securite
            dernier_recu = running_time()
        sleep(5)
    stop()

# =============================================================================
#  MODE TEST -- le robot annonce puis fait chaque mouvement, TOUT SEUL.
#  Sert a regler les 2 interrupteurs en regardant le robot (bouton A pour sortir)
# =============================================================================
def jouer(label, fonction):
    display.scroll(label, delay=70)     # annonce le mouvement ATTENDU
    fonction(VITESSE_TEST)
    sleep(1500)                         # ... puis on le fait pour de vrai
    stop()
    sleep(700)

def mode_test():
    while not button_a.was_pressed():
        jouer("AVANT", avancer)
        jouer("ARRIERE", reculer)
        jouer("GAUCHE", tourner_gauche)
        jouer("DROITE", tourner_droite)
        display.show(Image.YES); sleep(400)
    stop()

# =============================================================================
#  Programme principal
#  -> boot direct en mode radio.  B : mode TEST.  A : retour radio.
#  Ecran (debug reception) : ^ v < >  = ordre recu / point = idle / X = rien.
# =============================================================================
set_internal_pid(PID_INTERNE)   # voir MODE MATCH : False = puissance brute (recommande)
stop()

while True:
    mode_radio()    # tourne jusqu'a l'appui sur B
    mode_test()     # tourne jusqu'a l'appui sur A, puis retour radio
