# =============================================================================
#  Robot Maqueen Plus v1 + micro:bit (MicroPython)
#  ETAPE 9 : RADIO -- cote DONGLE (pont SERIE -> RADIO).
#
#  A FLASHER SUR UNE micro:bit BRANCHEE EN USB SUR LE MAC.
#
#  Role : lire les commandes envoyees par l'ordi sur le port serie (USB) et les
#  rediffuser TELLES QUELLES au robot par radio. Le robot (radio_robot.py) ne
#  voit aucune difference avec la telecommande a main : meme protocole.
#
#  Source des commandes cote ordi : radio_remote.html (Web Serial, Chrome/Edge)
#  ou n'importe quel script qui ecrit sur le port serie.
#
#  Protocole (IDENTIQUE au robot) : 1 lettre + vitesse 3 chiffres, termine par
#  un retour a la ligne "\n".  Ex : "F207\n"  "L150\n"  "S000\n".
#
#  /!\ channel + group DOIVENT etre IDENTIQUES a radio_robot.py.
# =============================================================================
from microbit import uart, display, Image, sleep
import radio

# =============================================================================
#  Configuration RADIO  (IDENTIQUE au robot : channel 7, group 77)
# =============================================================================
RADIO_CHANNEL = 7
RADIO_GROUP   = 77
radio.config(channel=RADIO_CHANNEL, group=RADIO_GROUP, power=7, length=8)
radio.on()

# Le port serie USB sert maintenant a recevoir les commandes de l'ordi.
# (Apres uart.init, le REPL n'est plus dispo sur ce port : c'est normal.)
uart.init(baudrate=115200)

display.show(Image.ARROW_E)   # pret, en attente du port serie

# =============================================================================
#  Boucle : on accumule les octets recus, on decoupe par lignes "\n",
#  et on rediffuse chaque ligne complete par radio.
# =============================================================================
buffer = ""
while True:
    if uart.any():
        buffer += str(uart.read(), "UTF-8")
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if line:
                radio.send(line)
                # petit feedback ecran selon la direction relayee
                c = line[0]
                if   c == "F": display.show(Image.ARROW_N)
                elif c == "B": display.show(Image.ARROW_S)
                elif c == "L": display.show(Image.ARROW_W)
                elif c == "R": display.show(Image.ARROW_E)
                else:          display.show(Image.SQUARE_SMALL)   # "S" / autre
    sleep(2)
