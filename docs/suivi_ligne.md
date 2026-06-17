# Étape « Suivi de ligne » — Suivre une ligne noire (PID)

## Objectif

Faire suivre au robot **Maqueen Plus** une **ligne noire sur fond blanc** (le
**cercle** et la **croix** de la carte du tutoriel MBT0021, Projets 1 et 2),
en utilisant ses **capteurs infrarouges de suivi de ligne** et un régulateur
**PID** pour le braquage.

## Matériel

- Robot **Maqueen Plus**, piloté par une carte **micro:bit** (MicroPython).
- Carte avec un **cercle noir** et une **croix noire** sur papier blanc.
- Communication carte ↔ robot par le **bus I2C** (adresse du robot : `0x10`).

## Les capteurs de suivi de ligne

Le Maqueen Plus possède **6 capteurs** infrarouges sous l'avant, de gauche à
droite : `L3 L2 L1 R1 R2 R3`. Chaque capteur dit s'il voit du **noir** ou du
**blanc**.

Deux façons de les lire (cf. `Adresses.pdf`) :

| Méthode | Registre | Valeur |
|---------|----------|--------|
| **Digitale** (utilisée ici) | `0x1D` | 1 octet, **1 bit par capteur** : `1` = noir, `0` = blanc |
| Niveau de gris | `0x1E`→`0x2A` | 2 octets par capteur, plage `0`–`4095` |

On utilise la méthode **digitale** : c'est la carte qui fait le seuillage
noir/blanc.

> ⚠️ **Polarité inversée sur notre robot.** Le PDF annonce « 1 = noir », mais en
> pratique un capteur **sur le noir renvoie un bit à 0** (le blanc → 1). Le code
> en tient compte via `LIGNE_EST_BIT_0 = True` et l'aide `sur_la_ligne()`.
> Vérifiable au mode TEST : poser du noir sous un capteur éteint son bit.

### Mapping des bits du registre `0x1D`

```
bit5=0x20 → L3   bit4=0x10 → L2   bit3=0x08 → L1
bit2=0x04 → R1   bit1=0x02 → R2   bit0=0x01 → R3
```

Vérifié par l'exemple `b'0000 1100'` de la doc : c'est `L1 + R1` allumés, soit
le robot **parfaitement centré** sur la ligne.

### Tous les capteurs, mais les externes prioritaires

On utilise les **6 capteurs**. Une première version n'utilisait que les 4
centraux : le robot **décrochait** (il faisait un tour sans suivre la ligne).
Deux raisons :

- avec une ligne **épaisse** (l'anneau du cercle), les capteurs centraux
  restent **tous sur le noir** → erreur ≈ 0 → le robot va tout droit et sort
  de la courbe ;
- n'utiliser que les 2 **externes** créerait au contraire une **zone morte** au
  centre (quand la ligne passe entre eux, personne ne la voit).

La solution : garder les 6 capteurs, mais donner aux **externes `L3`/`R3` le
poids le plus fort**. Ils dominent l'erreur dès que la ligne s'éloigne (rôle
« alarme grand écart »), sans laisser de trou au centre.

## Principe : erreur de position → PID → braquage

À chaque cycle, le robot :

1. **lit** l'état des capteurs (`0x1D`) ;
2. en déduit une **erreur** = position de la ligne par rapport au centre ;
3. le **PID** transforme cette erreur en **correction de braquage** ;
4. on **applique** la correction : la roue du côté de la ligne ralentit, la
   **roue opposée accélère**, donc le robot pivote pour se recentrer ;
5. recommence.

### Calcul de l'erreur (moyenne pondérée)

Chaque capteur a un **poids** selon sa position (externes = poids fort) :

```
L3 = -5   L2 = -3   L1 = -1   R1 = +1   R2 = +3   R3 = +5
```

`erreur = (somme des poids des capteurs voyant du noir) / (nombre de capteurs noirs)`

- `erreur < 0` → la ligne est **à gauche** → tourner à gauche.
- `erreur > 0` → la ligne est **à droite** → tourner à droite.
- `erreur ≈ 0` (p. ex. `L1` + `R1`) → robot centré → tout droit.
- **aucun** capteur noir → **ligne perdue** (voir ci-dessous).

### Le régulateur PID

```
correction = KP·erreur + KI·∫erreur·dt + KD·(Δerreur / dt)
```

| Terme | Rôle en suivi de ligne |
|-------|------------------------|
| **P** | corrige proportionnellement au décalage de la ligne |
| **I** | rattrape un biais persistant (souvent laissé à 0) |
| **D** | amortit les oscillations (évite de zigzaguer) |

## Architecture en cascade : 2 niveaux de PID

Au lieu d'envoyer directement un PWM aux moteurs, on régule en **cascade** :

```
ligne → [PID braquage] → consignes de vitesse → [PID vitesse G] → PWM G
                                              ↘ [PID vitesse D] → PWM D
                                                     ▲   ▲
                                                  encodeurs (mesure)
```

### 1) PID de braquage (externe — 1 seul)

- **Entrée** : l'erreur de position de la ligne (±5).
- **Sortie** : une **correction** ajoutée/retranchée à la vitesse de croisière,
  exprimée en **counts/seconde** :

```
consigne_gauche = BASE_CPS + correction
consigne_droite = BASE_CPS − correction
```

Réglages : `KP_S = 28`, `KI_S = 0`, `KD_S = 18` ; `BASE_CPS = 150` (~0,25 m/s).

### 2) PID de vitesse par roue (interne — 1 PAR ROUE = 2 PID)

Chaque roue a son PID qui **asservit sa vitesse réelle** (mesurée à l'encodeur)
sur la consigne donnée par le braquage :

```
erreur_vitesse = consigne − vitesse_mesurée
PWM = FF·consigne + KP_W·erreur_vitesse + KI_W·∫erreur_vitesse
```

- **FF** (feedforward) : PWM approximatif pour atteindre la consigne ; donne le
  gros de la commande, le PI corrige le reste.
- **Pourquoi ?** garantit que chaque roue tourne **vraiment** à la vitesse
  demandée — quelles que soient la charge, l'usure, la pente ou la batterie.
  Deux moteurs jamais identiques deviennent **égaux**, donc trajectoire propre
  et **rayon de virage constant**.

La vitesse réelle vient des **encodeurs** (`0x04`/`0x06`, magnitude) signés par
le **registre de direction** (`0x00`/`0x02`), exactement comme dans `robot_1m`.

Réglages de départ (**à affiner sur le robot**) : `FF_W = 0.55`, `KP_W = 0.40`,
`KI_W = 1.20`. Si les roues sont molles → monter `FF_W` ; si elles oscillent →
baisser `KP_W`.

### Marche arrière pour les virages serrés

Une consigne de roue peut devenir **négative** : la roue intérieure **recule**
au lieu de juste ralentir → pivot serré, le robot ne décroche pas dans les
courbes prononcées.

## Ligne perdue : recherche

Si **aucun** capteur ne voit du noir, le robot **pivote** vers le dernier côté
où la ligne a été vue (`dernier_signe`), à vitesse réduite, jusqu'à la
retrouver. Indispensable dans les **virages serrés** (cercle, coins de la croix).

## Autres approches possibles

| Approche | Idée | Compromis |
|----------|------|-----------|
| **Direct (1 PID)** | PID de braquage → PWM directement (pas d'asservissement de vitesse). | Plus simple, mais une roue plus faible que l'autre dévie la trajectoire. |
| **PID interne du robot** | Activer le PID de vitesse intégré (`reg 0x0A = 1`) au lieu d'écrire le nôtre. | Zéro code de vitesse, mais « boîte noire » non réglable et non pédagogique. |
| **Cascade (retenue)** | PID braquage + 1 PID vitesse **par roue** (encodeurs). | Notre choix : trajectoire précise, vitesse maîtrisée ; plus de réglages. |

Côté **capteurs**, l'alternative est l'**analogique (niveau de gris)** : lire
les registres `0x1E`→`0x2A` (0–4095 par capteur) au lieu du registre digital
`0x1D`. La position de la ligne devient **continue** (encore plus lisse), mais
il faut **calibrer** le blanc/noir de chaque capteur. La version actuelle
(digitale) évite cette calibration.

## Retours visuels

- **LED RGB** : **vert** = ligne suivie, **rouge** = recherche (ligne perdue).
- **Écran micro:bit** : `←` / `↑` / `→` selon le braquage, `✗` si ligne perdue.

## Fichiers

| Fichier | Rôle |
|---------|------|
| `suivi_ligne.py` | Programme final : lecture capteurs + erreur + PID + recherche. |

## Utilisation

1. Téléverser `suivi_ligne.py` ; allumer les piles du robot.
2. **Bouton B → mode TEST** : passer le robot au-dessus de la ligne et vérifier
   que les bons capteurs s'allument (et l'octet `0x1D` sur la console série).
   Si au suivi le robot **braque à l'envers** (s'éloigne de la ligne), mettre
   `INVERSE_LR = True` dans le code. Re-appuyer sur B pour sortir.
3. Poser le robot **sur la ligne**, **bouton A** → il suit la ligne.
4. Nouvel appui sur **A** → arrêt.

## Réglages à ajuster sur le terrain

- `BASE_CPS` : vitesse de croisière (counts/s) — baisser si le robot rate les virages.
- `KP_S` : braquage — augmenter si réaction trop molle, baisser s'il zigzague.
- `KD_S` : braquage — augmenter pour amortir le zigzag.
- `FF_W`, `KP_W`, `KI_W` : PID de vitesse par roue (FF d'abord, puis KI, puis KP).
- `SEARCH_TURN_CPS` / `SEARCH_BASE_CPS` : comportement quand la ligne est perdue.
