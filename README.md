# Objectif 1 — Avancer exactement 1 mètre avec un PID

Robot **Maqueen Plus** piloté par une carte **micro:bit** (MicroPython).
On utilise les **codeurs de roue** (encodeurs) pour mesurer la distance, et un
**régulateur PID** qu'on écrit nous-mêmes pour s'arrêter pile à 1,00 m.

## Fichiers

| Fichier         | Rôle |
|-----------------|------|
| `maqueen.py`    | Le « pilote » : parle au robot en I2C (moteurs, codeurs). À téléverser une fois. |
| `calibrer.py`   | **À lancer en premier** : mesure `COUNTS_PER_METER` pour ton robot. |
| `main.py`       | Le programme de l'Objectif 1 : le PID qui fait 1 m. |

## Comment ça marche (en 3 phrases)

1. Les codeurs comptent combien les roues tournent → on en déduit la **distance parcourue**.
2. L'**erreur** = `1 m − distance déjà parcourue`. Le PID transforme cette erreur en **vitesse**.
3. Plus on s'approche de 1 m, plus l'erreur est petite, plus on ralentit → arrêt précis.

### Les 3 termes du PID (intuition)

- **P** (proportionnel, `KP`) : « je suis loin → je vais vite ; je suis proche → je vais lentement ». C'est le moteur principal du comportement.
- **I** (intégral, `KI`) : rattrape une **petite erreur qui s'éternise** (ex : le robot s'arrête toujours 2 cm trop tôt). On le laisse à `0` au début.
- **D** (dérivé, `KD`) : **freine** quand on approche vite de la cible → évite de **dépasser** (overshoot).

## Procédure complète

### Étape 1 — Calibration (indispensable)
1. Téléverse `maqueen.py` **et** `calibrer.py` (en le renommant/utilisant comme programme principal).
2. Marque la position de départ au sol, appuie sur le bouton **A**.
3. Le robot avance ~3 s puis affiche `C:xxxx` (counts comptés).
4. Mesure à la règle la distance **réelle** parcourue, en cm.
5. Calcule :
   ```
   COUNTS_PER_METER = counts / (distance_cm / 100)
   ```
   Exemple : `740` counts pour `10` cm → `740 / 0.10 = 7400`.
6. Recopie ce nombre dans `main.py` (variable `COUNTS_PER_METER`).

> Refais la mesure 2–3 fois et prends la moyenne : c'est plus fiable.

### Étape 2 — Le 1 mètre
1. Téléverse `maqueen.py` **et** `main.py`.
2. Pose le robot, appuie sur **A**.
3. Il avance et s'arrête. L'écran affiche la distance parcourue en cm (pour vérifier).
4. Mesure au mètre : tu dois être très proche de 100 cm.

## Réglage du PID (tuning)

À faire **dans l'ordre**, en testant à chaque changement :

1. **Mets `KI = 0` et `KD = 0`.** Ne garde que `KP`.
   - Augmente `KP` jusqu'à ce que le robot atteigne presque la cible.
   - Trop petit → il s'arrête trop tôt (manque de force en fin de course).
   - Trop grand → il **dépasse** puis oscille.
2. **Ajoute `KD`** (commence petit, ex. 10, 20, 40…) pour **amortir** le dépassement : le robot doit arriver « en douceur » sans osciller.
3. **Si** il s'arrête systématiquement un peu trop tôt/tard, ajoute **un tout petit** `KI` (ex. 1, 2, 5…) pour effacer cette erreur résiduelle.

Autres réglages utiles :
- `MIN_SPEED` : si le robot **cale** en fin de course (vitesse trop faible pour bouger), augmente-le un peu. S'il dépasse, baisse-le.
- `MAX_SPEED` : vitesse de croisière. Plus haut = plus rapide mais plus dur à arrêter net.
- `KP_STRAIGHT` : si le robot **tire à gauche ou à droite**, ajuste-le pour qu'il roule droit.

## Aller plus loin : le PID interne (registre `0x0A`)

Le Maqueen a son **propre** PID intégré (`maqueen.set_internal_pid(True)`).
Ce n'est PAS le même que le nôtre :

- **PID interne** = PID de **vitesse** : il maintient une **vitesse de rotation** constante et identique sur les deux roues (corrige batterie, frottements…).
- **Notre PID** = PID de **distance** : il décide **quand s'arrêter** (à 1 m).

Pour l'Objectif 1 on le laisse **désactivé** pour bien montrer que c'est notre
PID qui travaille. Tu peux ensuite l'activer (les deux se complètent : on appelle
ça une commande « en cascade ») pour un mouvement plus régulier.

## Téléverser plusieurs fichiers sur la micro:bit

Avec l'éditeur **python.microbit.org** ou **Mu** : ajoute `maqueen.py` comme
fichier du projet (panneau des fichiers), garde `main.py` comme programme
principal, puis flashe. Les deux partent ensemble sur la carte.
