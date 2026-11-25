# Acquisition pour la calibration des BMI323

Ce dossier contient l'outil d'acquisition des accéléromètres et gyroscopes des
quatre patchs de la semelle gauche.

## Dépendance

```bash
python3 -m pip install pyserial
```

## Utilisation

Brancher l'ESP32 récepteur, allumer la semelle gauche, puis lancer depuis la
racine du dépôt. Par défaut, le script enregistre 30 poses statiques
arbitraires de 5 secondes :

```bash
python3 calibration_imu/acquire_orientations.py --port /dev/ttyUSB0
```

Le débit, le nombre de poses et leur durée peuvent être modifiés :

```bash
python3 calibration_imu/acquire_orientations.py \
  --port /dev/ttyUSB0 \
  --baud 2000000 \
  --pose-count 30 \
  --duration 5
```

Pour chaque pose :

1. placer la semelle dans une nouvelle orientation, nettement différente des
   précédentes ;
2. attendre qu'elle soit parfaitement immobile ;
3. appuyer une seule fois sur `s`, sans appuyer sur Entrée ;
4. ne plus toucher la semelle jusqu'à la fin de l'acquisition.

Le buffer d'entrée série est vidé au moment de l'appui sur `s`. Les octets
présents avant le déclenchement ne sont donc pas enregistrés.

Il faut répartir les poses dans toutes les directions 3D, sans chercher à
obtenir des angles précis. Les fichiers sont nommés `pose_01.csv` à
`pose_30.csv`.

L'ancien protocole des six orientations orthogonales reste accessible avec :

```bash
python3 calibration_imu/acquire_orientations.py \
  --port /dev/ttyUSB0 \
  --six-orientations \
  --duration 15
```

Les six positions correspondantes sont :

- `face_up` : face capteurs vers le haut ;
- `face_down` : face capteurs vers le bas ;
- `toes_up` : orteils vers le haut ;
- `heel_up` : talon vers le haut ;
- `medial_up` : bord intérieur du pied gauche vers le haut ;
- `lateral_up` : bord extérieur du pied gauche vers le haut.

## Fichiers produits

Une session est créée sous la forme :

```text
calibration_imu/data/session_YYYYMMDD_HHMMSS/
├── pose_01.bin
├── pose_01.csv
├── ...
├── pose_30.bin
├── pose_30.csv
└── metadata.json
```

Les CSV ne contiennent que les patchs gauches (`ID1` 1, 3, 5 et 7) et les
colonnes nécessaires à la calibration de l'accéléromètre et du gyroscope.
Les fichiers binaires sont conservés pour permettre un nouveau décodage sans
refaire l'expérience. `metadata.json` enregistre notamment l'instant UTC du
début et de la fin de chaque acquisition.

## Visualisation avant calibration

Le script suivant détecte automatiquement les fichiers `pose_*.csv` et crée
un graphique par pose, avec une colonne pour l'accéléromètre, une autre pour
le gyroscope et une ligne par patch :

```bash
python3 calibration_imu/plot_raw_orientations.py \
  calibration_imu/data/session_YYYYMMDD_HHMMSS
```

Les images sont enregistrées dans le sous-dossier `plots_raw` de la session.
Les valeurs affichées sont les données converties en `g` et en `°/s`, sans
correction de biais, d'échelle ou d'orientation.

Le même script reste compatible avec les anciennes sessions contenant
`face_up.csv`, `face_down.csv`, etc.

## Calibration des accéléromètres

La calibration multi-position complète (biais, échelles et non-orthogonalité)
se lance avec :

```bash
python3 calibration_imu/calibrate_accelerometers.py \
  calibration_imu/data/session_YYYYMMDD_HHMMSS
```

Pour chaque pose, le script recherche automatiquement une fenêtre statique
commune aux quatre patchs. Il applique ensuite le modèle de Tedaldi et al.
(2014) séparément à chaque BMI323 et sauvegarde dans `calibration_acc` :

- `accelerometer_calibration.json` : coefficients et matrices ;
- `stability_windows.csv` : fenêtres retenues et score de stabilité ;
- `pose_means_comparison.csv` : valeurs avant/après et validation croisée ;
- `acc_norm_before_after.png` : comparaison graphique avec la norme idéale de
  `1 g`.

## Acquisition pour les gyroscopes

La méthode complète du papier nécessite les mouvements entre deux positions
statiques. Le protocole dédié enregistre d'abord 50 secondes immobiles pour
le biais, puis 30 transitions guidées :

```bash
python3 calibration_imu/acquire_gyroscope.py --port /dev/ttyACM0 --initial-duration 2700
```

Pour chaque transition :

1. laisser la semelle immobile et appuyer sur `s` ;
2. attendre le signal `BOUGEZ` ;
3. effectuer une rotation vers une nouvelle orientation ;
4. au signal `IMMOBILISEZ`, poser la semelle et ne plus la toucher.

Un fichier `transition_XX.csv` contient donc une phase statique initiale, le
mouvement complet et une phase statique finale. Il faut varier les axes et
les amplitudes des rotations, sans donner de chocs à la semelle.

La calibration utilise ensuite les coefficients accélérométriques comme
référence :

```bash
python3 calibration_imu/calibrate_gyroscopes.py \
  calibration_imu/data/session_gyro_YYYYMMDD_HHMMSS \
  --acc-calibration \
  calibration_imu/data/session_YYYYMMDD_HHMMSS/calibration_acc/accelerometer_calibration.json
```

Les résultats sont enregistrés dans `calibration_gyro` : coefficients,
matrices, audit des transitions et comparaison des erreurs angulaires avant
et après calibration.

### Analyse d'Allan préliminaire

La phase statique initiale peut être analysée avec :

```bash
python3 calibration_imu/allan_variance_gyroscope.py \
  calibration_imu/data/session_gyro_YYYYMMDD_HHMMSS
```

Le script calcule la variance et la déviation d'Allan chevauchantes pour les
trois axes des quatre patchs. Par défaut, `tau` est limité à un dixième de la
durée totale afin de conserver un nombre raisonnable d'intervalles.


```bash
python3 calibration_imu/apply_imu_calibration.py datasemelle.csv \
  --show \
  --no-save-plots
```
