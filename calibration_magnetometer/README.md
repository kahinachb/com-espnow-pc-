# Calibration magneto MMC (memsic)

faire une acquisition avec differentes orientation avec piocom 
puis calibrate 
puis apply coef : 
python3 codePlusieursPatchs/applyMagnetometerCalibration.py \
    datasemelle.csv \
    --coefficients calibration_magnetometer_results/magnetometer_calibration.json \
    --output-csv resultats/datasemelle_calibrated.csv \
    --plot-dir resultats/plots



1. Éloigner la semelle des masses métalliques, de la plateforme de force, du téléphone, des enceintes et des alimentations.
2. Laisser la semelle complètement assemblée et alimentée normalement.
3. Démarrer l’enregistrement.
4. Faire lentement plusieurs rotations complètes autour des trois axes :
   - talon–orteils ;
   - gauche–droite ;
   - dessus–dessous.
5. Présenter successivement toutes les faces vers le haut :
   - dessus ;
   - dessous ;
   - talon ;
   - orteils ;
   - bord gauche ;
   - bord droit.
6. Faire des mouvements en huit en changeant simultanément l’inclinaison.
7. Ajouter des orientations diagonales et aléatoires.
8. Arrêter après environ 60 à 90 secondes.
Quelques précautions :
- effectuer les rotations lentement et régulièrement ;
- éviter de rester longtemps immobile ;
- ne pas seulement tourner la semelle à plat ;
- ne pas changer de pièce ou déplacer fortement la semelle dans l’espace ;
- éviter tout contact avec une table ou un support métallique ;
- calibrer dans la chaussure si elle sera normalement utilisée dans cette chaussure ;
- ne plus déplacer les composants électroniques après la calibration.


Pour vérifier visuellement la couverture, les points bruts doivent former une enveloppe ressemblant à un ellipsoïde complet. S’il manque une face ou si les points ressemblent surtout à un cercle, certaines orientations n’ont pas été suffisamment couvertes.