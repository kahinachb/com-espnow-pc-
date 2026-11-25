import cv2

# 1. Initialisation des deux caméras
cap1 = cv2.VideoCapture('/dev/video4')
cap  = cv2.VideoCapture('/dev/video2')

# Vérification de l'ouverture des flux
if not cap1.isOpened():
    print("Erreur : Impossible d'ouvrir la caméra sur /dev/video2")
if not cap.isOpened():
    print("Erreur : Impossible d'ouvrir la caméra sur /dev/video4")

print("Appuyez sur la touche 'q' pour quitter.")

while True:
    # 2. Lecture image par image (frame)
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap.read()

    # 3. Affichage des frames si la lecture a réussi
    if ret1:
        cv2.imshow('Camera 1 (/dev/video2)', frame1)
    else:
        print("Erreur de lecture sur la caméra 1")
        
    if ret2:
        cv2.imshow('Camera 2 (/dev/video4)', frame2)
    else:
        print("Erreur de lecture sur la caméra 2")

    # 4. Condition de sortie (touche 'q')
    # cv2.waitKey(1) attend 1 milliseconde pour voir si une touche est pressée
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 5. Libération des ressources et fermeture des fenêtres
cap1.release()
cap.release()
cv2.destroyAllWindows()