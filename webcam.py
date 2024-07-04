import cv2

# Capture vidéo depuis la caméra (0 indique la première caméra disponible)
cap = cv2.VideoCapture(1)

while True:
    # Capture image par image
    ret, frame = cap.read()

    # Affichage du cadre résultant
    cv2.imshow('Camera', frame)

    # Touche 'q' pour quitter la boucle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libération de la capture et fermeture de la fenêtre
cap.release()
cv2.destroyAllWindows()
