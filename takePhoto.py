import cv2
import os
import time


# Vérifie si le dossier "photos" existe, sinon le crée
if not os.path.exists('draft/photos'):
    os.makedirs('draft/photos')

# Ouvre la webcam
cap = cv2.VideoCapture(0)

while True:
    # Lit un frame de la webcam
    ret, frame = cap.read()

    # Vérifie si le frame a été correctement lu
    if not ret:
        print("Impossible de lire le flux vidéo")
        break

    # Génère un nom de fichier unique pour chaque photo
    filename = 'photos/photo_{}.jpg'.format(int(time.time()))

    # Enregistre la photo dans le dossier "photos"
    cv2.imwrite(filename, frame)

    # Affiche la photo pendant 1 seconde
    cv2.imshow('frame', frame)

    # Attend 1 seconde avant de prendre la prochaine photo
    time.sleep(1)

    # Vérifie si l'utilisateur a appuyé sur la touche 'q' pour quitter
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libère la webcam
cap.release()

# Ferme toutes les fenêtres ouvertes
cv2.destroyAllWindows()


