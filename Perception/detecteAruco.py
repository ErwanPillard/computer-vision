import cv2
import numpy as np

# Charger l'image
image = cv2.imread('../data/map/IMG_0701.jpg')

# Redimensionner l'image pour une résolution plus gérable (par exemple, 1024x768)
scale_percent = 90  # pourcentage de réduction
width = int(image.shape[1] * scale_percent / 100)
height = int(image.shape[0] * scale_percent / 100)
dim = (width, height)

# Redimensionner l'image
resized_image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

# Convertir l'image en niveaux de gris
gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

# Charger le dictionnaire ArUco
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# Initialiser le détecteur de paramètres
parameters = cv2.aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 1  # Exemple d'ajustement de paramètre


# Détecter les marqueurs dans l'image
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

# Vérifier si des marqueurs ont été détectés
if np.all(ids is not None):
    # Dessiner les marqueurs détectés
    cv2.aruco.drawDetectedMarkers(resized_image, corners, ids)
    # Afficher les identifiants des marqueurs détectés
    for i in range(len(ids)):
        print(f'Marqueur ID: {ids[i][0]}')
else:
    print('Aucun marqueur détecté')

# Afficher l'image avec les marqueurs détectés
cv2.imshow('Image avec marqueurs ArUco', resized_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
