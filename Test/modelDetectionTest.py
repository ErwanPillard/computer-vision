import cv2
from ultralytics import YOLO
import supervision as sv

# Charger le modèle YOLOv8 pré-entraîné
model = YOLO("Model/best.pt") #pour lancer diectement depuis ici modifier par "../Model/best.pt"

# Définir les annotateurs
box_annotator = sv.BoundingBoxAnnotator()
label_annotator = sv.LabelAnnotator()

# Ouvrir la webcam (index 0 par défaut)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Erreur: Impossible d'ouvrir la webcam.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Erreur: Impossible de lire l'image de la webcam.")
        break

    # Faire une prédiction sur le frame actuel
    results = model(frame, agnostic_nms=True)[0]
    detections = sv.Detections.from_ultralytics(results)

    # Filtrer les détections par la précision
    detections = detections[detections.confidence > 0.85]

    # Générer les étiquettes
    labels = [
        f"{results.names[class_id]}"
        for class_id in detections.class_id
    ]

    # Annoter le frame
    annotated_frame = box_annotator.annotate(
        scene=frame,
        detections=detections,
    )

    annotated_frame = label_annotator.annotate(
        annotated_frame,
        detections=detections,
        labels=labels
    )

    # Afficher le frame annoté
    cv2.imshow("Webcam - Detection", annotated_frame)

    # Quitter la boucle si la touche 'q' est pressée
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer les ressources
cap.release()
cv2.destroyAllWindows()
