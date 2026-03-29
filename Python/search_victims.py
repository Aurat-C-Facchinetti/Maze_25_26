import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO("best.pt")

# cheisti male male a chat, si possono testare o tenere per buoni questi...
COLOR_RANGES = {
    'BLACK': ((0, 0, 0), (180, 255, 50)),
    'RED': [((0, 100, 100), (10, 255, 255)),
            ((170, 100, 100), (180, 255, 255))], 
    'YELLOW': ((20, 100, 100), (30, 255, 255)),
    'GREEN': ((40, 40, 40), (80, 255, 255)),
    'BLUE': ((100, 100, 100), (130, 255, 255))
}

def detect_ring_colors(parImg, parCenter, parRadius, parNumRings=5):
    x, y = parCenter
    hsv = cv2.cvtColor(parImg, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]

    ringColors = []

    for i in range(parNumRings):
        # calcola raggio interno ed esterno dell'anello corrente
        innerRadius = int(parRadius * i / parNumRings) # es: 100 * 1 / 5 = 20 
        outerRadius = int(parRadius * (i+1) / parNumRings) # es: 100 * (1 + 1) / 5 = 100 * 2 / 5 = 40

        # cerchiamo solo i pixel tra inner_r e outer_r
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(mask, (x, y), outerRadius, 255, -1)  # cerchio esterno bianco
        cv2.circle(mask, (x, y), innerRadius, 0, -1)    # rimuove cerchio interno

        max = 0
        dominantColor = 'UNKNOWN'

        for color, ranges in COLOR_RANGES.items():
            if color == 'RED':
                # rosso richiede due range per HSV wrap-around, almeno chat dice cosi' come correzione al mio codice che non andava
                mask1 = cv2.inRange(hsv, ranges[0][0], ranges[0][1])
                mask2 = cv2.inRange(hsv, ranges[1][0], ranges[1][1])
                colorMask = cv2.bitwise_or(mask1, mask2)
            else:
                colorMask = cv2.inRange(hsv, ranges[0], ranges[1])

            # applica maschera e conta pixel del colore
            masked = cv2.bitwise_and(colorMask, mask)
            count = cv2.countNonZero(masked)

            if count > max:
                max = count
                dominantColor = color

        ringColors.append(dominantColor)

    return ringColors

def search_cognitive_target(par_image_path):
    img = cv2.imread(par_image_path)
    if img is None:
        print(f"Errore: impossibile caricare l'immagine {par_image_path}")
        return

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 2)

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=1000,
        param1=50,
        param2=30,
        minRadius=650,
        maxRadius=0
    )

    if circles is None:
        print("Nessun cerchio rilevato")
        return None
    else:
        circles = np.uint16(np.around(circles))
        print(f"Rilevati {len(circles[0])} cerchi")

        for (x, y, r) in circles[0]:
            cv2.circle(img, (x, y), r, (0, 255, 0), 2)
            cv2.circle(img, (x, y), 2, (0, 0, 255), -1)
            print(f"Centro=({x},{y})  Raggio={r}")

            ring_colors = detect_ring_colors(img, (x, y), r, parNumRings=5)
            for idx, color in enumerate(ring_colors):
                print(f"  Anello {idx+1}: {color}")
            total = sum_circles(ring_colors)
            if total == 0:
                return "U"
            elif total == 1:
                return "S"
            elif total == 2:
                return "H"
            else:
                return "FAKE"
        
def sum_circles(par_ring_colors):
    total = 0
    for idx, color in enumerate(par_ring_colors):
        if color == "BLACK":
            total -= 2
        elif color == "RED":
            total -= 1
        elif color == "GREEN":
            total += 1
        elif color == "BLUE":
            total += 2
    return total
        

def search_letter(par_image_path):
    result = model(par_image_path)
    best_result = result[0]
    class_id = int(best_result.probs.top1)
    confidence = float(best_result.probs.top1conf)
    class_name = model.names[class_id]

    print("=== RISULTATO INFERENCE YOLO ===")
    print(f"Classe predetta : {class_name}")
    print(f"Confidenza      : {confidence:.3f}")

    if confidence >= 0.66:
        if class_name == "omega":
            return "U"
        elif class_name == "psi":
            return "S"
        elif class_name == "phi":
            return "H"
        else:
            return "FAKE"
    else:
        return "FAKE"

    

def search(par_image_path):
    letter = search_letter(par_image_path)
    if letter != "FAKE":
        return letter
    else:
        cognitive_target = search_cognitive_target(par_image_path)
        return cognitive_target