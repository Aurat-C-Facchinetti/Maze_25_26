import cv2
import numpy as np

def analyze_grid(cropped):
    h, w = cropped.shape
    cell_h, cell_w = h // 5, w // 5
    binary_string = ""

    total_pixels = cell_h * cell_w
    soglia_nera = total_pixels * 0.05

    for i in range(5):
        for j in range(5):
            cell = cropped[i*cell_h:(i+1)*cell_h, j*cell_w:(j+1)*cell_w]
            neri_veri = cv2.countNonZero(cell)
            binary_string += '1' if neri_veri >= soglia_nera else '0'

    return binary_string

def riconosci_lettera_centrale(binario):
    
    c1 = binario[0]   # SX 1ª riga
    c2 = binario[2]   # centrale 1ª riga
    c3 = binario[4]   # DX 1ª riga
    c4 = binario[10]  # SX 3ª riga
    c5 = binario[12]  # centrale 3ª riga
    c6 = binario[14]  # DX 3ª riga
    c7 = binario[20]  # SX 5ª riga
    c8 = binario[22]  # centrale 5ª riga
    c9 = binario[24]  # DX 5ª riga    
    
    # Caso lettera H
    if c1 == '1' and c2 == '0' and c3 == '1' and c4 == '1' and c5 == '1' and c6 == '1' and c7 == '1' and c8 == '0' and c9 == '1':
        return 'H'
    elif c1 == '0' and c2 == '1' and c3 == '0' and c4 == '0' and c5 == '1' and c6 == '0' and c7 == '0' and c8 == '1' and c9 == '0' and binario[3] == '1' and binario[21] == '1': #CASO SPECIALE 
        return 'S'
    elif c1 == '1' and c2 == '0' and c3 == '1' and c4 == '1' and c5 == '0' and c6 == '1' and c7 == '0' and c8 == '1' and c9 == '0': #CASO U 
        return 'U'
    
'''
    if c1 == '0' and c2 == '0':
        return 'H'
    elif c1 == '0' and c2 == '1':
        return 'U'
    elif c1 == '1' and c2 == '1':
        return 'S'
    else:
        return '?'
'''

# Webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Premi 's' per analizzare - 'q' per uscire", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_roi = gray[50:430, :]  # solo tra y=50 e y=430
        _, thresh = cv2.threshold(gray_roi, 50, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        area_id = 0

        for cnt in contours:
            if cv2.contourArea(cnt) < 900:
                continue

            rot_rect = cv2.minAreaRect(cnt)
            original_angle = rot_rect[2]

            # ✅ Correzione super intuitiva e funzionante
            if original_angle > 45:
                corrected_angle = -(90 - original_angle)
            else:
                corrected_angle = original_angle

            print(f"📐 Angolo originale: {original_angle:.2f}°, corretto: {corrected_angle:.2f}°")

            # Rotazione
            center = (int(rot_rect[0][0]), int(rot_rect[0][1]))
            M = cv2.getRotationMatrix2D(center, corrected_angle, 1.0)
            rotated = cv2.warpAffine(thresh, M, (thresh.shape[1], thresh.shape[0]))

            # Ritaglio
            x, y, w, h = cv2.boundingRect(cnt)
            cropped = rotated[y:y+h, x:x+w]

            binary = analyze_grid(cropped)
            print(f"📦 Area {area_id} → Binario: {binary}")

            lettera = riconosci_lettera_centrale(binary)
            print(f"🔤 Lettera riconosciuta: {lettera}")

            # Griglia
            h_c, w_c = cropped.shape
            grid_display = cv2.cvtColor(cropped, cv2.COLOR_GRAY2BGR)
            for i in range(1, 5):
                cv2.line(grid_display, (0, i*h_c//5), (w_c, i*h_c//5), (255, 0, 0), 1)
                cv2.line(grid_display, (i*w_c//5, 0), (i*w_c//5, h_c), (255, 0, 0), 1)

            cv2.imshow(f"Area {area_id} - {lettera}", grid_display)
            area_id += 1

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
