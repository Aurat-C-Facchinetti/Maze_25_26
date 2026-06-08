import sensor, image, time

# ── 1. SETUP HARDWARE ─────────────────────────────────────────────────────────
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # Imposta la risoluzione a QVGA (320x240 pixel)
sensor.set_auto_gain(False) # Disabilita il controllo automatico del guadagno per avere esposizione stabile
sensor.set_auto_whitebal(False) # Disabilita il bilanciamento automatico del bianco per colori costanti
sensor.set_auto_exposure(False, exposure_us=10000) # Disabilita l'esposizione automatica e la fissa a 10.000 microsecondi (10ms)
sensor.skip_frames(time=1000) # Attende 1 secondo scartando i frame iniziali per stabilizzare il sensore

# ── SOGLIE LAB ────────────────────────────────────────────────────────────────
# Ogni tupla definisce un range nello spazio colore LAB: (L_min, L_max, A_min, A_max, B_min, B_max)
# I range sono calibrati manualmente per minimizzare le sovrapposizioni tra colori
SOGLIE_COLORI = [
    (10, 55,  15,  45,  0,   40),   # [0] ROSSO  → punteggio -1
    (50, 80, -20,  15,  35,  80),   # [1] GIALLO → punteggio  0
    (30, 70, -65, -10,  10,  60),   # [2] VERDE  → punteggio +1
    (20, 70, -20,  25, -50, -10),   # [3] BLU    → punteggio +2
    ( 0, 30, -15,  15, -15,  15),   # [4] NERO   → punteggio -2
]

SCORES = [-1, 0, 1, 2, -2] # Punteggi associati a ciascun colore nell'ordine di SOGLIE_COLORI

NOMI = ["R", "G", "V", "B", "N"]   # Rosso, Giallo, Verde, Blu, Nero per il debug grafico

"""
Analizza una ROI (Region Of Interest) dell'immagine e determina
il colore dominante tramite un sistema a 'voto per area'.
Restituisce il punteggio associato al colore vincente e il suo indice,
oppure (0, -1) se nessun colore supera la soglia minima di copertura.
"""
def get_voted_score(img, roi):
    voti = []

    # Per ogni soglia colore, calcola la somma dei pixel dei blob trovati nella ROI
    for soglia in SOGLIE_COLORI:
        area = 0
        # Cerca blob che corrispondono alla soglia corrente nella ROI specificata
        blobs = img.find_blobs([soglia], roi=roi, pixels_threshold=10) # pixels_threshold=10 ignora blob troppo piccoli (rumore)
        if blobs:
            for b in blobs: area += b.pixels() # Somma le aree di tutti i blob trovati per questo colore
        voti.append(area)  # Registra il "voto" (area totale) per questo colore

    massa_totale = sum(voti) # Calcola la massa totale di pixel colorati trovati nella ROI

    if massa_totale == 0: return 0, -1 # Se nessun pixel colorato è stato trovato, la cella è vuota

    # Trova il colore con la maggiore area (il "vincitore" del voto)
    max_voti = max(voti)
    idx = voti.index(max_voti)

    # Applica una soglia di confidenza: il colore vincente deve coprire
    area_cella = roi[2] * roi[3]
    if max_voti > (area_cella * 0.20): # almeno il 20% dell'area totale della ROI per essere considerato valido
        return SCORES[idx], idx  # Restituisce punteggio e indice del colore dominante

    # Se la copertura è insufficiente, considera la cella indeterminata
    return 0, -1

"""
Individua un singolo cerchio bersaglio nell'immagine usando una pipeline
in due fasi: prima blobbing per localizzare l'area di interesse,
poi Hough Circles per rilevare il cerchio preciso.
Restituisce l'oggetto cerchio migliore trovato, oppure None.
"""
def trova_singolo_cerchio(img):
    # Fase 1: trova blob con soglia LAB ampia per individuare qualsiasi oggetto
    # pixels_threshold=600 filtra piccole aree irrilevanti; merge=True unisce blob vicini
    t_blobs = img.find_blobs([(5, 90, -70, 70, -70, 70)], pixels_threshold=600, merge=True)

    if not t_blobs:
        print("nessun blob rilevato")
        return None # Se non viene trovato alcun blob

    b = max(t_blobs, key=lambda x: x.area()) # Seleziona il blob con l'area maggiore come candidato bersaglio principale

    # Fase 2: applica la trasformata di Hough per cerchi nella regione del blob
    # threshold=10000: soglia di accumulo alta per ridurre falsi positivi
    # x_margin=50, y_margin=50: margini alti per fondere cerchi concentrici vicini
    # r_min: raggio minimo pari a un quarto della larghezza del blob
    cs = img.find_circles(roi=b.rect(), threshold=3000, x_margin=50, y_margin=50, r_min=b.w()//4)

    if not cs:
        print("nessun cerchio rilevato con hough")
        return None # Se Hough non trova cerchi nel blob, restituisce None

    # Seleziona il cerchio il cui centro e raggio sono più coerenti con il blob trovato
    bcx, bcy = b.cx(), b.cy()   # Centro del blob di riferimento
    best_c = cs[0]              # Candidato iniziale: primo cerchio della lista
    min_err = 10000             # Errore minimo iniziale arbitrariamente alto

    for c in cs:
        # Calcola un errore combinato: distanza dal centro del blob + differenza di raggio
        err = abs(c.x()-bcx) + abs(c.y()-bcy) + abs(c.r() - (b.w()//2))
        if err < min_err:
            min_err = err
            best_c = c  # Aggiorna il miglior cerchio trovato finora

    return best_c


# ── LOOP PRINCIPALE ───────────────────────────────────────────────────────────
while(True):
    img = sensor.snapshot() # Acquisisce un nuovo frame dalla fotocamera

    c = trova_singolo_cerchio(img) # Tenta di trovare il cerchio bersaglio nel frame corrente

    if c:
        cx, cy, r = c.x(), c.y(), c.r() # Estrae le coordinate del centro e il raggio del cerchio rilevato

        img.draw_circle(cx, cy, r, color=(0,0,0), thickness=2) # Disegna il contorno del cerchio sull'immagine per il debug visivo

        score_tot = 0       # Accumulatore del punteggio totale del bersaglio
        step = r / 5        # Suddivide il raggio in 5 fasce anulari equidistanti

        # Analizza 5 fasce concentriche dal bordo esterno verso il centro
        for i in range(5):
            dist = r - (i * step) - (step/2) # Calcola la distanza dal centro per la fascia corrente (i=0 è la più esterna)

            # Dimensioni della ROI rettangolare per campionare la fascia anulare in cima
            # Il minimo garantisce che la ROI non sia mai troppo piccola
            cw, ch = max(6, int(r*0.3)), max(4, int(r*0.18))

            # Calcola la posizione della ROI: centrata orizzontalmente sul cerchio,
            # posizionata verticalmente alla distanza 'dist' dal centro verso l'alto
            roi_c = (int(cx - cw/2), int(cy - dist - ch/2), cw, ch)

            if roi_c[0] > 0 and roi_c[1] > 0 and (roi_c[0]+cw) < img.width(): # Verifica che la ROI sia completamente all'interno dei bordi dell'immagine
                # Determina il colore e il punteggio della fascia corrente
                val, idx = get_voted_score(img, roi_c)
                score_tot += val  # Accumula il punteggio della fascia

                img.draw_rectangle(roi_c, color=(0,0,0)) # Disegna il rettangolo della ROI per il debug visivo

                if idx != -1: img.draw_string(roi_c[0]+cw+2, roi_c[1], NOMI[idx], color=0) # Se è stato rilevato un colore valido, mostra la sua etichetta accanto alla ROI

        # Mostra il punteggio totale sopra il cerchio in un riquadro bianco
        # Il rettangolo bianco (fill=True) garantisce leggibilità del testo
        img.draw_rectangle(cx-20, cy-r-22, 50, 18, color=255, fill=True)
        img.draw_string(cx-18, cy-r-20, "S:%d" % score_tot, color=0, scale=1.2)
