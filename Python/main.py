from settings import *
from imps import *
import sys

# Stati celle in "visited"
# 0 = white (not yet visited)
# 1 = giallo (visited)
# 2 = nero (NOT VISITABLE)
# 3 = blu (B)
# 4 = verde (G)
# 5 = grigio (M)

def main():
    # ========== INIZIALIZZAZIONE ==========
    world = World()
    world.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # COM* on Windows, /dev/ttyUSB* on Raspberry
    print("::::::::: Connessione seriale stabilita ::::::::::")
    print(world.ser)
    
    # Attendi che Arduino sia pronto
    world.check_specified_command("START")
    print("::::::::: Arduino pronto ::::::::::")
    start_time = time.time()
    explore_seconds = 8 * 60 * 0.66 # prendo il 66% dei secondi di gara prima di tornare a casa autonomamente

    # Setup finestra OpenCV
    cv2.namedWindow('Campo', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Campo', VIEW_W, VIEW_H)
    
    # ========== SCANSIONE INIZIALE AMBIENTE ==========
    print("\n" + "="*60)
    print("::::::::: SCANSIONE AMBIENTE INIZIALE ::::::::::")
    print("="*60)
    world.get_walls("m001,", 0)  # fronte
    world.get_walls("m002,", 1)  # destra
    world.get_walls("m003,", 2)  # retro
    world.get_walls("m004,", 3)  # sinistra
    print(f"::::::::: Muri iniziali rilevati (bitmask): {int(world.walls[world.y, world.x])} ::::::::::")
    print("="*60 + "\n")
    # ========== FINE SCANSIONE INIZIALE ==========

    def handle_key(k):
        """
        Gestisce input tastiera.
        Ritorna:
            - 'quit': termina programma
            - 'repath': interrompe autopilot e forza ricalcolo
            - None: continua normalmente
        """
        if k == ord('q') or k == 27:  # Q o ESC
            return 'quit'
        
        # --- CONTROLLO MANUALE (interrompe autopilot) ---
        elif k in (KEY_W, KEY_WU):
            world.forward()
            return 'repath'
        elif k in (KEY_A, KEY_AU):
            world.rotate_left()
            return 'repath'
        elif k in (KEY_D, KEY_DU):
            world.rotate_right()
            return 'repath'
        elif k in (KEY_S, KEY_SU):
            world.flip_direction()
            return 'repath'
        
        # --- MODIFICA MAPPA (NON interrompe autopilot) ---
        elif k == ord('1'):
            world.get_walls("m001,", 0)
            return None
        elif k == ord('2'):
            world.get_walls("m002,", 1)
            return None
        elif k == ord('3'):
            world.get_walls("m003,", 2)
            return None
        elif k == ord('4'):
            world.get_walls("m004,", 3)
            return None
        elif k == ord('n') or k == ord('N'):
            # Blocca casella davanti
            dx, dy = world.heading_to_delta()
            nx, ny = world.x + dx, world.y + dy
            nx, ny = world.ensure_inside(nx, ny)
            for dir_idx in range(4):
                world.set_wall_absolute(nx, ny, dir_idx, 1)
            world.visited[ny, nx] = 2
            print("::::::::: Casella davanti bloccata manualmente ::::::::::")
            return None
        elif k == ord('b') or k == ord('B'):
            # Colora casella corrente di blu
            world.visited[world.y, world.x] = 3
            print("::::::::: Casella corrente colorata BLU ::::::::::")
            return None
        elif k == ord('g') or k == ord('G'):
            # Controlla inclinazione
            world.check_inclination()
            return None
        elif k == ord('m') or k == ord('M'):
            # Colora casella corrente di grigio (checkpoint)
            world.visited[world.y, world.x] = 5
            print("::::::::: Casella corrente colorata GRIGIO (checkpoint) ::::::::::")
            return None
        
        # --- CAMBIO OBIETTIVO (interrompe autopilot) ---
        elif k == ord('t') or k == ord('T'):
            world.target = world.bottom_right_cell()
            world.mode = 'to_target'
            print(f"::::::::: NUOVO OBIETTIVO: Angolo basso-destra {world.target} ::::::::::")
            return 'repath'
        elif k == ord('e') or k == ord('E'):
            world.target = world.home
            world.mode = 'to_home'
            print(f"::::::::: NUOVO OBIETTIVO: Ritorno a casa {world.home} ::::::::::")
            return 'repath'
        
        return None

    def on_step():
        """
        Pausa intelligente durante autopilot.
        Permette input utente durante il movimento automatico.
        Ritorna True se l'autopilot deve fermarsi.
        """
        end_t = time.time() + (STEP_DELAY_MS / 1000.0)
        need_stop = False
        
        while time.time() < end_t and not need_stop:
            # Aggiorna visualizzazione
            frame = world.render()
            cv2.imshow('Campo', frame)
            
            # Controlla input (non bloccante)
            k = cv2.waitKey(INPUT_POLL_MS) & 0xFF
            if k == 255:  # Nessun input
                continue
            
            # Gestisci input
            res = handle_key(k)
            if res in ('quit', 'repath'):
                need_stop = True
        
        return need_stop

    # ========== MAIN LOOP ==========
    print("\n" + "="*60)
    print("::::::::: MAIN LOOP AVVIATO ::::::::::")
    print("="*60)
    print("COMANDI:")
    print("  T = vai verso angolo basso-destra")
    print("  E = ritorna a casa")
    print("  W/A/D/S = controllo manuale")
    print("  1/2/3/4 = scansiona muro specifico")
    print("  N = blocca casella davanti")
    print("  B = colora casella blu")
    print("  G = controlla inclinazione")
    print("  M = segna checkpoint")
    print("  Q/ESC = esci")
    print("="*60 + "\n")
    
    while True:
        # ========== RENDERING ==========
        frame = world.render()
        cv2.imshow('Campo', frame)
        
        # ========== INPUT MANAGEMENT ==========
        # Se in autopilot: non bloccare (permetti movimento continuo)
        # Se in idle: blocca fino a input
        if world.mode in ('to_target', 'to_home'):
            key = cv2.waitKey(INPUT_POLL_MS) & 0xFF  # Non bloccante
        else:
            key = cv2.waitKey(0) & 0xFF  # Bloccante in idle
        
        # ========== GESTIONE COMANDI ==========
        if key != 255:  # Input ricevuto
            res = handle_key(key)
            if res == 'quit':
                print("\n::::::::: USCITA RICHIESTA ::::::::::")
                break
            if res == 'repath':
                # Comando manuale ricevuto, continua loop per ricalcolare
                continue
        
        # ========== AUTOPILOT ==========
        if world.mode in ('to_target', 'to_home'):
            goal = world.target if world.mode == 'to_target' else world.home
            
            print("\n" + "-"*60)
            print(f"::::::::: AUTOPILOT ATTIVO: {world.mode} ::::::::::")
            print(f"::::::::: Posizione attuale: ({world.x},{world.y}) ::::::::::")
            print(f"::::::::: Obiettivo: {goal} ::::::::::")
            print("-"*60)
            
            # ========== LOOP RETRY ==========
            while time.time() - start_time < explore_seconds or world.mode == 'to_home':
                # ========== CALCOLO PERCORSO ==========
                start = (world.x, world.y)
                print(f"::::::::: Calcolo percorso da {start} a {goal} ::::::::::")
                
                path = world.dijkstra(start, goal)
                
                # ========== TERMINAZIONE SU PERCORSO IMPOSSIBILE ==========
                if path is None or len(path) == 0:
                    print("\n" + "X"*60)
                    print("ERRORE FATALE: NESSUN PERCORSO TROVATO!")
                    print(f"Posizione attuale: ({world.x}, {world.y})".ljust(58))
                    print(f"Obiettivo: {goal}".ljust(58))
                    print("Il robot è completamente bloccato.")
                    print("Impossibile proseguire.")
                    print("TERMINAZIONE PROGRAMMA.")
                    print("X"*60 + "\n")
                    
                    # Shutdown e terminazione
                    print("::::::::: SHUTDOWN FORZATO ::::::::::")
                    world.shutdown()
                    cv2.destroyAllWindows()
                    sys.exit(1)
                # ========== FINE TERMINAZIONE ==========
                
                print(f"::::::::: Percorso trovato! Lunghezza: {len(path)} passi ::::::::::")
                print(f"::::::::: Percorso: {path[:5]}{'...' if len(path) > 5 else ''} ::::::::::")
                
                # ========== ESECUZIONE PERCORSO ==========
                completed = world.follow_path(
                    path, 
                    on_step=on_step, 
                    stop_when_home=(world.mode == 'to_home')
                )
                
                if completed:
                    print("\n" + "="*60)
                    print("::::::::: PERCORSO COMPLETATO CON SUCCESSO! ::::::::::")
                    print("="*60 + "\n")
                    
                    # Se siamo tornati a casa in modalità to_home, esci dal loop
                    if world.mode == 'to_home' and (world.x, world.y) == world.home:
                        print("::::::::: MISSIONE COMPLETATA: RITORNO A CASA ::::::::::")
                        #world.ser.write() TODO comunicare ad arduino di lampeggiare per bonus
                        world.mode = 'idle'
                    
                    break  # Esci dal retry loop
                else:
                    # ========= GESTIONE LOP =========
                    if world.lop_event.is_set():
                        world.lop_event.clear()
                        print("::::::::: LoP rilevato, aspetto 15 secondi per fare /tp... :::::::::")
                        time.sleep(15)
                        world.x, world.y = world.last_checkpoint
                        world.shared_x.value = world.x
                        world.shared_y.value = world.y
                        world.deg = 0 # direzione di default, si puo' decidere
                        world.shared_deg.value = world.deg
                        print(f"::::::::: LoP, /tp a checkpoint: {world.last_checkpoint} :::::::::")
                        continue
                    # ========= FINE GESTIONE LOP =========

                    if time.time() - start_time >= explore_seconds:
                        print("\n" + "!"*60)
                        print("::::::::: TEMPO SCADUTO, TORNO A CASA ::::::::::")
                        print("!"*60 + "\n")
                    
            # ========== FINE RETRY LOOP ==========
            if world.mode != 'idle':
                    world.mode = 'idle'
                    print("::::::::: Autopilot completato, torno in IDLE ::::::::::")
    
    # ========== SHUTDOWN NORMALE ==========
    print("\n" + "="*60)
    print("::::::::: SHUTDOWN IN CORSO ::::::::::")
    print("="*60)
    world.shutdown()
    cv2.destroyAllWindows()
    print("::::::::: PROGRAMMA TERMINATO ::::::::::")


if __name__ == '__main__':
    main()
