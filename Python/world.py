from imps import *
from settings import *
from color_sensor import *
from side_camera import SideCamera
import threading
import time
import cv2
from multiprocessing import Process, Queue, Manager
import search_victims
import reset_button
from gpiozero import DigitalOutputDevice
lop_interrupt = DigitalOutputDevice(17)

class World:
    def __init__(self, rows=INIT_ROWS, cols=INIT_COLS):
        # matrice logica
        self.visited = np.zeros((rows, cols), dtype=np.uint8)
        # walls bitmask per N,E,S,W = 1,2,4,8
        self.walls = np.zeros((rows, cols), dtype=np.uint8)
        self.h, self.w = self.visited.shape

        # player
        self.x = self.w // 2
        self.y = self.h // 2
        self.home = (self.x, self.y)  # casella di partenza
        self.deg = 0  # 0=N, 90=E, 180=S, 270=W

        # navigazione
        self.target = None
        self.mode = "idle"  # idle | to_target | to_home

        self.visit_current()
        # raddoppia subito la matrice per più spazio iniziale
        self.expand_double()

        self.ser = None
        self.isInvertito = False

        self.run_event = threading.Event()
        self.stop_event = threading.Event()
        self.thread_check_black = threading.Thread(target=check_black, args=(self.run_event,self.stop_event), daemon=False)
        self.thread_check_black.start()
        print("|||||||| thread nero iniziato ||||||||||")

        self.last_checkpoint = self.home
        self.lop_event = threading.Event()
        reset_button.setup_button()
        self.thread_lop_handler = threading.Thread(target=self.handle_lop, daemon=True)
        self.thread_lop_handler.start()
        print("|||||||| thread pulsante iniziato ||||||||")

        self.manager = Manager()
        self.shared_x = self.manager.Value('i', self.w // 2)  # int condiviso
        self.shared_y = self.manager.Value('i', self.h // 2)  # int condiviso
        self.shared_deg = self.manager.Value('i', 0)  # int condiviso
        self.shared_walls = self.manager.dict()  # dict condiviso per walls
        self.shared_search_victim = self.manager.Value('b', True)

        self.victim_queue = Queue()

        self.shared_running = self.manager.Value('b', True) # flag per terminazione processo
        self.recogn_proc = Process(
            target=World.recognition_process,
            args=(self.shared_x, self.shared_y, self.shared_deg, self.shared_walls, 
                  self.shared_search_victim, self.victim_queue, self.shared_running)
        )
        self.recogn_proc.start()
        print("||||||| Processo riconoscimento vittime avviato ||||||||")

    @staticmethod
    def recognition_process(shared_x, shared_y, shared_deg, shared_walls, shared_search_victim, shared_victim_queue, shared_running):
        right_camera = SideCamera(1)
        left_camera = SideCamera(0)
        
        while shared_running.value:
            if not shared_search_victim.value:
                time.sleep(0.1)
            else:
                try:
                    x = shared_x.value
                    y = shared_y.value
                    deg = shared_deg.value
                    
                    current_dir = ((deg // 90) % 4)
                    right_dir = (current_dir + 1) % 4
                    right_wall_bit = [1, 2, 4, 8][right_dir]  # bit_for_dir inline
                    
                    key = f"{x},{y}"
                    walls_value = shared_walls.get(key, 0)
                    
                    if (walls_value & right_wall_bit) != 0:
                        right_camera.capture("right.jpg")
                        victim = search_victims.search("right.jpg")
                        if victim:
                            shared_victim_queue.put({'side': 'right', 'type': victim})
                            print(f"##### VITTIMA {victim} rilevata a DESTRA #####")
                            shared_search_victim.value = False
                    
                    left_dir = (current_dir + 3) % 4
                    left_wall_bit = [1, 2, 4, 8][left_dir]
                    
                    if (walls_value & left_wall_bit) != 0:
                        left_camera.capture("left.jpg")
                        victim = search_victims.search("left.jpg")
                        if victim:
                            shared_victim_queue.put({'side': 'left', 'type': victim})
                            print(f"##### VITTIMA {victim} rilevata a SINISTRA #####")
                            shared_search_victim.value = False
                    
                    time.sleep(0.1)
                except Exception as e:
                    print(f"[ERRORE] Recognition: {e}")
                    time.sleep(0.5)

            
    def check_victim_queue(self):
        try:
            while not self.victim_queue.empty():
                victim_data = self.victim_queue.get_nowait()
                print(f"vittima nella queue: {victim_data}")
                current_victim_type = victim_data['type']
                current_victim_side = victim_data['side']

                command = current_victim_type
                if current_victim_side == 'right':
                    command = command.upper()
                else:
                    command = command.lower()
                
                self.ser.write(command.encode())
            print("queue vuota")
        except Exception as e:
            print(f"[ERRORE] check_victim_queue: {e}")
    
    def handle_lop(self):
        while True:
            reset_button.wait_for_press()
            print("RESET " * 10)

            lop_interrupt.on()
            time.sleep(0.05)
            lop_interrupt.off()

            self.lop_event.set()

    # ---------- util direzione ----------
    """
        Translates degrees into direction 
    """
    @staticmethod
    def dir_index_from_deg(deg):
        # 0=N,1=E,2=S,3=W
        return ((deg // 90) % 4)

    """
        Translates the direction into bits
    """
    @staticmethod
    def bit_for_dir(idx):
        return [1, 2, 4, 8][idx]

    """
        Revers the direction
    """
    @staticmethod
    def opposite_dir(idx):
        return (idx + 2) % 4
    
    def shutdown(self):
        print("SHUTDOWN WORLD in corso")
        self.stop_event.set()
        self.run_event.set()
        if self.thread_check_black.is_alive():
            self.thread_check_black.join()
    
        if hasattr(self, 'shared_running'):
            print("Fermando processo riconoscimento...")
            self.shared_running.value = False  # Segnala al processo di terminare
            
        if hasattr(self, 'recognition_proc') and self.recognition_proc.is_alive():
            print("Aspettando terminazione processo riconoscimento...")
            self.recognition_proc.join(timeout=3)  # Aspetta max 3 secondi
            
            if self.recognition_proc.is_alive():
                print("Processo non terminato, forzo la chiusura...")
                self.recognition_proc.terminate()  # Forza la terminazione
                self.recognition_proc.join(timeout=1)  # Aspetta conferma
                print("Processo riconoscimento terminato forzatamente")
            else:
                print("Processo riconoscimento chiuso correttamente")
        print("thread check black chiuso, SHUTDOWN WORLD completo")

    def check_command(self):
        recived_command = "False"
        while recived_command != "True":
            recived_command = self.ser.readline().decode('utf-8').rstrip()
            print("ricevuto da seriale per check 1:", recived_command)
            time.sleep(0.01)
        return True

    def check_specified_command(self, par_command):
        recived_command = ""
        while recived_command != par_command:
            recived_command = self.ser.readline().decode('utf-8').rstrip()
            print("ricevuto da seriale:", recived_command)
            time.sleep(0.01)

    def check_movement_confirmation(self):
        recived_command = ""
        while recived_command != "1" and recived_command != "-1":
            self.check_victim_queue() #durante un movimento sono dentro questo loop, in quanto aspetto conferma da arduino
            if self.lop_event.is_set():
                return False
            recived_command = self.ser.readline().decode('utf-8').rstrip()
            print("ricevuto da seriale:", recived_command)
            time.sleep(0.01)
        if recived_command == "1":
            return True
        else:
            return False

    """
        Heads towards the delta (how much we have to move)
    """
    def heading_to_delta(self):
        if self.deg == 0:
            return (0, -1)
        elif self.deg == 90:
            return (1, 0)
        elif self.deg == 180:
            return (0, 1)
        elif self.deg == 270:
            return (-1, 0)
        else:
            # arrotonda a multipli di 90
            r = int(round(self.deg / 90.0)) * 90
            self.deg = r % 360
            return self.heading_to_delta()

    """
        Normalizes degrees (interval: 0-360)
    """
    def normalize_deg(self):
        self.deg = ((self.deg % 360) + 360) % 360

    # ---------- azioni player ----------
    "Function to rotate left"
    def rotate_left(self):
        self.deg -= 90
        self.shared_deg.value = self.deg
        self.normalize_deg()
        self.ser.write(b'a090,')
        print("------ inviato a090, ------")
        self.check_command()
        self.check_movement_confirmation()

    "Function to rotate right"
    def rotate_right(self):
        self.deg += 90
        self.shared_deg.value = self.deg
        self.normalize_deg()
        self.ser.write(b'd090,')
        print("------ inviato d090, ------")
        self.check_command()
        self.check_movement_confirmation()


    "Function to flip direction: used when we will be stuck"
    def flip_direction(self):
        self.deg += 180
        self.normalize_deg()
        self.ser.write(b's,')
        self.check_command()
        self.isInvertito = not self.isInvertito
        print("-----    inversione    -----")

        if (not self.isInvertito):
            self.run_event.set()
        else:
            self.run_event.clear()


    "Function to visit the current cell flagging the cell as visited"
    def visit_current(self):
        # non sovrascrivere stati speciali (2..5)
        if self.visited[self.y, self.x] == 0:
            self.visited[self.y, self.x] = 1
        if self.visited[self.y, self.x] == 5:
            self.last_checkpoint = (self.x, self.y)
            print(f"::::::::: Checkpoint salvato: ({self.x},{self.y}) ::::::::::")

    """ Doubles the matrix keeping the data centered (visited + walls)
        Returns the applied shift (off_x, off_y) to re-align the target coordinates
    """
    def expand_double(self):
        
        oh, ow = self.visited.shape
        nh, nw = oh * 2, ow * 2
        # guardia anti OOM
        MAX_SIDE = 32768
        if nh > MAX_SIDE or nw > MAX_SIDE:
            return 0, 0  # non espandere

        new_v = np.zeros((nh, nw), dtype=np.uint8)
        new_w = np.zeros((nh, nw), dtype=np.uint8)
        off_y = (nh - oh) // 2
        off_x = (nw - ow) // 2
        new_v[off_y:off_y+oh, off_x:off_x+ow] = self.visited
        new_w[off_y:off_y+oh, off_x:off_x+ow] = self.walls
        self.visited, self.walls = new_v, new_w
        self.h, self.w = nh, nw
        self.x += off_x
        self.y += off_y
        # Mantieni allineati anche home e target quando la matrice si espande
        if hasattr(self, 'home') and self.home is not None:
            self.home = (self.home[0] + off_x, self.home[1] + off_y)
        if getattr(self, 'target', None) is not None:
            tx, ty = self.target
            self.target = (tx + off_x, ty + off_y)
        return off_x, off_y
        
    """ Expand until (gx,gy) fits within the boundaries. Returns the realigned coordinates (gx,gy).
        If the maximum limit is exceeded, does not expand and returns the clamped coordinates.
    """
    def ensure_inside(self, gx, gy):
        
        safety = 0
        while gx < 0 or gx >= self.w or gy < 0 or gy >= self.h:
            off_x, off_y = self.expand_double()
            safety += 1
            if off_x == 0 and off_y == 0:
                gx = max(0, min(self.w - 1, gx))
                gy = max(0, min(self.h - 1, gy))
                break
            gx += off_x
            gy += off_y
            if safety > 8:
                gx = max(0, min(self.w - 1, gx))
                gy = max(0, min(self.h - 1, gy))
                break
        return gx, gy

    """
        Sets or remove a wall in an absolute position of the matrix
    """
    def set_wall_absolute(self, gx, gy, dir_idx, value=1):
        bit = self.bit_for_dir(dir_idx)
        if value:
            self.walls[gy, gx] |= bit
        else:
            self.walls[gy, gx] &= (~bit) & 0xF
        
        key = f"{gx},{gy}"
        self.shared_walls[key] = int(self.walls[gy, gx])

    """
        Fa la stessa cosa di quello sopra (TODO: Capire perchè è così)
    """
    def toggle_wall_absolute(self, gx, gy, dir_idx):
        bit = self.bit_for_dir(dir_idx)
        self.walls[gy, gx] ^= bit

    """
        Sets a wall realtive to player's position in both cells
        (the player's one and also sets the opposite wall)
    """
    def set_wall_relative(self, rel_idx, toggle=True):
        """rel_idx: 0=fronte,1=destra,2=retro,3=sinistra rispetto all'heading."""
        cur_abs = self.dir_index_from_deg(self.deg)
        abs_idx = (cur_abs + rel_idx) % 4
        dxdy = [(0,-1),(1,0),(0,1),(-1,0)][abs_idx]
        nx, ny = self.x + dxdy[0], self.y + dxdy[1]
        nx, ny = self.ensure_inside(nx, ny)
        if toggle:
            self.toggle_wall_absolute(self.x, self.y, abs_idx)
            self.toggle_wall_absolute(nx, ny, self.opposite_dir(abs_idx))
        else:
            self.set_wall_absolute(self.x, self.y, abs_idx, 1)
            self.set_wall_absolute(nx, ny, self.opposite_dir(abs_idx), 1)
        self.visit_current()

    #par_tof is the tof we want to read
    def get_walls(self, par_tof, par_dir):
        self.ser.write(par_tof.encode())
        print(f"------ inviato {par_tof} ------")
        if self.check_command():
            is_wall = -1
            while is_wall == -1:
                is_wall = int(self.ser.readline().decode('utf-8').rstrip())
                print("is_wall:", is_wall)
                if is_wall == 1:
                    self.set_wall_relative(par_dir, toggle=False)

    """
        1. Checks if there is a wall in the direction where the player is pointing (forward).
        2. Calculate the new position exploiting heading_to_delta and ensure_inside (inside the matrix)
        3. Checks if there is another wall 
        4. Checks if the next cell is black or visited.
        5. If it can move, it does (coloring the cell in yellow).
    """
    def forward(self):
        dir_idx = self.dir_index_from_deg(self.deg)
        bit = self.bit_for_dir(dir_idx)
        if (self.walls[self.y, self.x] & bit) != 0:
            return False
        dx, dy = self.heading_to_delta()
        nx, ny = self.x + dx, self.y + dy
        nx, ny = self.ensure_inside(nx, ny)
        opp_bit = self.bit_for_dir(self.opposite_dir(dir_idx))
        if (self.walls[ny, nx] & opp_bit) != 0:
            return False
        if self.visited[ny, nx] == 2:
            return False
        moved = (nx != self.x) or (ny != self.y)
        if moved:
            self.ser.write(b'w032,')
            print("------ inviato w032, ------")
            self.check_command()
            if self.check_movement_confirmation():
                self.x, self.y = nx, ny
                self.visit_current()
                self.shared_x.value = self.x
                self.shared_y.value = self.y
            else:
                if not self.lop_event.is_set():
                    dx, dy = self.heading_to_delta()
                    nx, ny = self.x + dx, self.y + dy
                    nx, ny = self.ensure_inside(nx, ny)
                    for dir_idx in range(4):
                        self.set_wall_absolute(nx, ny, dir_idx, 1)
                        self.visited[ny, nx] = 2
                moved = False

        return moved


    """
        Costo di ingresso nella cella in base allo stato.
        0 (white) -> 1 unexplored
        1 (yellow) -> 2 Explore but empty
        2 (black)   -> (np.inf) You shall not pass (Not visitable)
        3 (blue)    -> 3  Wait 5 seconds (TODO: Check rule)
        4 (green)  -> 3 Uphill / Downhill
        5 (grey) -> 2 Checkpoint
    """
    # ---------- info celle ----------
    def cell_cost(self, val):
        
        variabile = 2
        
        if val == 0:
           variabile = 1
        elif val == 1:
            variabile = 2
        elif val == 2:
            variabile = np.inf
        elif val == 3:
            variabile = 3
        elif val == 4:
            variabile = 3
        """elif val == 5: Rimosso per ridondanza
            variabile = 2"""
        
        return variabile
    
    def get_color(self):
        if self.visited[self.y, self.x] not in (2, 3, 4, 5):
            self.visited[self.y, self.x] = check_color()
        if self.visited[self.y, self.x] == 5:
            self.last_checkpoint = (self.x, self.y)
            print(f"::::::::: Checkpoint salvato: ({self.x},{self.y}) ::::::::::")
    
    def check_inclination(self):
        self.ser.write(b'i,')
        print("------ inviato i, ------")
        if self.check_command():
            inclination = 999
            while inclination == 999:
                inclination = float(self.ser.readline().decode('utf-8').rstrip())
            print("inclination da seriale:", inclination)
            if abs(inclination) >= 20 and abs(inclination) <= 25:
                self.visited[self.y, self.x] = 4

    """
        Returns a dictionary with useful info about the cell
    """
    def get_cell_info(self, gx, gy):
        
        val = int(self.visited[gy, gx])
        return {
            "x": gx,
            "y": gy,
            "state": val,          # 0..5
            "walls": int(self.walls[gy, gx]), # bitmask NESW
            "cost": float(self.cell_cost(val)),
            "blocked": val == 2,
        }

    # ---------- pathfinding ----------
    """
        Return the list of reachable adjacent cells
    """
    def neighbors(self, gx, gy):
        res = []
        for idx, (dx, dy) in enumerate([(0,-1),(1,0),(0,1),(-1,0)]):
            nx, ny = gx + dx, gy + dy
            if not (0 <= nx < self.w and 0 <= ny < self.h):
                continue
            # muri tra celle
            if (self.walls[gy, gx] & self.bit_for_dir(idx)) != 0:
                continue
            if (self.walls[ny, nx] & self.bit_for_dir(self.opposite_dir(idx))) != 0:
                continue
            # cella bloccata nera
            if self.visited[ny, nx] == 2:
                continue
            res.append((nx, ny))
        return res

    """ 
        True if we can go from (x,y) to (nx,ny) now. (Considering walls and obstacles)
    """
    def can_step_to(self, x, y, nx, ny):
        
        dx, dy = nx - x, ny - y
        if (abs(dx) + abs(dy)) != 1:
            return False
        if not (0 <= nx < self.w and 0 <= ny < self.h):
            return False
        # determina direzione
        if dy == -1: dir_idx = 0
        elif dx == 1: dir_idx = 1
        elif dy == 1: dir_idx = 2
        else: dir_idx = 3
        if (self.walls[y, x] & self.bit_for_dir(dir_idx)) != 0:
            return False
        if (self.walls[ny, nx] & self.bit_for_dir(self.opposite_dir(dir_idx))) != 0:
            return False
        if self.visited[ny, nx] == 2:
            return False
        return True

    """
        Finds the best path using the dijkstra algorithm
    """
    def dijkstra(self, start, goal):
        import heapq
        sx, sy = start
        gx, gy = goal
        if not (0 <= gx < self.w and 0 <= gy < self.h):
            return None
        if self.cell_cost(self.visited[gy, gx]) == np.inf:
            return None
        dist = np.full((self.h, self.w), np.inf, dtype=float)
        prev = np.full((self.h, self.w, 2), -1, dtype=int)
        pq = []
        dist[sy, sx] = 0.0
        heapq.heappush(pq, (0.0, (sx, sy)))
        while pq:
            d, (cx, cy) = heapq.heappop(pq)
            if d != dist[cy, cx]:
                continue
            if (cx, cy) == (gx, gy):
                break
            for nx, ny in self.neighbors(cx, cy):
                w = self.cell_cost(self.visited[ny, nx])
                nd = d + w
                if nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    prev[ny, nx] = [cx, cy]
                    heapq.heappush(pq, (nd, (nx, ny)))
        if not np.isfinite(dist[gy, gx]):
            return None
        # ricostruisci path
        path = []
        cx, cy = gx, gy
        while not (cx == sx and cy == sy):
            path.append((cx, cy))
            pcx, pcy = prev[cy, cx]
            if pcx < 0:
                return None
            cx, cy = int(pcx), int(pcy)
        path.reverse()
        return path

    """
        Returns the coordinate of the bottom right cell 
    """
    def bottom_right_cell(self):
        return (self.w - 1, self.h - 1)

    """
        Choose which direction to take in order to get to the target point
    """
    def face_towards(self, nx, ny):
        dx = np.sign(nx - self.x)
        dy = np.sign(ny - self.y)
        # determina direzione target (N,E,S,W)
        if dy < 0:
            want = 0
        elif dy > 0:
            want = 2
        elif dx > 0:
            want = 1
        else:
            want = 3
        # ruota nel modo più semplice
        while self.dir_index_from_deg(self.deg) != want:
            cur = self.dir_index_from_deg(self.deg)
            diff = (want - cur) % 4
            if diff == 3:
                self.rotate_left()
            else:
                self.rotate_right()

    def follow_path(self, path, on_step=None, stop_when_home=False):
        """
        Segui il path passo-passo con rilevamento preventivo dei muri.
        Ritorna True se completato, False se bloccato/interrotto.
        """
        for idx, (nx, ny) in enumerate(path):
            if self.lop_event.is_set():
                print("Lop rilevato nel follow path, interrompo movimento")
                return False

            print(f"::::::::: STEP {idx+1}/{len(path)}: verso ({nx},{ny}) ::::::::::")

            self.shared_search_victim.value = True
            
            print("::::::::: SCANSIONE MURI PRIMA DEL MOVIMENTO ::::::::::")
            self.get_walls("m001,", 0)  # avanti
            self.get_walls("m002,", 1)  # destra
            self.get_walls("m003,", 2)  # dietro
            self.get_walls("m004,", 3)  # sinistra
            
            if not self.can_step_to(self.x, self.y, nx, ny):
                print(f"::::::::: BLOCCATO! Muro tra ({self.x},{self.y}) e ({nx},{ny}) ::::::::::")
                print("::::::::: Necessario ricalcolo percorso ::::::::::")
                return False
            
            print("::::::::: Orientamento verso destinazione ::::::::::")
            self.face_towards(nx, ny)
            
            print("::::::::: Tentativo movimento avanti ::::::::::")
            has_moved = self.forward()
            
            if not has_moved:
                print("::::::::: MOVIMENTO FALLITO (conferma Arduino negativa o muro) ::::::::::")
                return False
            
            print(f"::::::::: Movimento riuscito! Nuova posizione: ({self.x},{self.y}) ::::::::::")
            
            print("::::::::: Controllo colore casella ::::::::::")
            self.get_color()
            
            print("::::::::: Controllo inclinazione ::::::::::")
            self.check_inclination()
            
            # ========== PAUSA E INPUT UTENTE ==========
            if on_step is not None:
                should_stop = on_step()
                if should_stop:
                    print("::::::::: INTERROTTO DA INPUT UTENTE ::::::::::")
                    return False
            
            if stop_when_home and (self.x, self.y) == self.home:
                print("::::::::: RITORNO A CASA COMPLETATO ::::::::::")
                return True
        
        print("::::::::: PERCORSO COMPLETATO ::::::::::")
        return True

    # ---------- rendering ----------
    """
        Draws and renders the matrix
    """
    def render(self):
        # viewport centrato sul player, clamp ai bordi
        start_c = max(0, min(self.w - VIEW_COLS, self.x - VIEW_COLS // 2)) if self.w > VIEW_COLS else 0
        start_r = max(0, min(self.h - VIEW_ROWS, self.y - VIEW_ROWS // 2)) if self.h > VIEW_ROWS else 0

        img = np.full((VIEW_H, VIEW_W, 3), 255, dtype=np.uint8)

        # disegna riempimenti celle
        for vr in range(VIEW_ROWS):
            gy = start_r + vr
            if gy >= self.h:
                break
            for vc in range(VIEW_COLS):
                gx = start_c + vc
                if gx >= self.w:
                    break
                val = self.visited[gy, gx]
                if val == 0:
                    continue
                x0 = vc * CELL
                y0 = vr * CELL
                x1 = x0 + CELL - 1
                y1 = y0 + CELL - 1
                if val == 1:
                    cv2.rectangle(img, (x0, y0), (x1, y1), COLOR_YELLOW, thickness=-1)
                elif val == 2:
                    cv2.rectangle(img, (x0, y0), (x1, y1), COLOR_BLACK, thickness=-1)
                elif val == 3:
                    cv2.rectangle(img, (x0, y0), (x1, y1), COLOR_BLUE, thickness=-1)
                elif val == 4:
                    cv2.rectangle(img, (x0, y0), (x1, y1), COLOR_GREEN, thickness=-1)
                elif val == 5:
                    cv2.rectangle(img, (x0, y0), (x1, y1), COLOR_GREY, thickness=-1)

        # griglia
        for r in range(VIEW_ROWS + 1):
            y = r * CELL
            cv2.line(img, (0, y), (VIEW_W, y), GRID_COLOR, 1)
        for c in range(VIEW_COLS + 1):
            x = c * CELL
            cv2.line(img, (x, 0), (x, VIEW_H), GRID_COLOR, 1)

        # muri interni
        inset = max(2, CELL // 8)
        thick = max(2, CELL // 10)
        for vr in range(VIEW_ROWS):
            gy = start_r + vr
            if gy >= self.h:
                break
            for vc in range(VIEW_COLS):
                gx = start_c + vc
                if gx >= self.w:
                    break
                wmask = self.walls[gy, gx]
                if wmask == 0:
                    continue
                x0 = vc * CELL
                y0 = vr * CELL
                x1 = x0 + CELL - 1
                y1 = y0 + CELL - 1
                # N
                if wmask & 1:
                    cv2.line(img, (x0+inset, y0+inset), (x1-inset, y0+inset), (0,0,0), thick)
                # E
                if wmask & 2:
                    cv2.line(img, (x1-inset, y0+inset), (x1-inset, y1-inset), (0,0,0), thick)
                # S
                if wmask & 4:
                    cv2.line(img, (x0+inset, y1-inset), (x1-inset, y1-inset), (0,0,0), thick)
                # W
                if wmask & 8:
                    cv2.line(img, (x0+inset, y0+inset), (x0+inset, y1-inset), (0,0,0), thick)

        # player nel viewport
        px = self.x - start_c
        py = self.y - start_r
        if 0 <= px < VIEW_COLS and 0 <= py < VIEW_ROWS:
            cx = int(px * CELL + CELL / 2)
            cy = int(py * CELL + CELL / 2)
            radius = int(CELL * 0.35)
            cv2.circle(img, (cx, cy), radius, COLOR_PLAYER, thickness=-1)
            dx, dy = self.heading_to_delta()
            tip = (int(cx + dx * radius * 0.9), int(cy + dy * radius * 0.9))
            cv2.line(img, (cx, cy), tip, (0, 0, 0), 2)

        # HUD
        hud = (
            f"Pos: ({self.x},{self.y})  Gradi: {self.deg}  Matrice: {self.w}x{self.h}  "
            f"Walls: {int(self.walls[self.y,self.x])}  "
            f"(W=avanti A=sinistra D=destra S=flip  1-4=muri  N=blocca avanti  B/G/M=colora qui  T=verso basso-destra  E=casa  Q/ESC=esci)"
        )
        cv2.rectangle(img, (0, 0), (VIEW_W, 24), (245, 245, 245), -1)
        cv2.putText(img, hud, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, TXT, 1, cv2.LINE_AA)

        return img
