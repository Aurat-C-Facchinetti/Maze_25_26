from imps import *

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

    # ---------- util direzione ----------
    @staticmethod
    def dir_index_from_deg(deg):
        # 0=N,1=E,2=S,3=W
        return ((deg // 90) % 4)

    @staticmethod
    def bit_for_dir(idx):
        return [1, 2, 4, 8][idx]

    @staticmethod
    def opposite_dir(idx):
        return (idx + 2) % 4

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

    def normalize_deg(self):
        self.deg = ((self.deg % 360) + 360) % 360

    # ---------- azioni player ----------
    def rotate_left(self):
        self.deg -= 90
        self.normalize_deg()

    def rotate_right(self):
        self.deg += 90
        self.normalize_deg()

    def flip_direction(self):
        self.deg += 180
        self.normalize_deg()

    def visit_current(self):
        # non sovrascrivere stati speciali (2..5)
        if self.visited[self.y, self.x] == 0:
            self.visited[self.y, self.x] = 1

    def expand_double(self):
        """Raddoppia la matrice mantenendo dati centrati (visited + walls).
        Ritorna lo shift applicato (off_x, off_y) per riallineare coordinate target.
        """
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

    def ensure_inside(self, gx, gy):
        """Espandi finché (gx,gy) entra. Restituisce le coordinate riallineate (gx,gy).
        Se supera il limite massimo, non espande e ritorna le coordinate clampate.
        """
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

    def set_wall_absolute(self, gx, gy, dir_idx, value=1):
        bit = self.bit_for_dir(dir_idx)
        if value:
            self.walls[gy, gx] |= bit
        else:
            self.walls[gy, gx] &= (~bit) & 0xF

    def toggle_wall_absolute(self, gx, gy, dir_idx):
        bit = self.bit_for_dir(dir_idx)
        self.walls[gy, gx] ^= bit

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
            self.x, self.y = nx, ny
            self.visit_current()
        return moved

    # ---------- info celle ----------
    def cell_cost(self, val):
        """Costo di ingresso nella cella in base allo stato.
        0 (bianca) -> 1 Inesplorata
        1 (gialla) -> 2 Esplorata vuota
        2 (nera)   -> (np.inf) Impassabile
        3 (blu)    -> 3 Devi fermarti 5 secondi (TODO: Check regolamento)
        4 (verde)  -> 3 Salita / Discesa
        5 (grigia) -> 2 Checkpoint
        """
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

    def get_cell_info(self, gx, gy):
        """Ritorna un dizionario con info utili sulla cella."""
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

    def can_step_to(self, x, y, nx, ny):
        """True se si può andare da (x,y) a (nx,ny) ora (rispetta muri e blocchi)."""
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

    def bottom_right_cell(self):
        return (self.w - 1, self.h - 1)

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
        """Segui il path passo-passo. Ritorna True se completato, False se bloccato/interrotto."""
        for (nx, ny) in path:
            # orienta verso la prossima cella e prova ad avanzare
            self.face_towards(nx, ny)
            if not self.forward():
                return False
            # pausa e chance di input
            if on_step is not None and on_step():
                return False
            if stop_when_home and (self.x, self.y) == self.home:
                return True
        return True

    # ---------- rendering ----------
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
