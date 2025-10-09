# ================== CONFIG ==================
VIEW_W, VIEW_H = 640, 480      # finestra/viewport fissa
CELL = 20                      # dimensione cella costante
VIEW_COLS = VIEW_W // CELL     # 32
VIEW_ROWS = VIEW_H // CELL     # 24
INIT_ROWS, INIT_COLS = VIEW_ROWS, VIEW_COLS  # start 24x32

STEP_DELAY_MS = 220            # pausa tra step automatici
INPUT_POLL_MS = 35             # polling input durante la pausa

GRID_COLOR = (180, 180, 180)
TXT = (30, 30, 30)

# Colori celle
COLOR_YELLOW = (0, 255, 255)   # visitata
COLOR_BLACK  = (0, 0, 0)       # bloccata
COLOR_BLUE   = (255, 0, 0)     # speciale B
COLOR_GREEN  = (0, 255, 0)     # speciale G
COLOR_GREY   = (128, 128, 128) # speciale M
COLOR_PLAYER = (0, 200, 0)

# Tasti (WASD)
KEY_W = ord('w'); KEY_WU = ord('W')
KEY_A = ord('a'); KEY_AU = ord('A')
KEY_S = ord('s'); KEY_SU = ord('S')
KEY_D = ord('d'); KEY_DU = ord('D')

# Stati celle in "visited"
# 0 = bianco (mai vista)
# 1 = giallo (visitata)
# 2 = nero (bloccata con 4 muri)
# 3 = blu (B)
# 4 = verde (G)
# 5 = grigio (M)

# ============================================