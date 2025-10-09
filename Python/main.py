
from imps import *

# Stati celle in "visited"
# 0 = bianco (mai vista)
# 1 = giallo (visitata)
# 2 = nero (bloccata con 4 muri)
# 3 = blu (B)
# 4 = verde (G)
# 5 = grigio (M)

def main():
    world = World()
    cv2.namedWindow('Campo', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Campo', VIEW_W, VIEW_H)

    def handle_key(k):
        """Gestisce input. Restituisce 'quit' o 'repath' solo quando serve fermare l'autopilota.
        - AZIONI DI MODIFICA MAPPA (1-4, N, B, G, M): applica subito ma NON ferma l'autopilota;
          se crei un ostacolo, al passo successivo il path fallirà e verrà ricalcolato.
        - AZIONI MANUALI DI MOVIMENTO (W/A/D/S) o cambio target (T/E): fermano e richiedono ricalcolo.
        """
        if k == ord('q') or k == 27:
            return 'quit'
        # --- controllo manuale: interrompe autopilota ---
        elif k in (KEY_W, KEY_WU):
            world.forward(); return 'repath'
        elif k in (KEY_A, KEY_AU):
            world.rotate_left(); return 'repath'
        elif k in (KEY_D, KEY_DU):
            world.rotate_right(); return 'repath'
        elif k in (KEY_S, KEY_SU):
            world.flip_direction(); return 'repath'
        # --- modifica mappa: NON interrompe ---
        elif k == ord('1'):
            world.set_wall_relative(0, toggle=True); return None
        elif k == ord('2'):
            world.set_wall_relative(1, toggle=True); return None
        elif k == ord('3'):
            world.set_wall_relative(2, toggle=True); return None
        elif k == ord('4'):
            world.set_wall_relative(3, toggle=True); return None
        elif k == ord('n') or k == ord('N'):
            dx, dy = world.heading_to_delta()
            nx, ny = world.x + dx, world.y + dy
            nx, ny = world.ensure_inside(nx, ny)
            for dir_idx in range(4):
                world.set_wall_absolute(nx, ny, dir_idx, 1)
            world.visited[ny, nx] = 2
            return None
        elif k == ord('b') or k == ord('B'):
            world.visited[world.y, world.x] = 3; return None
        elif k == ord('g') or k == ord('G'):
            world.visited[world.y, world.x] = 4; return None
        elif k == ord('m') or k == ord('M'):
            world.visited[world.y, world.x] = 5; return None
        # --- cambio obiettivo: interrompe ---
        elif k == ord('t') or k == ord('T'):
            world.target = world.bottom_right_cell()
            world.mode = 'to_target'
            return 'repath'
        elif k == ord('e') or k == ord('E'):
            world.target = world.home
            world.mode = 'to_home'
            return 'repath'
        return None

    def on_step():
        # pausa in cui ascolto input per permettere muro/colori in corsa
        end_t = time.time() + (STEP_DELAY_MS / 1000.0)
        need_stop = False
        while time.time() < end_t and not need_stop:
            frame = world.render()
            cv2.imshow('Campo', frame)
            k = cv2.waitKey(INPUT_POLL_MS) & 0xFF
            if k == 255:
                continue
            res = handle_key(k)
            if res in ('quit', 'repath'):
                need_stop = True
        return need_stop

    while True:
        frame = world.render()
        cv2.imshow('Campo', frame)
        key = cv2.waitKey(0) & 0xFF

        res = handle_key(key)
        if res == 'quit':
            break

        # autopilota semplice: T/E per avviare; se bloccato, premi T/E di nuovo per ricalcolare
        if world.mode in ('to_target', 'to_home'):
            start = (world.x, world.y)
            goal = world.target if world.mode == 'to_target' else world.home
            path = world.dijkstra(start, goal)
            if path is None or len(path) == 0:
                world.mode = 'idle'
                continue
            completed = world.follow_path(path, on_step=on_step, stop_when_home=(world.mode=='to_home'))
            if not completed:
                # si è bloccato: attendi nuovo T/E per ripartire
                continue
            if world.mode == 'to_home' and (world.x, world.y) == world.home:
                break
            world.mode = 'idle'

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
