from picamera2 import Picamera2

class SideCamera:
    def __init__(self, par_id):
        self.cam_id = par_id
        self.cam = Picamera2(camera_num=self.cam_id)  # ? Crea la camera
        configuration = self.cam.create_still_configuration(
            main={"size": (3280, 2400)}
        )
        self.cam.configure(configuration)
        self.cam.start()  # ? Avvia dopo la configurazione
        print(f"? Camera {self.cam_id} avviata correttamente")

    def capture(self, par_file_name):
        self.cam.capture_file(par_file_name)  # ? Usa la cam gi� avviata
        print(f"?? Foto salvata: {par_file_name}")

    def close(self):
        """Chiudi la camera quando non serve pi"""
        if self.cam:
            self.cam.stop()
            self.cam.close()
            print(f"Camera {self.cam_id} chiusa")
