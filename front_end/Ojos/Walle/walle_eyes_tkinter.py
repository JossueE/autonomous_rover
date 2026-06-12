import tkinter as tk
from pathlib import Path
import random


class WalleEyesApp:
    """
    Ojos estilo WALL-E usando solo tkinter.
    No necesita pygame ni librerias externas.

    Coloca este archivo .py en la misma carpeta que estas imagenes:
    - Abiertos.PNG
    - Cerrando.PNG
    - Cerrados.PNG
    - Abiertos_llorando.PNG
    - Cerrando_llorando.PNG
    - Cerrados_llorando.PNG
    """

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Ojos WALL-E")
        self.root.configure(bg="black")

        self.base_path = Path(__file__).resolve().parent

        self.images = {
            "open": self.load_image("Abiertos.PNG"),
            "closing": self.load_image("Cerrando.PNG"),
            "closed": self.load_image("Cerrados.PNG"),
            "cry_open": self.load_image("Abiertos_llorando.PNG"),
            "cry_closing": self.load_image("Cerrando_llorando.PNG"),
            "cry_closed": self.load_image("Cerrados_llorando.PNG"),
        }

        self.is_crying = False
        self.is_blinking = False
        self.is_fullscreen = False

        # Toma el tamano de la imagen abierta como tamano base de ventana.
        self.image_width = self.images["open"].width()
        self.image_height = self.images["open"].height()

        self.canvas = tk.Canvas(
            self.root,
            width=self.image_width,
            height=self.image_height,
            bg="black",
            highlightthickness=0
        )
        self.canvas.pack(expand=True, fill="both")

        self.image_on_canvas = self.canvas.create_image(
            self.image_width // 2,
            self.image_height // 2,
            image=self.images["open"],
            anchor="center"
        )

        self.root.geometry(f"{self.image_width}x{self.image_height}")
        self.center_window()

        self.bind_keys()
        self.show_open()
        self.schedule_next_blink()

    def load_image(self, filename):
        path = self.base_path / filename
        if not path.exists():
            raise FileNotFoundError(
                f"No encontre {filename}. Pon la imagen en la misma carpeta que este .py"
            )
        return tk.PhotoImage(file=str(path))

    def center_window(self):
        self.root.update_idletasks()
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = max((screen_width - self.image_width) // 2, 0)
        y = max((screen_height - self.image_height) // 2, 0)
        self.root.geometry(f"{self.image_width}x{self.image_height}+{x}+{y}")

    def bind_keys(self):
        self.root.bind("<Escape>", lambda event: self.root.destroy())
        self.root.bind("<space>", lambda event: self.blink())
        self.root.bind("c", lambda event: self.collision_detected())
        self.root.bind("C", lambda event: self.collision_detected())
        self.root.bind("r", lambda event: self.recover())
        self.root.bind("R", lambda event: self.recover())
        self.root.bind("f", lambda event: self.toggle_fullscreen())
        self.root.bind("F", lambda event: self.toggle_fullscreen())

    def current_prefix(self):
        return "cry_" if self.is_crying else ""

    def set_frame(self, frame_name):
        image = self.images[frame_name]
        self.canvas.itemconfig(self.image_on_canvas, image=image)
        self.canvas.coords(
            self.image_on_canvas,
            self.canvas.winfo_width() // 2,
            self.canvas.winfo_height() // 2
        )

    def show_open(self):
        if self.is_crying:
            self.set_frame("cry_open")
        else:
            self.set_frame("open")

    def blink(self):
        if self.is_blinking:
            return

        self.is_blinking = True

        if self.is_crying:
            sequence = [
                ("cry_closing", 90),
                ("cry_closed", 120),
                ("cry_closing", 90),
                ("cry_open", 0),
            ]
        else:
            sequence = [
                ("closing", 90),
                ("closed", 120),
                ("closing", 90),
                ("open", 0),
            ]

        self.play_sequence(sequence, index=0)

    def play_sequence(self, sequence, index):
        frame_name, delay_ms = sequence[index]
        self.set_frame(frame_name)

        if index >= len(sequence) - 1:
            self.is_blinking = False
            return

        self.root.after(delay_ms, lambda: self.play_sequence(sequence, index + 1))

    def schedule_next_blink(self):
        delay = random.randint(2200, 5200)
        self.root.after(delay, self.auto_blink)

    def auto_blink(self):
        self.blink()
        self.schedule_next_blink()

    def collision_detected(self):
        """
        Simula que el robot choco.
        En ROS2, esta funcion se llamaria desde el callback del sensor de choque.
        """
        self.is_crying = True
        self.show_open()

    def recover(self):
        """
        Quita el estado de llanto.
        """
        self.is_crying = False
        self.show_open()

    def toggle_fullscreen(self):
        self.is_fullscreen = not self.is_fullscreen
        self.root.attributes("-fullscreen", self.is_fullscreen)

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = WalleEyesApp()
    app.run()
