"""
Controller class for desktop environments using keyboard and mouse inputs.
"""

from pynput import keyboard, mouse

class DesktopController:

    def start(self):
        self.keyboard_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.mouse_listener = mouse.Listener(on_move=self.on_move, on_click=self.on_click, on_scroll=self.on_scroll)
        self.keyboard_listener.start()
        self.mouse_listener.start()

    def stop(self):
        pass

    def on_press(self, key):
        pass

    def on_release(self, key):
        pass

    def on_move(self, x, y):
        pass

    def on_click(self, x, y, button, pressed):
        pass

    def on_scroll(self, x, y, dx, dy):
        pass


if __name__ == '__main__':
    DesktopController()
    from time import sleep
    sleep(1000)
