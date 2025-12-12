from pydualsense import pydualsense

# https://flok.github.io/pydualsense/ds_main.html
class PhysicalControllerHandler:
    def __init__(self):
        self.controller = pydualsense()
        self.controller.init()

    def close(self):
        self.controller.close()

    def __del__(self):
        self.close()
        print('closed')
