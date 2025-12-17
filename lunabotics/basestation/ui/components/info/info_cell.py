from PySide6.QtGui import QPalette, Qt, QColor
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout

from lunabotics.basestation.ui.components.util import test_widget


class InfoCell(QWidget):
    """ A panel that represents a single value. """
    def __init__(self, value_name: str, unit: str, dp: float = 2):
        super().__init__()
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setProperty('class', 'info_cell_panel')

        # Init
        self.value_name = value_name
        self.unit = unit
        self.dp = dp
        self.value = 0

        self.layout = QVBoxLayout()
        self.layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setLayout(self.layout)

        # Value name
        self.value_name_lbl = QLabel(f'{self.value_name}:')
        self.layout.addWidget(self.value_name_lbl)
        self.value_name_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.value_lbl = QLabel()
        self.layout.addWidget(self.value_lbl)
        self.value_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.set_value('5')

    def set_value(self, value):
        self.value = value
        self.value_lbl.setText(self.get_formatted_value())

    def get_formatted_value(self):
        return f'{self.value}{self.unit}'


if __name__ == '__main__':
    # test_widget(InfoCell, ('Speed', 'm/s'))
    test_widget(InfoCell, ('Speed', 'm/s'), 'dark_teal.xml', '../styles.css')
