import sys
from typing import Type

from PySide6.QtGui import QColor, QPalette, Qt
from PySide6.QtWidgets import QWidget

class ColoredRegion(QWidget):
    """
    Simple colored region for testing the boundaries of layouts.
    """
    def __init__(self, color):
        super().__init__()

        # Will only work if qt-material is not applied
        palette = self.palette()
        palette.setColor(QPalette.ColorRole.Window, QColor(color))
        self.setPalette(palette)

        # Support qt-material style
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setProperty('class', 'test_panel')


def test_widget(widget_class: Type[QWidget],
                params: tuple,
                theme: str | None = None,
                css_file: str = 'styles.css') -> None:
    """
    Creates an application containing the widget given as the central widget.
    :param widget_class: the widget to create an instance of.
    :param params: the parameters to pass to the widget constructor.
    :param theme: what qt-material theme to use, None to not use qt-material.
    """
    from PySide6.QtWidgets import QApplication, QMainWindow
    app = QApplication(sys.argv)

    window = QMainWindow()
    window.setCentralWidget(widget_class(*params))

    if theme:
        from qt_material import apply_stylesheet
        apply_stylesheet(app, theme=theme, css_file=css_file)
    window.show()

    app.exec()

if __name__ == '__main__':
    test_widget(ColoredRegion, ('red',))
    # test_widget(ColoredRegion, ('red',), 'dark_teal.xml')
