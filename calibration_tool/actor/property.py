from PyQt5.QtGui import QColor


class Property(object):

    def __init__(self):
        self.color = QColor(255, 255, 255)

        self.line_width = 1.5

        self.point_size = 1.0

    def set_color(self, r: int, g: int, b: int):
        self.color.setRgb(r, g, b)

    def set_line_width(self, line_width: float):
        self.line_width = line_width

    def set_point_size(self, point_size: float):
        self.point_size = point_size
