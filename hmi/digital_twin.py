"""Gemelo digital 2D isométrico dibujado con QPainter, sin motor 3D externo."""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QFont, QPainter, QPen
from PyQt6.QtWidgets import QWidget

from kinematics import forward


class DigitalTwin(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(420, 360)
        self.joints_deg = [0.0] * 5
        self.teach_points: list[tuple[str, list[float]]] = []
        self.model_valid = False

    def set_robot_state(self, joints_deg: list[float], teach_points: list[tuple[str, list[float]]]) -> None:
        self.joints_deg = list(joints_deg)
        self.teach_points = [(name, list(joints)) for name, joints in teach_points]
        self.model_valid = True
        self.update()

    @staticmethod
    def _project(point: tuple[float, float, float], center_x: float, center_y: float, scale: float) -> tuple[float, float]:
        x, y, z = point
        iso_x = (x - y) * 0.7071
        iso_y = (x + y) * 0.32 + z
        return center_x + iso_x * scale, center_y - iso_y * scale

    def paintEvent(self, _event) -> None:  # type: ignore # noqa: N802 - Qt event naming
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#101820"))
        painter.setPen(QPen(QColor("#d9e2ec"), 1))
        painter.setFont(QFont("Segoe UI", 10))
        painter.drawText(16, 24, "Gemelo cinemático · escala isométrica")

        try:
            current_pose = forward(self.joints_deg)
            current_points = list(current_pose.link_points_mm)
            saved = [(name, forward(joints).position_mm) for name, joints in self.teach_points]
        except (ValueError, IndexError):
            painter.drawText(16, 52, "Revisa el modelo DH en robot_config.py")
            return

        all_points = current_points + [point for _, point in saved]
        max_extent = max(250.0, max((max(abs(v) for v in p) for p in all_points), default=250.0))
        scale = min((self.width() - 60) / (2.0 * max_extent), (self.height() - 80) / (2.0 * max_extent))
        cx, cy = self.width() * 0.52, self.height() * 0.58

        # Plano de referencia ligero.
        grid_pen = QPen(QColor("#293845"), 1)
        painter.setPen(grid_pen)
        for offset in range(-3, 4):
            a = self._project((offset * 80.0, -240.0, 0.0), cx, cy, scale)
            b = self._project((offset * 80.0, 240.0, 0.0), cx, cy, scale)
            c = self._project((-240.0, offset * 80.0, 0.0), cx, cy, scale)
            d = self._project((240.0, offset * 80.0, 0.0), cx, cy, scale)
            painter.drawLine(round(a[0]), round(a[1]), round(b[0]), round(b[1]))
            painter.drawLine(round(c[0]), round(c[1]), round(d[0]), round(d[1]))

        # Ejes de referencia del mundo.
        origin = self._project((0.0, 0.0, 0.0), cx, cy, scale)
        for end, color, label in (((100.0, 0.0, 0.0), "#ef5350", "X"),
                                  ((0.0, 100.0, 0.0), "#66bb6a", "Y"),
                                  ((0.0, 0.0, 100.0), "#42a5f5", "Z")):
            projected = self._project(end, cx, cy, scale)
            painter.setPen(QPen(QColor(color), 2))
            painter.drawLine(round(origin[0]), round(origin[1]), round(projected[0]), round(projected[1]))
            painter.drawText(round(projected[0]) + 4, round(projected[1]) - 4, label)

        # Eslabones actuales.
        painter.setPen(QPen(QColor("#80cbc4"), 7, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        screen_points = [self._project(point, cx, cy, scale) for point in current_points]
        for start, end in zip(screen_points, screen_points[1:]):
            painter.drawLine(round(start[0]), round(start[1]), round(end[0]), round(end[1]))
        painter.setBrush(QColor("#e0f2f1"))
        painter.setPen(QPen(QColor("#102027"), 1))
        for x, y in screen_points:
            painter.drawEllipse(round(x - 5), round(y - 5), 10, 10)

        # Puntos guardados en el teach pendant.
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.DemiBold))
        for name, point in saved:
            x, y = self._project(point, cx, cy, scale)
            painter.setPen(QPen(QColor("#ffca28"), 2))
            painter.setBrush(QColor("#ffca28"))
            painter.drawEllipse(round(x - 4), round(y - 4), 8, 8)
            painter.drawText(round(x + 7), round(y - 5), name)

        painter.setPen(QPen(QColor("#cfd8dc"), 1))
        painter.setFont(QFont("Consolas", 9))
        painter.drawText(16, self.height() - 14,
                         f"J: {', '.join(f'{v:.1f}°' for v in self.joints_deg)}  |  TCP: "
                         f"{current_pose.position_mm[0]:.1f}, {current_pose.position_mm[1]:.1f}, "
                         f"{current_pose.position_mm[2]:.1f} mm")
