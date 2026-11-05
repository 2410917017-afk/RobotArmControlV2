"""Interfaz PyQt6 para jogging, IK/FK, teach pendant, repetición y gemelo digital."""

from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from serial.tools import list_ports

from digital_twin import DigitalTwin
from kinematics import forward, inverse
from robot_config import (
    DEFAULT_CARTESIAN_JOG_MM,
    DEFAULT_FEED_STEPS_S,
    DEFAULT_JOINT_JOG_DEG,
    DEFAULT_ANGULAR_JOG_DEG,
    JOINT_LIMITS_DEG,
    JOINT_NAMES,
    MAX_FEED_STEPS_S,
)
from serial_worker import SerialWorker
from teach_store import load_points, save_points


class RobotHMI(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Control 5 GDL · HMI")
        self.resize(1440, 900)

        self.worker: SerialWorker | None = None
        self.request_id = 0
        self.current_joints = [0.0] * 5
        self.position_valid = False
        self.tool_open = False
        self.homed = False
        self.points: list[dict] = []
        self.sequence_active = False
        self.sequence_points: list[dict] = []
        self.sequence_index = 0
        self.sequence_repeats_left = 0
        self.sequence_move_id: int | None = None
        self.sequence_command_ids: set[int] = set()
        self.cartesian_dirty = False
        self._closing_after_disconnect = False

        self._build_ui()
        self._refresh_ports()
        self._set_connected(False)

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._send_status)
        self.poll_timer.start(1000)
        #eef2f5
    def _build_ui(self) -> None:
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #00011a; color: #F5F5DC; }
            QGroupBox {background: #0b0c2d; border: 1px solid #bdc8d2; border-radius: 8px; margin-top: 10px; padding: 10px; font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
            QPushButton { min-height: 30px; padding: 4px 10px; border-radius: 5px; background: #ff3509; }
            QPushButton:hover { background: #c6d8e5; }
            QPushButton:disabled { color: #89949e; background: #e2e6e9; }
            QPushButton#stopButton { background: #b71c1c; color: white; font-weight: bold; }
            QPushButton#homeButton { background: #546e7a; color: white; font-weight: bold; }
            QHeaderView::section { padding: 5px; background: #dce5eb; border: 0; }
            QTabBar::tab { padding: 10px 18px; }
        """)

        root = QWidget()
        root_layout = QGridLayout(root)
        self.setCentralWidget(root)

        toolbar = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(250)
        self.refresh_ports_button = QPushButton("Actualizar puertos")
        self.refresh_ports_button.clicked.connect(self._refresh_ports)
        self.connect_button = QPushButton("Conectar")
        self.connect_button.clicked.connect(self._toggle_connection)
        self.connection_label = QLabel("Desconectado")
        self.connection_label.setMinimumWidth(130)
        self.home_button = QPushButton("HOME · pose actual")
        self.home_button.setObjectName("homeButton")
        self.home_button.setToolTip("Asigna cero de software a la pose actual; no mueve el brazo")
        self.home_button.clicked.connect(self._send_home)
        self.stop_button = QPushButton("PARO")
        self.stop_button.setObjectName("stopButton")
        self.stop_button.clicked.connect(self._send_stop)
        toolbar.addWidget(QLabel("Puerto ESP32:"))
        toolbar.addWidget(self.port_combo)
        toolbar.addWidget(self.refresh_ports_button)
        toolbar.addWidget(self.connect_button)
        toolbar.addWidget(self.connection_label)
        toolbar.addStretch(1)
        toolbar.addWidget(self.home_button)
        toolbar.addWidget(self.stop_button)
        root_layout.addLayout(toolbar, 0, 0)

        self.tabs = QTabWidget()
        root_layout.addWidget(self.tabs, 1, 0)
        self._build_manual_tab()
        self._build_teach_tab()
        self._build_twin_tab()

        self.log = QTableWidget(0, 2)
        self.log.setColumnWidth(0, 50)
        self.log.setMaximumWidth(300)
        self.log.setHorizontalHeaderLabels(["Hora", "Evento / diagnóstico"])
        self.log.horizontalHeader().setStretchLastSection(True)
        self.log.setStyleSheet("""color:black; background:white""")
        #self.log.setMaximumHeight(105)
        self.log.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        root_layout.addWidget(self.log, 0, 1, 2, 1)
        self.statusBar().showMessage("HOME del ESP32 es una referencia de software en la pose actual.")

    def _build_manual_tab(self) -> None:
        page = QWidget()
        layout = QGridLayout(page)
        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 3)

        joint_group = QGroupBox("Jog articular · ángulos de salida")
        joint_layout = QGridLayout(joint_group)
        joint_layout.addWidget(QLabel("Junta"), 0, 0)
        joint_layout.addWidget(QLabel("Posición estimada"), 0, 1)
        joint_layout.addWidget(QLabel("Jog"), 0, 2, 1, 3)
        self.joint_value_labels: list[QLabel] = []
        self.joint_step = QDoubleSpinBox()
        self.joint_step.setRange(0.1, 10.0)
        self.joint_step.setDecimals(1)
        self.joint_step.setSingleStep(0.1)
        self.joint_step.setValue(DEFAULT_JOINT_JOG_DEG)
        self.feed_spin = QSpinBox()
        self.feed_spin.setRange(1, MAX_FEED_STEPS_S)
        self.feed_spin.setValue(DEFAULT_FEED_STEPS_S)
        self.feed_spin.setSuffix(" pasos/s")
        joint_layout.addWidget(QLabel("Incremento"), 7, 0)
        joint_layout.addWidget(self.joint_step, 7, 1)
        joint_layout.addWidget(QLabel("Feed"), 8, 0)
        joint_layout.addWidget(self.feed_spin, 8, 1, 1, 2)

        for index, name in enumerate(JOINT_NAMES):
            row = index + 1
            value = QLabel("0.00°")
            value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            value.setFont(QFont("Consolas", 11, QFont.Weight.Bold))
            self.joint_value_labels.append(value)
            minus = QPushButton("−")
            plus = QPushButton("+")
            minus.clicked.connect(lambda _checked=False, i=index: self._jog_joint(i, -1))
            plus.clicked.connect(lambda _checked=False, i=index: self._jog_joint(i, 1))
            nombre = QLabel(name)
            joint_layout.addWidget(nombre, row, 0)
            joint_layout.addWidget(value, row, 1)
            joint_layout.addWidget(minus, row, 2)
            joint_layout.addWidget(plus, row, 3)
            joint_layout.addWidget(QLabel(f"{JOINT_LIMITS_DEG[index][0]:g}° a {JOINT_LIMITS_DEG[index][1]:g}°"), row, 4)

        cart_group = QGroupBox("Cinemática inversa · XYZ + orientación del eje herramienta")
        cart_layout = QGridLayout(cart_group)
        self.cart_fields: dict[str, QDoubleSpinBox] = {}
        cart_specs = (
            ("X", "mm", -1000.0, 1000.0),
            ("Y", "mm", -1000.0, 1000.0),
            ("Z", "mm", -1000.0, 1000.0),
            ("Yaw", "°", -180.0, 180.0),
            ("Pitch", "°", -90.0, 90.0),
        )
        self.cart_step = QDoubleSpinBox()
        self.cart_step.setRange(0.1, 50.0)
        self.cart_step.setDecimals(1)
        self.cart_step.setValue(DEFAULT_CARTESIAN_JOG_MM)
        for row, (name, units, low, high) in enumerate(cart_specs):
            spin = QDoubleSpinBox()
            spin.setRange(low, high)
            spin.setDecimals(2)
            spin.setSingleStep(1.0)
            spin.valueChanged.connect(self._mark_cartesian_dirty)
            self.cart_fields[name] = spin
            minus, plus = QPushButton("−"), QPushButton("+")
            minus.clicked.connect(lambda _checked=False, n=name: self._jog_cartesian(n, -1))
            plus.clicked.connect(lambda _checked=False, n=name: self._jog_cartesian(n, 1))
            cart_layout.addWidget(QLabel(f"{name} ({units})"), row, 0)
            cart_layout.addWidget(spin, row, 1)
            cart_layout.addWidget(minus, row, 2)
            cart_layout.addWidget(plus, row, 3)
        cart_layout.addWidget(QLabel("Paso de jog (mm / °)"), 5, 0)
        cart_layout.addWidget(self.cart_step, 5, 1)
        solve_button = QPushButton("Resolver IK y mover")
        solve_button.clicked.connect(self._move_cartesian_target)
        current_button = QPushButton("Cargar pose actual (FK)")
        current_button.clicked.connect(self._load_current_fk)
        cart_layout.addWidget(solve_button, 6, 0, 1, 2)
        cart_layout.addWidget(current_button, 6, 2, 1, 2)

        fk_group = QGroupBox("Cinemática directa de la posición estimada")
        fk_layout = QFormLayout(fk_group)
        self.fk_position_label = QLabel("X: —   Y: —   Z: —")
        self.fk_orientation_label = QLabel("Yaw: —   Pitch: —")
        self.reference_label = QLabel("Referencia: sin lectura")
        fk_layout.addRow("TCP estimado", self.fk_position_label)
        fk_layout.addRow("Orientación", self.fk_orientation_label)
        fk_layout.addRow("Estado", self.reference_label)

        gripper_group = QGroupBox("Gripper servo")
        gripper_layout = QHBoxLayout(gripper_group)
        self.gripper_button = QPushButton("Abrir gripper")
        self.gripper_button.clicked.connect(self._toggle_gripper)
        self.gripper_state_label = QLabel("Estado desconocido")
        gripper_layout.addWidget(self.gripper_button)
        gripper_layout.addWidget(self.gripper_state_label)

        left = QVBoxLayout()
        left.addWidget(joint_group)
        left.addWidget(fk_group)
        left.addStretch(1)
        layout.addLayout(left, 0, 0)
        right = QVBoxLayout()
        right.addWidget(cart_group)
        right.addWidget(gripper_group)
        right.addStretch(1)
        layout.addLayout(right, 0, 1)

        self.tabs.addTab(page, "Manual / IK-FK")

    def _build_teach_tab(self) -> None:
        page = QWidget()
        layout = QVBoxLayout(page)
        hint = QLabel("Guarda la pose articular estimada y el estado del gripper. Las secuencias se guardan como JSON local.")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.points_table = QTableWidget(0, 12)
        self.points_table.setHorizontalHeaderLabels(
            ["Punto", "J1°", "J2°", "J3°", "J4°", "J5°", "X mm", "Y mm", "Z mm", "Yaw°", "Pitch°", "Gripper"]
        )
        self.points_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.points_table.setSelectionMode(QTableWidget.SelectionMode.ExtendedSelection)
        self.points_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.points_table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.points_table, 1)

        actions = QHBoxLayout()
        add_button = QPushButton("Guardar pose actual")
        add_button.clicked.connect(self._add_teach_point)
        delete_button = QPushButton("Eliminar seleccionado")
        delete_button.clicked.connect(self._remove_teach_points)
        clear_button = QPushButton("Vaciar tabla")
        clear_button.clicked.connect(self._clear_teach_points)
        save_button = QPushButton("Guardar archivo…")
        save_button.clicked.connect(self._save_teach_file)
        load_button = QPushButton("Cargar archivo…")
        load_button.clicked.connect(self._load_teach_file)
        actions.addWidget(add_button)
        actions.addWidget(delete_button)
        actions.addWidget(clear_button)
        actions.addStretch(1)
        actions.addWidget(save_button)
        actions.addWidget(load_button)
        layout.addLayout(actions)

        run_row = QHBoxLayout()
        self.repeat_spin = QSpinBox()
        self.repeat_spin.setRange(1, 1000)
        self.repeat_spin.setValue(1)
        self.repeat_spin.setPrefix("Repeticiones: ")
        self.run_sequence_button = QPushButton("Ejecutar secuencia")
        self.run_sequence_button.clicked.connect(self._start_sequence)
        self.stop_sequence_button = QPushButton("Cancelar secuencia")
        self.stop_sequence_button.clicked.connect(self._cancel_sequence)
        self.stop_sequence_button.setEnabled(False)
        run_row.addWidget(self.repeat_spin)
        run_row.addWidget(self.run_sequence_button)
        run_row.addWidget(self.stop_sequence_button)
        run_row.addStretch(1)
        layout.addLayout(run_row)
        self.tabs.addTab(page, "Teach pendant")

    def _build_twin_tab(self) -> None:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.twin = DigitalTwin()
        layout.addWidget(self.twin, 1)
        note = QLabel("Visualización ligera con el DH estándar confirmado; TCP y offsets aún están por calibrar. Las marcas amarillas son puntos teach guardados.")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.tabs.addTab(page, "Gemelo digital")
        self._refresh_twin()

    def _refresh_ports(self) -> None:
        selected = self.port_combo.currentData() if hasattr(self, "port_combo") else None
        self.port_combo.clear()
        found = list(list_ports.comports())
        for port in found:
            self.port_combo.addItem(f"{port.device} — {port.description}", port.device)
        if selected:
            index = self.port_combo.findData(selected)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
        if not found:
            self.port_combo.addItem("No se encontraron puertos", None)

    def _toggle_connection(self) -> None:
        if self.worker is not None:
            self._cancel_sequence(send_stop=False)
            self._send_command({"cmd": "STOP"})
            self.worker.request_stop()
            self.connect_button.setEnabled(False)
            self.connection_label.setText("Desconectando…")
            return
        port = self.port_combo.currentData()
        if not port:
            self._log("error", "Selecciona un puerto serie disponible")
            return
        self.connect_button.setEnabled(False)
        self.connection_label.setText("Conectando…")
        self.worker = SerialWorker(port)
        self.worker.connected.connect(self._on_connected)
        self.worker.disconnected.connect(self._on_disconnected)
        self.worker.finished.connect(self._on_worker_finished)
        self.worker.message_received.connect(self._on_message)
        self.worker.error_received.connect(lambda message: self._log("serie", message))
        self.worker.start()

    def _on_connected(self, port: str) -> None:
        self._set_connected(True)
        self.connection_label.setText(f"Conectado: {port}")
        self.statusBar().showMessage("Puerto serie abierto; consultando estado del ESP32.", 5000)
        self._log("serie", f"Conectado a {port} a 115200 baudios")
        self._send_status()

    def _on_disconnected(self) -> None:
        self._cancel_sequence(send_stop=False)
        self._set_connected(False)
        self.connect_button.setEnabled(False)
        self.connection_label.setText("Desconectado")

    def _on_worker_finished(self) -> None:
        finished_worker = self.sender()
        if finished_worker is self.worker:
            self.worker = None
            finished_worker.deleteLater()
        self._set_connected(False)
        self.connection_label.setText("Desconectado")
        if self._closing_after_disconnect:
            self._closing_after_disconnect = False
            QTimer.singleShot(0, self.close)

    def _set_connected(self, connected: bool) -> None:
        if not hasattr(self, "connect_button"):
            return
        self.connect_button.setText("Desconectar" if connected else "Conectar")
        self.connect_button.setEnabled(True)
        self.connection_label.setText("Conectado" if connected else "Desconectado")
        for button in (self.home_button, self.stop_button, self.gripper_button,
                       self.run_sequence_button, self.stop_sequence_button):
            if button is not None:
                button.setEnabled(connected and (button is not self.stop_sequence_button or self.sequence_active))

    def _send_command(self, command: dict) -> int | None:
        if self.worker is None or not self.worker.isRunning():
            self._log("error", "No hay conexión serie activa")
            return None
        self.request_id += 1
        command["id"] = self.request_id
        self.worker.send_command(command)
        return self.request_id

    def _send_status(self) -> None:
        if self.worker is not None and self.worker.isRunning():
            self._send_command({"cmd": "STATUS"})

    def _send_home(self) -> None:
        self._send_command({"cmd": "HOME"})
        self.statusBar().showMessage("HOME asigna cero de software a la pose detenida actual; no busca finales de carrera.", 7000)

    def _send_stop(self) -> None:
        self._cancel_sequence(send_stop=False)
        self._send_command({"cmd": "STOP"})
        self.statusBar().showMessage("Paro solicitado: desaceleración controlada del firmware.", 5000)

    def _toggle_gripper(self) -> None:
        target_open = not self.tool_open
        self._send_command({"cmd": "TOOL", "tool": target_open})

    def _move_joint_target(self, target: list[float]) -> int | None:
        if not self.position_valid:
            self._log("error", "Espera la lectura de posición del ESP32 antes de mover")
            return None
        for index, value in enumerate(target):
            low, high = JOINT_LIMITS_DEG[index]
            if not low <= value <= high:
                self._log("límite", f"{JOINT_NAMES[index]} fuera del límite provisional {low:g}°…{high:g}°")
                return None
        return self._send_command({
            "cmd": "MOVE",
            "axes": [round(float(value), 3) for value in target],
            "feed": self.feed_spin.value(),
        })

    def _jog_joint(self, index: int, sign: int) -> None:
        target = self.current_joints[:]
        target[index] += sign * self.joint_step.value()
        self._move_joint_target(target)

    def _mark_cartesian_dirty(self, _value: float) -> None:
        self.cartesian_dirty = True

    def _load_current_fk(self) -> None:
        if not self.position_valid:
            self._log("error", "Aún no hay una posición estimada del ESP32")
            return
        self._set_cartesian_fields(forward(self.current_joints))
        self.cartesian_dirty = False

    def _set_cartesian_fields(self, pose) -> None:
        values = {
            "X": pose.position_mm[0],
            "Y": pose.position_mm[1],
            "Z": pose.position_mm[2],
            "Yaw": pose.yaw_deg,
            "Pitch": pose.pitch_deg,
        }
        for key, value in values.items():
            spin = self.cart_fields[key]
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)

    def _move_cartesian_target(self) -> None:
        if not self.position_valid:
            self._log("error", "Conecta y consulta estado antes de calcular IK")
            return
        target_position = [self.cart_fields[key].value() for key in ("X", "Y", "Z")]
        result = inverse(
            target_position,
            self.cart_fields["Yaw"].value(),
            self.cart_fields["Pitch"].value(),
            self.current_joints,
        )
        if not result.success:
            self._log("IK", f"Sin convergencia: error {result.position_error_mm:.1f} mm, "
                      f"{result.orientation_error_deg:.1f}°; revisa alcance y DH")
            return
        self.cartesian_dirty = False
        self._move_joint_target(list(result.joints_deg))

    def _jog_cartesian(self, field: str, sign: int) -> None:
        if not self.position_valid:
            self._log("error", "Conecta y consulta estado antes del jog cartesiano")
            return
        pose = forward(self.current_joints)
        position = list(pose.position_mm)
        yaw, pitch = pose.yaw_deg, pose.pitch_deg
        delta = self.cart_step.value()
        if field in ("X", "Y", "Z"):
            position[("X", "Y", "Z").index(field)] += sign * delta
        elif field == "Yaw":
            yaw = max(-180.0, min(180.0, yaw + sign * delta))
        else:
            pitch = max(-90.0, min(90.0, pitch + sign * delta))
        result = inverse(position, yaw, pitch, self.current_joints)
        if not result.success:
            self._log("IK", f"Jog cartesiano no alcanzable: {result.position_error_mm:.1f} mm, "
                      f"{result.orientation_error_deg:.1f}°")
            return
        self._set_cartesian_fields(forward(result.joints_deg))
        self.cartesian_dirty = False
        self._move_joint_target(list(result.joints_deg))

    def _on_message(self, message: dict) -> None:
        event = str(message.get("event", ""))
        if event == "error":
            self._log("ESP32 error", f"{message.get('code', 'error')} · id={message.get('id', '')}")
            if message.get("id") in self.sequence_command_ids:
                self._cancel_sequence(send_stop=False)
            return

        axes = message.get("axes")
        if isinstance(axes, list) and len(axes) == 5:
            try:
                self.current_joints = [float(axis["joint_deg_est"]) for axis in axes]
                self.position_valid = all(math.isfinite(v) for v in self.current_joints)
            except (KeyError, TypeError, ValueError):
                self.position_valid = False

        if "homed" in message:
            self.homed = bool(message["homed"])
        if "tool_open" in message:
            self.tool_open = bool(message["tool_open"])
            self.gripper_state_label.setText("Abierto" if self.tool_open else "Cerrado")
            self.gripper_button.setText("Cerrar gripper" if self.tool_open else "Abrir gripper")

        if self.position_valid:
            pose = forward(self.current_joints)
            self.fk_position_label.setText(
                f"X {pose.position_mm[0]:.1f} · Y {pose.position_mm[1]:.1f} · Z {pose.position_mm[2]:.1f} mm"
            )
            self.fk_orientation_label.setText(f"Yaw {pose.yaw_deg:.1f}° · Pitch {pose.pitch_deg:.1f}°")
            state = "cero software disponible" if self.homed else "referencia no confirmada"
            self.reference_label.setText(f"{state} · estimación sin encoder")
            for index, value in enumerate(self.current_joints):
                self.joint_value_labels[index].setText(f"{value:+.2f}°")
            if not self.cartesian_dirty:
                self._set_cartesian_fields(pose)
            self._refresh_twin()

        if event == "ready":
            self._log("ESP32", f"Firmware listo · motor={message.get('stepper_engine', 'no informado')} · posición estimada en lazo abierto")
        elif event == "ack":
            if message.get("cmd") != "STATUS":
                self._log("ack", f"{message.get('cmd', '')} aceptado en cola · id={message.get('id', '')}")
        elif event in ("position", "complete", "status", "homed", "stopping", "tool"):
            step_counts = " ".join(
                f"J{index + 1}:{axis.get('steps', '?')}"
                for index, axis in enumerate(axes)
                if isinstance(axes, list) and isinstance(axis, dict)
            ) if isinstance(axes, list) else ""
            suffix = f" · pasos {step_counts}" if step_counts else ""
            self._log(event, f"id={message.get('id', '')} · moving={message.get('moving', False)}{suffix}")

        if event == "complete" and self.sequence_active and message.get("id") == self.sequence_move_id:
            self.sequence_move_id = None
            self.sequence_index += 1
            QTimer.singleShot(100, self._send_next_sequence_point)

    def _add_teach_point(self) -> None:
        if not self.position_valid:
            self._log("error", "No hay posición estimada para guardar")
            return
        used_names = {str(point.get("name", "")) for point in self.points}
        next_number = 1
        while f"P{next_number}" in used_names:
            next_number += 1
        name = f"P{next_number}"
        point = {
            "name": name,
            "joints_deg": [round(v, 3) for v in self.current_joints],
            "tool_open": self.tool_open,
        }
        self.points.append(point)
        self._render_teach_points()
        self._log("teach", f"Guardado {name}")

    def _render_teach_points(self) -> None:
        self.points_table.setRowCount(len(self.points))
        twin_points: list[tuple[str, list[float]]] = []
        for row, point in enumerate(self.points):
            joints = point["joints_deg"]
            pose = forward(joints)
            values = [point["name"], *[f"{v:.2f}" for v in joints],
                      f"{pose.position_mm[0]:.1f}", f"{pose.position_mm[1]:.1f}",
                      f"{pose.position_mm[2]:.1f}", f"{pose.yaw_deg:.1f}",
                      f"{pose.pitch_deg:.1f}", "Abierto" if point.get("tool_open") else "Cerrado"]
            for col, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.points_table.setItem(row, col, item)
            twin_points.append((point["name"], joints))
        self._refresh_twin(twin_points)

    def _refresh_twin(self, teach_points: list[tuple[str, list[float]]] | None = None) -> None:
        if not hasattr(self, "twin"):
            return
        if teach_points is None:
            teach_points = [(point["name"], point["joints_deg"]) for point in self.points]
        self.twin.set_robot_state(self.current_joints, teach_points)

    def _remove_teach_points(self) -> None:
        rows = sorted({index.row() for index in self.points_table.selectionModel().selectedRows()}, reverse=True)
        for row in rows:
            if 0 <= row < len(self.points):
                self.points.pop(row)
        self._render_teach_points()

    def _clear_teach_points(self) -> None:
        if not self.points:
            return
        choice = QMessageBox.question(self, "Vaciar tabla", "¿Eliminar todos los puntos teach de la tabla?")
        if choice == QMessageBox.StandardButton.Yes:
            self.points.clear()
            self._render_teach_points()

    def _save_teach_file(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Guardar secuencia", "secuencia_robot.json", "JSON (*.json)")
        if not path:
            return
        try:
            save_points(path, self.points)
            self._log("teach", f"Archivo guardado: {Path(path).name}")
        except OSError as exc:
            self._log("error", f"No se pudo guardar: {exc}")

    def _load_teach_file(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Cargar secuencia", "", "JSON (*.json)")
        if not path:
            return
        try:
            self.points = load_points(path)
            self._render_teach_points()
            self._log("teach", f"Cargados {len(self.points)} puntos")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self._log("error", f"Archivo teach inválido: {exc}")

    def _start_sequence(self) -> None:
        if not self.position_valid or self.worker is None:
            self._log("error", "Conecta el ESP32 y espera la posición antes de ejecutar")
            return
        if not self.points:
            self._log("teach", "La tabla no contiene puntos")
            return
        self.sequence_points = copy.deepcopy(self.points)
        self.sequence_index = 0
        self.sequence_repeats_left = self.repeat_spin.value()
        self.sequence_active = True
        self.sequence_command_ids.clear()
        self.run_sequence_button.setEnabled(False)
        self.stop_sequence_button.setEnabled(True)
        self._send_next_sequence_point()

    def _send_next_sequence_point(self) -> None:
        if not self.sequence_active:
            return
        if self.sequence_index >= len(self.sequence_points):
            self.sequence_repeats_left -= 1
            if self.sequence_repeats_left <= 0:
                self._cancel_sequence(send_stop=False)
                self._log("teach", "Secuencia terminada")
                return
            self.sequence_index = 0

        point = self.sequence_points[self.sequence_index]
        tool_id = self._send_command({"cmd": "TOOL", "tool": bool(point.get("tool_open", False))})
        move_id = self._send_command({
            "cmd": "MOVE",
            "axes": point["joints_deg"],
            "feed": self.feed_spin.value(),
        })
        if tool_id is None or move_id is None:
            self._cancel_sequence(send_stop=False)
            return
        self.sequence_command_ids = {tool_id, move_id}
        self.sequence_move_id = move_id
        self._log("teach", f"Ejecutando {point['name']} · repetición "
                  f"{self.repeat_spin.value() - self.sequence_repeats_left + 1}/" f"{self.repeat_spin.value()}")

    def _cancel_sequence(self, *, send_stop: bool = True) -> None:
        was_active = self.sequence_active
        self.sequence_active = False
        self.sequence_move_id = None
        self.sequence_command_ids.clear()
        if hasattr(self, "run_sequence_button"):
            connected = self.worker is not None and self.worker.isRunning()
            self.run_sequence_button.setEnabled(connected)
            self.stop_sequence_button.setEnabled(False)
        if send_stop and was_active:
            self._send_command({"cmd": "STOP"})

    def _log(self, category: str, message: str) -> None:
        from datetime import datetime

        row = self.log.rowCount()
        self.log.insertRow(row)
        self.log.setItem(row, 0, QTableWidgetItem(datetime.now().strftime("%H:%M:%S")))
        self.log.setItem(row, 1, QTableWidgetItem(f"[{category}] {message}"))
        self.log.scrollToBottom()
        while self.log.rowCount() > 100:
            self.log.removeRow(0)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt event naming
        if self.worker is not None:
            worker = self.worker
            self._cancel_sequence(send_stop=False)
            self._send_command({"cmd": "STOP"})
            worker.request_stop()
            if not worker.wait(1200):
                self._closing_after_disconnect = True
                event.ignore()
                return
            self.worker = None
            worker.deleteLater()
        super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    window = RobotHMI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
