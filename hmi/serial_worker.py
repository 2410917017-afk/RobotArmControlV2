"""Worker serie no bloqueante para JSON delimitado por líneas."""

from __future__ import annotations

import json
import queue
import threading

import serial
from PyQt6.QtCore import QThread, pyqtSignal


class SerialWorker(QThread):
    connected = pyqtSignal(str)
    disconnected = pyqtSignal()
    message_received = pyqtSignal(dict)
    error_received = pyqtSignal(str)

    def __init__(self, port: str, baud: int = 115200) -> None:
        super().__init__()
        self.port = port
        self.baud = baud
        self._outgoing: queue.Queue[dict] = queue.Queue()
        self._stop_event = threading.Event()

    def send_command(self, command: dict) -> None:
        if not self._stop_event.is_set():
            self._outgoing.put(command)

    def request_stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        connection: serial.Serial | None = None
        try:
            connection = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.5)
            self.connected.emit(self.port)
            while not self._stop_event.is_set():
                self._write_pending(connection)
                raw = connection.readline()
                if not raw:
                    continue
                try:
                    decoded = raw.decode("utf-8", errors="replace").strip()
                    message = json.loads(decoded)
                    if isinstance(message, dict):
                        self.message_received.emit(message)
                    else:
                        self.error_received.emit(f"Respuesta JSON no es objeto: {decoded}")
                except json.JSONDecodeError:
                    self.error_received.emit(f"Respuesta JSON inválida: {raw!r}")
        except (serial.SerialException, OSError) as exc:
            if not self._stop_event.is_set():
                self.error_received.emit(f"No se pudo abrir/usar {self.port}: {exc}")
        finally:
            if connection is not None and connection.is_open:
                self._write_pending(connection, force=True)
                connection.close()
            self.disconnected.emit()

    def _write_pending(self, connection: serial.Serial, *, force: bool = False) -> None:
        while force or not self._stop_event.is_set():
            try:
                command = self._outgoing.get_nowait()
            except queue.Empty:
                return
            try:
                line = json.dumps(command, separators=(",", ":"), allow_nan=False) + "\n"
                connection.write(line.encode("utf-8"))
            except (serial.SerialException, OSError, ValueError) as exc:
                self.error_received.emit(f"No se pudo enviar el comando: {exc}")
                self._stop_event.set()
                return
