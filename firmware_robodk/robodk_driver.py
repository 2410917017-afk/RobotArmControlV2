"""Driver online RoboDK -> USB serial JSON del ESP32 (cinco ejes)."""

from __future__ import annotations

import json
import sys
import time

import serial


class Esp32Bridge:
    def __init__(self) -> None:
        self.serial: serial.Serial | None = None
        self.command_id = 1
        self.joints = [0.0] * 5
        self.feed = 800

    def connect(self, port_name: str, baud: int = 115200) -> None:
        self.disconnect()
        # Opening USB serial can reset many ESP32 dev boards through DTR/RTS.
        self.serial = serial.Serial(
            port=port_name,
            baudrate=baud,
            timeout=0.2,
            write_timeout=2,
            dsrdtr=False,
            rtscts=False,
        )
        deadline = time.monotonic() + 15
        status_id = self._send({"cmd": "STATUS"})
        next_status = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            try:
                event = self._read_event(timeout=min(0.5, deadline - time.monotonic()))
            except (TimeoutError, ValueError):
                if time.monotonic() >= next_status:
                    status_id = self._send({"cmd": "STATUS"})
                    next_status = time.monotonic() + 1.0
                continue  # Skip boot text and wait for a valid JSON ready/status event.
            if event.get("event") == "ready" or (
                event.get("event") == "status" and event.get("id") == status_id
            ):
                self._status("ESP32 USB serial listo")
                return
        self.disconnect()
        raise TimeoutError("No llegó el evento JSON ready del ESP32 por USB")

    def disconnect(self) -> None:
        if self.serial is not None:
            try:
                self.serial.close()
            except serial.SerialException:
                pass
        self.serial = None

    def _send(self, command: dict) -> int:
        if self.serial is None or not self.serial.is_open:
            raise ConnectionError("No conectado al ESP32 por USB")
        command_id = self.command_id
        self.command_id += 1
        command["id"] = command_id
        line = json.dumps(command, separators=(",", ":")) + "\n"
        self.serial.write(line.encode("utf-8"))
        self.serial.flush()
        return command_id

    def _read_event(self, timeout: float | None = None) -> dict:
        if self.serial is None or not self.serial.is_open:
            raise ConnectionError("No conectado al ESP32 por USB")
        previous_timeout = self.serial.timeout
        deadline = None if timeout is None else time.monotonic() + max(0.0, timeout)
        try:
            while deadline is None or time.monotonic() < deadline:
                self.serial.timeout = 0.2 if deadline is None else min(0.2, max(0.01, deadline - time.monotonic()))
                raw = self.serial.readline()
                if not raw:
                    continue
                try:
                    event = json.loads(raw.decode("utf-8", errors="replace").strip())
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue  # Ignore boot diagnostics that are not JSON.
                if not isinstance(event, dict):
                    continue
                self._update_position(event)
                if event.get("event") == "error":
                    raise RuntimeError(f"ESP32 {event.get('code', 'error')}")
                return event
        finally:
            self.serial.timeout = previous_timeout
        raise TimeoutError("Tiempo agotado esperando respuesta JSON del ESP32")

    def _update_position(self, event: dict) -> None:
        axes = event.get("axes")
        if isinstance(axes, list) and len(axes) == 5:
            self.joints = [float(axis["joint_deg_est"]) for axis in axes]

    def _wait_for(self, command_id: int, wanted: set[str]) -> dict:
        while True:
            event = self._read_event()
            if event.get("id") == command_id and event.get("event") in wanted:
                return event

    def _status(self, message: str) -> None:
        print(f"SMS:{message}", flush=True)

    def get_joints(self) -> list[float]:
        command_id = self._send({"cmd": "STATUS"})
        self._wait_for(command_id, {"status"})
        return self.joints[:]

    def move_joints(self, joints: list[float]) -> None:
        if len(joints) != 5:
            raise ValueError("RoboDK debe configurar exactamente 5 ejes")
        command_id = self._send({"cmd": "MOVE", "axes": joints, "feed": self.feed})
        self._status("Working...")
        self._wait_for(command_id, {"complete"})
        self._status("Ready")

    def stop(self) -> None:
        command_id = self._send({"cmd": "STOP"})
        self._status("Working...")
        self._wait_for(command_id, {"complete"})
        self._status("Ready")

    def set_gripper(self, opened: bool) -> None:
        command_id = self._send({"cmd": "TOOL", "tool": opened})
        self._wait_for(command_id, {"tool"})


ROBOT = Esp32Bridge()


def run_command(line: str) -> bool:
    parts = line.strip().split()
    if not parts:
        return True
    command = parts[0].upper()
    try:
        if command == "CONNECT":
            if len(parts) < 2:
                raise ValueError("Uso: CONNECT COMx 115200 5")
            baud = int(parts[2]) if len(parts) >= 3 else 115200
            if len(parts) >= 4 and int(parts[3]) != 5:
                raise ValueError("El brazo requiere DOF=5")
            ROBOT.connect(parts[1], baud)
        elif command == "DISCONNECT":
            ROBOT.disconnect()
            print("SMS:Disconnected", flush=True)
        elif command == "QUIT":
            ROBOT.disconnect()
            return False
        elif command == "PAUSE":
            time.sleep(max(0.0, float(parts[1]) / 1000.0))
        elif command == "STOP":
            ROBOT.stop()
        elif command == "CJNT":
            joints = ROBOT.get_joints()
            print("JNTS " + ", ".join(f"{joint:.6f}" for joint in joints), flush=True)
        elif command == "MOVJ":
            if len(parts) < 6:
                raise ValueError("MOVJ requiere cinco ángulos articulares")
            ROBOT.move_joints([float(value) for value in parts[1:6]])
        elif command == "SETDO":
            if len(parts) < 3:
                raise ValueError("SETDO requiere nombre y valor")
            ROBOT.set_gripper(float(parts[2]) > 0.0)
        elif command in {"MOVL", "MOVC", "MOVLSEARCH", "SPEED", "SETTOOL", "SETAO", "WAITDI", "RUNPROG"}:
            raise NotImplementedError(f"{command} todavía no está implementado")
        else:
            raise ValueError(f"Comando no soportado: {command}")
        return True
    except Exception as exc:  # Don't put tracebacks on RoboDK's stdout protocol.
        print(f"SMS:Error - {exc}", flush=True)
        print(f"RE:ERROR {exc}", flush=True)
        return True


def main() -> None:
    for line in sys.stdin:
        if not run_command(line):
            break
    ROBOT.disconnect()


if __name__ == "__main__":
    main()
