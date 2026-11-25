"""Persistencia local de puntos teach pendant en JSON."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from hmi.robot_config import JOINT_LIMITS_DEG


def save_points(path: str | Path, points: list[dict[str, Any]]) -> None:
    payload = {
        "format": "robot5dof-teach-v1",
        "points": points,
    }
    Path(path).write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def load_points(path: str | Path) -> list[dict[str, Any]]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if (not isinstance(payload, dict)
            or payload.get("format") != "robot5dof-teach-v1"
            or not isinstance(payload.get("points"), list)):
        raise ValueError("El archivo no es una secuencia teach de este HMI")

    checked: list[dict[str, Any]] = []
    for index, point in enumerate(payload["points"], start=1):
        joints = point.get("joints_deg") if isinstance(point, dict) else None
        if not isinstance(joints, list) or len(joints) != 5:
            raise ValueError(f"El punto {index} no contiene cinco ángulos")
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in joints):
            raise ValueError(f"El punto {index} tiene ángulos no numéricos")
        if any(not math.isfinite(float(value)) for value in joints):
            raise ValueError(f"El punto {index} contiene ángulos no finitos")
        if any(not low <= float(value) <= high
               for value, (low, high) in zip(joints, JOINT_LIMITS_DEG)):
            raise ValueError(f"El punto {index} excede los límites articulares configurados")
        if "tool_open" in point and not isinstance(point["tool_open"], bool):
            raise ValueError(f"El punto {index} tiene un estado de gripper inválido")
        checked.append({
            "name": str(point.get("name", f"P{index}")),
            "joints_deg": [float(value) for value in joints],
            "tool_open": bool(point.get("tool_open", False)),
        })
    return checked
