"""Cinemática directa e inversa numérica para una cadena DH de 5 GDL."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from robot_config import (
    DH_LINKS,
    JOINT_LIMITS_DEG,
    KINEMATIC_ZERO_OFFSETS_DEG,
    TCP_OFFSET_MM,
)


@dataclass(frozen=True)
class Pose:
    position_mm: tuple[float, float, float]
    yaw_deg: float
    pitch_deg: float
    tool_axis: tuple[float, float, float]
    link_points_mm: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True)
class IKResult:
    success: bool
    joints_deg: tuple[float, ...]
    position_error_mm: float
    orientation_error_deg: float
    iterations: int


def _identity() -> list[list[float]]:
    return [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def _matmul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def _dh_matrix(a_mm: float, d_mm: float, alpha_deg: float, theta_deg: float) -> list[list[float]]:
    alpha = math.radians(alpha_deg)
    theta = math.radians(theta_deg)
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st * ca, st * sa, a_mm * ct],
        [st, ct * ca, -ct * sa, a_mm * st],
        [0.0, sa, ca, d_mm],
        [0.0, 0.0, 0.0, 1.0],
    ]


def forward(joints_deg: Sequence[float]) -> Pose:
    if len(joints_deg) != len(DH_LINKS):
        raise ValueError("Se requieren cinco ángulos articulares")

    transform = _identity()
    points: list[tuple[float, float, float]] = [(0.0, 0.0, 0.0)]
    for index, link in enumerate(DH_LINKS):
        theta = joints_deg[index] + KINEMATIC_ZERO_OFFSETS_DEG[index] + link.theta_offset_deg
        transform = _matmul(
            transform,
            _dh_matrix(link.a_mm, link.d_mm, link.alpha_deg, theta),
        )
        points.append((transform[0][3], transform[1][3], transform[2][3]))

    tcp = _identity()
    tcp[0][3], tcp[1][3], tcp[2][3] = TCP_OFFSET_MM
    transform = _matmul(transform, tcp)
    position = (transform[0][3], transform[1][3], transform[2][3])
    axis = (transform[0][2], transform[1][2], transform[2][2])
    horizontal = math.hypot(axis[0], axis[1])
    yaw = math.degrees(math.atan2(axis[1], axis[0])) if horizontal > 1e-8 else 0.0
    pitch = math.degrees(math.atan2(axis[2], horizontal))
    return Pose(position, yaw, pitch, axis, tuple(points + [position]))


def direction_from_yaw_pitch(yaw_deg: float, pitch_deg: float) -> tuple[float, float, float]:
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    return (
        math.cos(pitch) * math.cos(yaw),
        math.cos(pitch) * math.sin(yaw),
        math.sin(pitch),
    )


def _solve_linear(matrix: list[list[float]], rhs: list[float]) -> list[float] | None:
    n = len(rhs)
    augmented = [matrix[row][:] + [rhs[row]] for row in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(augmented[row][col]))
        if abs(augmented[pivot][col]) < 1e-12:
            return None
        augmented[col], augmented[pivot] = augmented[pivot], augmented[col]
        divisor = augmented[col][col]
        for k in range(col, n + 1):
            augmented[col][k] /= divisor
        for row in range(n):
            if row == col:
                continue
            factor = augmented[row][col]
            for k in range(col, n + 1):
                augmented[row][k] -= factor * augmented[col][k]
    return [augmented[row][n] for row in range(n)]


def inverse(
    target_position_mm: Sequence[float],
    yaw_deg: float,
    pitch_deg: float,
    seed_deg: Sequence[float],
    *,
    max_iterations: int = 100,
    position_tolerance_mm: float = 2.0,
    orientation_tolerance_deg: float = 2.0,
) -> IKResult:
    """Damped least-squares IK for XYZ + tool-axis yaw/pitch (no roll target)."""
    if len(target_position_mm) != 3 or len(seed_deg) != len(DH_LINKS):
        raise ValueError("La pose objetivo requiere XYZ y cinco ángulos semilla")
    if not all(math.isfinite(float(v)) for v in (*target_position_mm, yaw_deg, pitch_deg, *seed_deg)):
        raise ValueError("La pose contiene valores no finitos")

    target_pos = tuple(float(v) for v in target_position_mm)
    target_axis = direction_from_yaw_pitch(yaw_deg, pitch_deg)
    q = [max(lo, min(hi, float(seed_deg[i]))) for i, (lo, hi) in enumerate(JOINT_LIMITS_DEG)]
    orientation_scale_mm = 120.0
    damping = 0.15

    def residual(joints: Sequence[float]) -> tuple[list[float], float, float]:
        pose = forward(joints)
        position_delta = [target_pos[i] - pose.position_mm[i] for i in range(3)]
        dir_delta = [target_axis[i] - pose.tool_axis[i] for i in range(3)]
        position_error = math.sqrt(sum(v * v for v in position_delta))
        dot = max(-1.0, min(1.0, sum(target_axis[i] * pose.tool_axis[i] for i in range(3))))
        orientation_error = math.degrees(math.acos(dot))
        values = position_delta + [orientation_scale_mm * v for v in dir_delta]
        return values, position_error, orientation_error

    position_error = math.inf
    orientation_error = math.inf
    for iteration in range(1, max_iterations + 1):
        error, position_error, orientation_error = residual(q)
        if position_error <= position_tolerance_mm and orientation_error <= orientation_tolerance_deg:
            return IKResult(True, tuple(q), position_error, orientation_error, iteration - 1)

        epsilon_deg = 0.1
        jacobian = [[0.0 for _ in range(len(DH_LINKS))] for _ in range(6)]
        for col in range(len(DH_LINKS)):
            perturbed = q[:]
            perturbed[col] = min(JOINT_LIMITS_DEG[col][1], perturbed[col] + epsilon_deg)
            delta = perturbed[col] - q[col]
            if delta <= 1e-9:
                perturbed[col] = max(JOINT_LIMITS_DEG[col][0], q[col] - epsilon_deg)
                delta = perturbed[col] - q[col]
            if abs(delta) <= 1e-9:
                continue
            shifted_error, _, _ = residual(perturbed)
            for row in range(6):
                jacobian[row][col] = (error[row] - shifted_error[row]) / delta

        normal = [[0.0 for _ in range(5)] for _ in range(5)]
        rhs = [0.0 for _ in range(5)]
        for row in range(5):
            rhs[row] = sum(jacobian[k][row] * error[k] for k in range(6))
            for col in range(5):
                normal[row][col] = sum(jacobian[k][row] * jacobian[k][col] for k in range(6))
            normal[row][row] += damping * damping

        delta_q = _solve_linear(normal, rhs)
        if delta_q is None:
            break
        for i in range(5):
            step = max(-5.0, min(5.0, delta_q[i]))
            lo, hi = JOINT_LIMITS_DEG[i]
            q[i] = max(lo, min(hi, q[i] + step))

    _, position_error, orientation_error = residual(q)
    return IKResult(False, tuple(q), position_error, orientation_error, max_iterations)


def circular_path(
    start_joints_deg: Sequence[float],
    center_mm: Sequence[float],
    normal: Sequence[float],
    sweep_deg: float,
    samples: int,
    *,
    yaw_deg: float | None = None,
    pitch_deg: float | None = None,
    ) -> tuple[IKResult, ...]:
    
    """Calcula puntos IK sobre un arco circular desde la pose articular inicial.

    ``center_mm`` y ``normal`` definen el plano/círculo. El punto inicial se
    obtiene con ``forward(start_joints_deg)`` y debe estar en ese plano. El
    barrido es firmado: positivo sigue la regla de la mano derecha respecto a
    ``normal``; negativo recorre el arco en sentido contrario. ``samples`` es
    el número de puntos objetivo, sin contar la pose inicial y contando el
    extremo final. La orientación de herramienta se mantiene fija en yaw/pitch
    de la pose inicial, salvo que se proporcionen ambos argumentos.

    Resuelve la trayectoria completa antes de devolverla para que el llamador
    pueda evitar enviar una secuencia parcial al robot si algún punto no es
    alcanzable. Los IKResults se entregan en orden desde el primer punto hasta
    el final; cada solución se usa como semilla de la siguiente.
    """
    
    if len(start_joints_deg) != len(DH_LINKS):
        raise ValueError("Se requieren cinco ángulos articulares iniciales")
    if len(center_mm) != 3 or len(normal) != 3:
        raise ValueError("El centro y la normal deben tener tres componentes")
    if not isinstance(samples, int) or samples < 1:
        raise ValueError("samples debe ser un entero mayor o igual que 1")
    if not math.isfinite(float(sweep_deg)) or abs(float(sweep_deg)) < 1e-9:
        raise ValueError("sweep_deg debe ser finito y distinto de cero")
    values = [float(v) for v in (*start_joints_deg, *center_mm, *normal)]
    if not all(math.isfinite(v) for v in values):
        raise ValueError("La trayectoria contiene valores no finitos")
    if (yaw_deg is None) != (pitch_deg is None):
        raise ValueError("Proporciona yaw_deg y pitch_deg juntos, o ninguno")

    start_pose = forward(start_joints_deg)
    center = tuple(float(v) for v in center_mm)
    n = tuple(float(v) for v in normal)
    n_length = math.sqrt(sum(v * v for v in n))
    if n_length <= 1e-12:
        raise ValueError("La normal del plano no puede ser cero")
    n = tuple(v / n_length for v in n)

    radius = tuple(start_pose.position_mm[i] - center[i] for i in range(3))
    radius_length = math.sqrt(sum(v * v for v in radius))
    if radius_length <= 1e-9:
        raise ValueError("El punto inicial coincide con el centro del círculo")
    plane_error = abs(sum(radius[i] * n[i] for i in range(3)))
    if plane_error > max(1.0, radius_length * 1e-3):
        raise ValueError("El punto inicial no pertenece al plano definido por center_mm y normal")

    yaw = start_pose.yaw_deg if yaw_deg is None else float(yaw_deg)
    pitch = start_pose.pitch_deg if pitch_deg is None else float(pitch_deg)
    if not math.isfinite(yaw) or not math.isfinite(pitch):
        raise ValueError("La orientación objetivo debe ser finita")

    results: list[IKResult] = []
    seed = tuple(float(v) for v in start_joints_deg)
    sweep = math.radians(float(sweep_deg))
    for index in range(1, samples + 1):
        angle = sweep * index / samples
        cosine, sine = math.cos(angle), math.sin(angle)
        cross = (
            n[1] * radius[2] - n[2] * radius[1],
            n[2] * radius[0] - n[0] * radius[2],
            n[0] * radius[1] - n[1] * radius[0],
        )
        projection = sum(n[i] * radius[i] for i in range(3))
        rotated = tuple(
            radius[i] * cosine + cross[i] * sine + n[i] * projection * (1.0 - cosine)
            for i in range(3)
        )
        target = tuple(center[i] + rotated[i] for i in range(3))
        result = inverse(target, yaw, pitch, seed)
        if not result.success:
            raise ValueError(
                f"El punto {index}/{samples} del arco no converge: "
                f"error XYZ={result.position_error_mm:.2f} mm, "
                f"orientación={result.orientation_error_deg:.2f}°"
            )
        results.append(result)
        seed = result.joints_deg

    return tuple(results)


