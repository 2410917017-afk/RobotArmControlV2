"""Modelo DH del brazo y ajustes aún por calibrar.

Se usa DH estándar en milímetros: Rz(theta) Tz(d) Tx(a) Rx(alpha).
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class DHLink:
    a_mm: float
    d_mm: float
    alpha_deg: float
    theta_offset_deg: float = 0.0


JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5")
JOINT_LIMITS_DEG = ((-160, 160),(-140, 220),(-130, 360),(-180, 180),(-100, 100))


# En DH estándar, theta es la variable articular y a corresponde a r.
DH_LINKS = (
    DHLink(a_mm=25.0, d_mm=145.0, alpha_deg=90.0),
    DHLink(a_mm=200.0, d_mm=0.0, alpha_deg=0.0),
    DHLink(a_mm=0.0, d_mm=0.0, alpha_deg=90.0),
    DHLink(a_mm=0.0, d_mm=177.0, alpha_deg=-90.0),
    DHLink(a_mm=0.0, d_mm=0.0, alpha_deg=90.0),
)

# Offset mecánico entre el cero lógico de arranque del ESP32 y DH theta=0.
# Ajustar cuando se conozca la pose física usada al encender.
KINEMATIC_ZERO_OFFSETS_DEG = (0.0, 90.0, 0.0, 0.0, 0.0)
TCP_OFFSET_MM = (0.0, 0.0, 0.0)

DEFAULT_FEED_STEPS_S = 1000
MAX_FEED_STEPS_S = 5000
DEFAULT_JOINT_JOG_DEG = 1.0
DEFAULT_CARTESIAN_JOG_MM = 5.0
DEFAULT_ANGULAR_JOG_DEG = 2.0
