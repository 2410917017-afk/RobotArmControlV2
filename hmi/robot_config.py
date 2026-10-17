"""Modelo DH del brazo y ajustes aún por calibrar.

Se usa DH estándar en milímetros: Rz(theta) Tz(d) Tx(a) Rx(alpha).
El TCP, los offsets mecánicos y los límites siguen siendo provisionales.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class DHLink:
    a_mm: float
    d_mm: float
    alpha_deg: float
    theta_offset_deg: float = 0.0


JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5")
JOINT_LIMITS_DEG = ((-10.0, 10.0),) * 5

# Las primeras cuatro filas provienen del código compartido. La quinta usa los
# datos añadidos por el usuario para la junta faltante: a/r=0, d=0, alpha=-90°.
# En DH estándar, theta es la variable articular y a corresponde a r.
DH_LINKS = (
    DHLink(a_mm=0.0, d_mm=145.0, alpha_deg=-90.0),
    DHLink(a_mm=200.0, d_mm=0.0, alpha_deg=0.0),
    DHLink(a_mm=0.0, d_mm=0.0, alpha_deg=-90.0),
    DHLink(a_mm=0.0, d_mm=177.0, alpha_deg=90.0),
    DHLink(a_mm=0.0, d_mm=0.0, alpha_deg=-90.0),
)

# Offset mecánico entre el cero lógico de arranque del ESP32 y DH theta=0.
# Ajustar cuando se conozca la pose física usada al encender.
KINEMATIC_ZERO_OFFSETS_DEG = (0.0, 0.0, 0.0, 0.0, 0.0)
TCP_OFFSET_MM = (0.0, 0.0, 0.0)

DEFAULT_FEED_STEPS_S = 300
MAX_FEED_STEPS_S = 800
DEFAULT_JOINT_JOG_DEG = 1.0
DEFAULT_CARTESIAN_JOG_MM = 5.0
DEFAULT_ANGULAR_JOG_DEG = 2.0
