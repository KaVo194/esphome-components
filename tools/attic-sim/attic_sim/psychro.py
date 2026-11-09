"""Psychrometrische Grundfunktionen (Wasserdampf in Luft)."""
import numpy as np

RV = 461.5          # spez. Gaskonstante Wasserdampf [J/(kg K)]
RHO_C_AIR = 1.2 * 1005.0  # volumetrische Waermekapazitaet Luft [J/(m3 K)]
# Wasserdampf-Diffusionsleitkoeffizient ruhender Luft [kg/(m s Pa)].
# Groessenordnung nach Literatur ~1.9e-10 .. 2.0e-10 (temperatur-/druckabhaengig).
DELTA_AIR = 2.0e-10


def psat(T):
    """Saettigungsdampfdruck [Pa] (Magnus-Formel wie in DIN 4108-3), T in degC."""
    T = np.asarray(T, dtype=float)
    over_water = 610.5 * np.exp(17.269 * T / (237.3 + T))
    over_ice = 610.5 * np.exp(21.875 * T / (265.5 + T))
    return np.where(T >= 0.0, over_water, over_ice)


def dewpoint(p):
    """Taupunkt [degC] aus Dampfdruck [Pa] (ueber Wasser)."""
    x = np.log(np.maximum(np.asarray(p, dtype=float), 1e-3) / 610.5)
    return 237.3 * x / (17.269 - x)


def p_from_rh(T, rh):
    """Dampfdruck [Pa] aus Temperatur [degC] und rel. Feuchte [0..1]."""
    return np.asarray(rh, dtype=float) * psat(T)


def p_from_dewpoint(Td):
    return psat(Td)
