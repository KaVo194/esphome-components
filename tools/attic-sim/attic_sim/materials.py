"""Baustoffkennwerte und Sorptionsisothermen.

ALLE Zahlenwerte sind typische Tabellen-/Literaturwerte (Groessenordnung) und
KEINE geprueften Herstellerangaben. Fuer belastbare Ergebnisse bitte mit
Datenblaettern bzw. DIN EN ISO 10456 / DIN 4108-4 abgleichen.

Sorptionsisotherme nach dem Naeherungsansatz von Kuenzel:
    w(phi) = wf * (b - 1) * phi / (b - phi)
mit b so gewaehlt, dass w(0.8) = w80. Zusaetzlich wird der Wasserdampf in der
Porenluft als Speicher beruecksichtigt (wichtig bei Mineralwolle).
"""
from dataclasses import dataclass

import numpy as np

from .psychro import RV, psat


@dataclass(frozen=True)
class Material:
    name: str
    rho: float        # Rohdichte [kg/m3]
    c: float          # spez. Waermekapazitaet [J/(kg K)]
    lam: float        # Waermeleitfaehigkeit [W/(m K)]
    mu: float         # Wasserdampf-Diffusionswiderstandszahl [-]
    porosity: float   # Porositaet [-] (fuer Dampfspeicher in Porenluft)
    w80: float        # Wassergehalt bei 80 % r.F. [kg/m3]
    wf: float         # Wassergehalt bei 100 % r.F. (freie Saettigung) [kg/m3]

    def __post_init__(self):
        if not (0 < self.w80 < 0.8 * self.wf):
            raise ValueError(f"{self.name}: es muss 0 < w80 < 0.8*wf gelten")

    @property
    def b(self):
        return 0.8 * (self.w80 - self.wf) / (self.w80 - 0.8 * self.wf)

    # --- Isotherme inkl. Dampf in Porenluft -----------------------------
    def w(self, phi, T):
        phi = np.asarray(phi, dtype=float)
        b = self.b
        hyg = self.wf * (b - 1.0) * phi / (b - phi)
        vap = self.porosity * phi * psat(T) / (RV * (np.asarray(T) + 273.15))
        return hyg + vap

    def dw_dphi(self, phi, T):
        phi = np.asarray(phi, dtype=float)
        b = self.b
        hyg = self.wf * (b - 1.0) * b / (b - phi) ** 2
        vap = self.porosity * psat(T) / (RV * (np.asarray(T) + 273.15))
        return hyg + vap

    def phi_from_w(self, W, T):
        """Rel. Feuchte und freies Tauwasser [kg/m3] aus Gesamtwassergehalt."""
        W = np.asarray(W, dtype=float)
        T = np.asarray(T, dtype=float)
        w_sat = self.w(1.0, T)
        cond = np.maximum(W - w_sat, 0.0)
        Wc = np.clip(W, 0.0, w_sat)
        b = self.b
        # Startwert: Kuenzel-Inverse ohne Dampfanteil
        phi = np.clip(b * Wc / (self.wf * (b - 1.0) + Wc), 0.0, 1.0)
        for _ in range(8):  # Newton
            f = self.w(phi, T) - Wc
            phi = np.clip(phi - f / self.dw_dphi(phi, T), 0.0, 1.0)
        return phi, cond


# Typische Kennwerte (bitte pruefen!)
LIBRARY = {
    # Stahlbeton, Tabellenwerte Groessenordnung DIN EN ISO 10456 (mu trocken 130 / feucht 80)
    "beton": Material("Stahlbeton", rho=2300, c=1000, lam=2.3, mu=130,
                      porosity=0.15, w80=85.0, wf=150.0),
    # Variante: junger, nasser Beton. Kleinere Diffusionswiderstandszahl als grober Ersatz
    # fuer den im Modell fehlenden Kapillartransport (Sensitivitaet, kein Messwert).
    "beton_nass": Material("Stahlbeton", rho=2300, c=1000, lam=2.3, mu=30,
                           porosity=0.15, w80=85.0, wf=150.0),
    # Mineralwolle: praktisch nicht hygroskopisch
    "mineralwolle": Material("Mineralwolle", rho=30, c=840, lam=0.035, mu=1.0,
                             porosity=0.95, w80=0.05, wf=0.1),
    # Zellulose-Einblas-/Schuettdaemmung, offen auf Decke (Dichte ~35-50 kg/m3).
    # Ausgleichsfeuchte ca. 12-14 M-% bei 80 % r.F. -> GROBE ANNAHME, bitte Datenblatt pruefen.
    "zellulose": Material("Zellulose", rho=45, c=2000, lam=0.040, mu=1.5,
                          porosity=0.95, w80=5.9, wf=11.0),
    "eps": Material("EPS", rho=20, c=1500, lam=0.035, mu=60,
                    porosity=0.95, w80=0.01, wf=0.02),
    # Nadelholz ~16 M-% bei 80 % r.F., ~28 M-% Fasersaettigung
    "holz": Material("Nadelholz", rho=450, c=1600, lam=0.13, mu=50,
                     porosity=0.6, w80=72.0, wf=126.0),
    "osb": Material("OSB", rho=600, c=1700, lam=0.13, mu=200,
                    porosity=0.5, w80=70.0, wf=150.0),
}


def get(name):
    try:
        return LIBRARY[name]
    except KeyError:
        raise KeyError(f"Unbekanntes Material '{name}'. Verfuegbar: {sorted(LIBRARY)}")
