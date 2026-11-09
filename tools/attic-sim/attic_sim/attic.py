"""Vereinfachtes Zonenmodell des geschlossenen, ungedaemmten Dachraums.

Knoten (Waerme): Dachraumluft, Dachschalung+Sparren (Holz), Kniestock/Giebel
(Beton/Mauerwerk innen vor EPS).
Knoten (Feuchte): Dachraumluft, Sorptionsschicht Holz, Sorptionsschicht Waende
(Ansatz "effektive Eindringtiefe" / EMPD).

Die Decke wird als Leitwert zu ihrem obersten Rechenknoten angekoppelt
(Werte des vorherigen Zeitschritts -> semi-implizit).

Randbedingung Dach: Unter dem Gruendach liegt eine hinterlueftete Ebene. Die
untere Schalung sieht deshalb Aussenluft (Konvektion) und die Unterseite der
oberen Schalung (Strahlung). Deren Temperatur wird als gleitendes 24-h-Mittel
der Aussentemperatur + Zuschlag dT_upper angenaehert (Gruendach daempft stark).
"""
import numpy as np

from . import materials
from .psychro import DELTA_AIR, RHO_C_AIR, RV, psat


class Attic:
    def __init__(self, p, T0, phi0):
        self.A_floor = p["floor_area"]
        self.A_roof = p["floor_area"] / np.cos(np.radians(p["roof_pitch_deg"]))
        self.A_wood = self.A_roof * p["wood_area_factor"]
        self.A_wall = p["wall_area"]
        self.V = p["floor_area"] * p["mean_height"]
        self.n = p["air_change_per_h"]
        self.h_in = p["h_inside"]
        self.h_cav_conv = p["h_cavity_conv"]
        self.h_cav_rad = p["h_cavity_rad"]
        self.beta = p["beta_inside"]
        self.U_wall_out = p["U_wall_outside"]
        self.sd_roof_to_cavity = p["sd_roof_to_cavity"]

        self.wood = materials.get(p["wood_material"])
        self.wall = materials.get(p["wall_material"])
        self.d_wood = p["wood_sorption_depth"]
        self.d_wall = p["wall_sorption_depth"]

        # Waermekapazitaeten [J/K]
        self.C_air = RHO_C_AIR * self.V
        self.C_roof = self.wood.rho * self.wood.c * p["wood_volume_per_roof_m2"] * self.A_roof
        self.C_wall = self.wall.rho * self.wall.c * p["wall_thickness"] * self.A_wall
        # Leitwert Holz -> Hinterlueftungsebene (halbe Schalung + Uebergang)
        R_board = 0.5 * p["board_thickness"] / self.wood.lam
        self.H_ext = self.A_roof / (R_board + 1.0 / (self.h_cav_conv + self.h_cav_rad))

        self.Ta = self.Tr = self.Tw = float(T0)
        self.pa = phi0 * float(psat(T0))
        self.Wr = float(self.wood.w(phi0, T0))
        self.Ww = float(self.wall.w(phi0, T0))
        self.fog_steps = 0

    def phi_wood(self):
        return float(self.wood.phi_from_w(self.Wr, self.Tr)[0])

    def step(self, dt, T_out, p_out, T_upper, ceil_T_top, ceil_p_top, G_ceil, K_ceil):
        A = self.A_floor
        # ---------------- Waerme (3x3 implizit) ----------------
        H_r = self.h_in * self.A_wood
        H_w = self.h_in * self.A_wall
        H_v = RHO_C_AIR * self.n * self.V / 3600.0
        H_c = G_ceil * A
        H_wo = self.U_wall_out * self.A_wall
        T_cav = (self.h_cav_conv * T_out + self.h_cav_rad * T_upper) / (
            self.h_cav_conv + self.h_cav_rad)
        ca, cr, cw = self.C_air / dt, self.C_roof / dt, self.C_wall / dt
        M = np.array([
            [ca + H_r + H_w + H_v + H_c, -H_r, -H_w],
            [-H_r, cr + H_r + self.H_ext, 0.0],
            [-H_w, 0.0, cw + H_w + H_wo],
        ])
        b = np.array([
            ca * self.Ta + H_v * T_out + H_c * ceil_T_top,
            cr * self.Tr + self.H_ext * T_cav,
            cw * self.Tw + H_wo * T_out,
        ])
        self.Ta, self.Tr, self.Tw = np.linalg.solve(M, b)

        # ---------------- Feuchte (3x3 implizit) ----------------
        TaK = self.Ta + 273.15
        phr = float(self.wood.phi_from_w(self.Wr, self.Tr)[0])
        phw = float(self.wall.phi_from_w(self.Ww, self.Tw)[0])
        psr, psw = float(psat(self.Tr)), float(psat(self.Tw))
        pr_old, pw_old = phr * psr, phw * psw
        Cpa = self.V / (RV * TaK) / dt
        Cpr = float(self.wood.dw_dphi(phr, self.Tr)) / psr * self.d_wood * self.A_wood / dt
        Cpw = float(self.wall.dw_dphi(phw, self.Tw)) / psw * self.d_wall * self.A_wall / dt
        Kn = self.n * self.V / 3600.0 / (RV * TaK)
        Krb = self.A_wood / (1.0 / self.beta + 0.5 * self.d_wood * self.wood.mu / DELTA_AIR)
        Krc = self.A_roof * DELTA_AIR / self.sd_roof_to_cavity
        Kwb = self.A_wall / (1.0 / self.beta + 0.5 * self.d_wall * self.wall.mu / DELTA_AIR)
        Kc = K_ceil * A
        M = np.array([
            [Cpa + Kn + Krb + Kwb + Kc, -Krb, -Kwb],
            [-Krb, Cpr + Krb + Krc, 0.0],
            [-Kwb, 0.0, Cpw + Kwb],
        ])
        b = np.array([
            Cpa * self.pa + Kn * p_out + Kc * ceil_p_top,
            Cpr * pr_old + Krc * p_out,
            Cpw * pw_old,
        ])
        pa, pr, pw = np.linalg.solve(M, b)
        # massenerhaltende Aktualisierung der Sorptionsschichten
        self.Wr += dt * (Krb * (pa - pr) + Krc * (p_out - pr)) / (self.d_wood * self.A_wood)
        self.Ww += dt * Kwb * (pa - pw) / (self.d_wall * self.A_wall)
        ps_a = float(psat(self.Ta))
        if pa > ps_a:  # Nebel/Tauwasser in der Luft: auf Saettigung begrenzen
            self.fog_steps += 1
            pa = ps_a
        self.pa = pa
        return {"T_cav": T_cav}
