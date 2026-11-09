"""Instationaeres 1D-Waerme- und Feuchtemodell der Geschossdecke (Raum -> Dachraum).

Finite-Volumen, implizit (Rueckwaerts-Euler). Feuchtetransport nur als
Dampfdiffusion (keine Kapillarleitung, keine Konvektion durch Leckagen).
Tauwasser entsteht, wenn der Dampfdruck in einer Zelle den Saettigungsdruck
erreichen wuerde; die Zelle wird dann auf p_sat festgehalten und die
ueberschuessige Feuchte als freies Tauwasser bilanziert (massenerhaltend).
"""
import math

import numpy as np
from scipy.linalg import solve_banded

from . import materials
from .psychro import DELTA_AIR, psat

C_WATER = 4190.0


def _grow(length, dx_min, dx_max, g=1.3):
    cells, s, x = [], 0.0, dx_min
    while s + x < length - 1e-12:
        cells.append(x)
        s += x
        x = min(x * g, dx_max)
    rest = length - s
    if cells and rest < 0.5 * cells[-1]:
        cells[-1] += rest
    else:
        cells.append(rest)
    return cells


def graded_cells(length, dx_min, dx_max, refine):
    """Zellbreiten; 'bottom' = fein an der Unterseite, 'top', 'both', 'none'."""
    if refine == "none":
        n = max(1, math.ceil(length / dx_max))
        return [length / n] * n
    if refine == "bottom":
        return _grow(length, dx_min, dx_max)
    if refine == "top":
        return _grow(length, dx_min, dx_max)[::-1]
    if refine == "both":
        half = _grow(length / 2, dx_min, dx_max)
        return half + half[::-1]
    raise ValueError(refine)


def _tridiag_solve(lower, diag, upper, rhs):
    ab = np.zeros((3, diag.size))
    ab[0, 1:] = upper
    ab[1, :] = diag
    ab[2, :-1] = lower
    return solve_banded((1, 1), ab, rhs)


class CeilingColumn:
    """Deckenaufbau von unten (Raum) nach oben (Dachraum).

    layers: Liste von dicts, von UNTEN nach OBEN, z.B.
        {"material": "beton", "d": 0.20, "dx_min": 0.002, "dx_max": 0.01,
         "refine": "both", "pipe_height": 0.10}
        {"membrane": "PE-Folie", "sd": 100.0}
    pipe_height: Hoehe der Rohrebene ueber Unterkante dieser Schicht [m].
    """

    def __init__(self, layers, R_pipe, h_room, beta_room, sd_room, h_top, beta_top,
                 sd_top, phi0, T0):
        dx, mats, labels = [], [], []
        iface_sd = []          # sd zwischen Zelle i und i+1 durch Folien
        pending_sd = 0.0
        self.pipe_index = None
        self.layer_bounds = {}
        z = 0.0
        for lay in layers:
            if "membrane" in lay:
                if not dx:
                    raise ValueError("Folie darf nicht die unterste Schicht sein (sd_room verwenden)")
                pending_sd += float(lay["sd"])
                continue
            mat = materials.get(lay["material"])
            cells = graded_cells(lay["d"], lay.get("dx_min", 0.002),
                                 lay.get("dx_max", 0.02), lay.get("refine", "both"))
            start = len(dx)
            for k, c in enumerate(cells):
                if dx:  # Grenzflaeche zur vorherigen Zelle
                    iface_sd.append(pending_sd if k == 0 else 0.0)
                pending_sd = 0.0 if k == 0 else pending_sd
                dx.append(c)
                mats.append(mat)
                labels.append(lay.get("name", mat.name))
            self.layer_bounds[lay.get("name", mat.name)] = (start, len(dx), z, z + lay["d"])
            if "pipe_height" in lay:
                centers = z + np.cumsum(cells) - np.asarray(cells) / 2
                self.pipe_index = start + int(np.argmin(np.abs(centers - (z + lay["pipe_height"]))))
            z += lay["d"]
        if pending_sd:
            raise ValueError("Folie darf nicht die oberste Schicht sein (sd_top verwenden)")

        self.dx = np.array(dx)
        self.mats = mats
        self.labels = labels
        self.z = np.cumsum(self.dx) - self.dx / 2
        self.N = len(dx)
        self.lam = np.array([m.lam for m in mats])
        self.rhoc = np.array([m.rho * m.c for m in mats])
        self.delta = np.array([DELTA_AIR / m.mu for m in mats])
        iface_sd = np.array(iface_sd)

        # Leitwerte zwischen Zellen
        self.G = 1.0 / (self.dx[:-1] / (2 * self.lam[:-1]) + self.dx[1:] / (2 * self.lam[1:]))
        self.K = 1.0 / (self.dx[:-1] / (2 * self.delta[:-1]) + iface_sd / DELTA_AIR
                        + self.dx[1:] / (2 * self.delta[1:]))
        self.G_bot = 1.0 / (1.0 / h_room + self.dx[0] / (2 * self.lam[0]))
        self.G_top = 1.0 / (1.0 / h_top + self.dx[-1] / (2 * self.lam[-1]))
        self.K_bot = 1.0 / (1.0 / beta_room + sd_room / DELTA_AIR + self.dx[0] / (2 * self.delta[0]))
        self.K_top = 1.0 / (1.0 / beta_top + sd_top / DELTA_AIR + self.dx[-1] / (2 * self.delta[-1]))
        self.R_pipe = R_pipe
        self.h_room = h_room

        # Gruppierung nach Material fuer vektorisierte Isothermen
        self._groups = {}
        for i, m in enumerate(mats):
            self._groups.setdefault(m, []).append(i)
        self._groups = {m: np.array(ix) for m, ix in self._groups.items()}

        self.T = np.full(self.N, float(T0))
        self.W = np.empty(self.N)
        for m, ix in self._groups.items():
            self.W[ix] = m.w(phi0, self.T[ix])
        self.phi, self.cond = self.state(self.T)
        self.p = self.phi * psat(self.T)

    # ------------------------------------------------------------------
    def reset_moisture(self):
        self.phi, self.cond = self.state(self.T)
        self.p = self.phi * psat(self.T)

    def state(self, T):
        phi = np.empty(self.N)
        cond = np.empty(self.N)
        for m, ix in self._groups.items():
            phi[ix], cond[ix] = m.phi_from_w(self.W[ix], T[ix])
        return phi, cond

    def _xi(self, phi, T):
        xi = np.empty(self.N)
        for m, ix in self._groups.items():
            xi[ix] = m.dw_dphi(phi[ix], T[ix]) / psat(T[ix])
        return xi

    def surface_temperature_bottom(self):
        """Oberflaechentemperatur der Deckenunterseite (Raumseite) [degC]."""
        if not hasattr(self, "_T_room"):
            return float(self.T[0])
        return float(self._T_room - (self._T_room - self.T[0]) * self.G_bot / self.h_room)

    # ------------------------------------------------------------------
    def step(self, dt, T_room, p_room, T_att, p_att, pipe_on, T_water):
        N = self.N
        self._T_room = T_room
        # ---------- Waerme ----------
        C = (self.rhoc + C_WATER * self.W) * self.dx / dt
        diag = C.copy()
        rhs = C * self.T
        diag[:-1] += self.G
        diag[1:] += self.G
        diag[0] += self.G_bot
        rhs[0] += self.G_bot * T_room
        diag[-1] += self.G_top
        rhs[-1] += self.G_top * T_att
        if pipe_on and self.pipe_index is not None:
            Gp = 1.0 / self.R_pipe
            diag[self.pipe_index] += Gp
            rhs[self.pipe_index] += Gp * T_water
        self.T = _tridiag_solve(-self.G, diag, -self.G, rhs)
        q_pipe = (self.T[self.pipe_index] - T_water) / self.R_pipe if (
            pipe_on and self.pipe_index is not None) else 0.0

        # ---------- Feuchte ----------
        phi, cond = self.state(self.T)
        ps = psat(self.T)
        p_old = phi * ps
        Cm = self._xi(phi, self.T) * self.dx / dt
        wet = cond > 0.0
        w_sat = np.empty(N)
        for m, ix in self._groups.items():
            w_sat[ix] = m.w(1.0, self.T[ix])
        for _ in range(2 * N + 2):
            diag = Cm.copy()
            rhs = Cm * p_old
            diag[:-1] += self.K
            diag[1:] += self.K
            diag[0] += self.K_bot
            rhs[0] += self.K_bot * p_room
            diag[-1] += self.K_top
            rhs[-1] += self.K_top * p_att
            lower = -self.K.copy()
            upper = -self.K.copy()
            # nasse Zellen: p = p_sat (Dirichlet)
            diag[wet] = 1.0
            rhs[wet] = ps[wet]
            upper[wet[:-1]] = 0.0
            lower[wet[1:]] = 0.0
            p = _tridiag_solve(lower, diag, upper, rhs)
            # massenerhaltende Bilanz
            g = self.K * (p[:-1] - p[1:])          # nach oben positiv
            g_bot = self.K_bot * (p_room - p[0])   # in die Decke hinein
            g_top = self.K_top * (p[-1] - p_att)   # aus der Decke in den Dachraum
            div = np.zeros(N)
            div[:-1] -= g
            div[1:] += g
            div[0] += g_bot
            div[-1] -= g_top
            W_new = self.W + dt * div / self.dx
            new_wet = (~wet) & (p > ps)
            # Tauwasser im Zeitschritt vollstaendig abgetrocknet -> nicht mehr auf p_sat halten
            dried = wet & (W_new < w_sat)
            if not new_wet.any() and not dried.any():
                break
            wet = (wet | new_wet) & ~dried
        self.W = W_new
        self.min_W = min(getattr(self, "min_W", 0.0), float(self.W.min()))
        self.phi, self.cond = self.state(self.T)
        self.p = p
        return {
            "q_top": self.G_top * (self.T[-1] - T_att),   # W/m2, Decke -> Dachraum
            "g_top": g_top,                               # kg/(m2 s), Decke -> Dachraum
            "g_bot": g_bot,                               # kg/(m2 s), Raum -> Decke
            "q_pipe": q_pipe,                             # W/m2 an Kuehlwasser
        }

    # Kenngroessen ------------------------------------------------------
    def condensate_total(self):
        """Freies Tauwasser in der Decke [kg/m2]."""
        return float(np.sum(self.cond * self.dx))

    def moisture_in(self, name, depth_from_top=None):
        """Wassermenge [kg/m2] in Schicht `name` (optional nur obere `depth_from_top` m)."""
        i0, i1, z0, z1 = self.layer_bounds[name]
        ix = np.arange(i0, i1)
        if depth_from_top is not None:
            ix = ix[self.z[ix] >= z1 - depth_from_top]
        return float(np.sum(self.W[ix] * self.dx[ix]))
