"""Plausibilitaetstests: python -m unittest discover -s tests"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from attic_sim import materials  # noqa: E402
from attic_sim.ceiling import CeilingColumn  # noqa: E402
from attic_sim.psychro import DELTA_AIR, dewpoint, psat  # noqa: E402


def column(layers=None, **kw):
    layers = layers or [
        {"name": "Beton", "material": "beton", "d": 0.2, "pipe_height": 0.1},
        {"name": "Daemmung", "material": "mineralwolle", "d": 0.3},
    ]
    args = dict(R_pipe=0.07, h_room=10.0, beta_room=2e-8, sd_room=0.1, h_top=6.0,
                beta_top=2e-8, sd_top=0.0, phi0=0.6, T0=20.0)
    args.update(kw)
    return CeilingColumn(layers, **args)


class TestPsychro(unittest.TestCase):
    def test_psat_20C(self):
        # DIN 4108-3 Tabellenwert ~2337 Pa
        self.assertAlmostEqual(float(psat(20.0)), 2337, delta=5)

    def test_dewpoint_roundtrip(self):
        for T in (5.0, 15.0, 22.0):
            self.assertAlmostEqual(float(dewpoint(psat(T))), T, places=6)


class TestMaterials(unittest.TestCase):
    def test_isotherm_inverse(self):
        for m in materials.LIBRARY.values():
            for phi in (0.3, 0.8, 0.97):
                W = m.w(phi, 20.0)
                ph, cond = m.phi_from_w(W, 20.0)
                self.assertAlmostEqual(float(ph), phi, places=5, msg=m.name)
                self.assertEqual(float(cond), 0.0)

    def test_w80(self):
        m = materials.get("beton")
        self.assertAlmostEqual(float(m.w(0.8, 20.0)) - 0.15 * 0.8 * float(psat(20.0)) / (461.5 * 293.15),
                               m.w80, places=6)


class TestCeiling(unittest.TestCase):
    def test_steady_heat_flux_equals_U(self):
        col = column()
        for _ in range(3000):
            info = col.step(3600, 25.0, 1500.0, 15.0, 1500.0, False, 0.0)
        R = 1 / 10 + 0.2 / 2.3 + 0.3 / 0.035 + 1 / 6
        self.assertAlmostEqual(info["q_top"], 10.0 / R, delta=0.01 * 10 / R)

    def test_steady_vapour_flux(self):
        col = column(layers=[{"name": "MW", "material": "mineralwolle", "d": 0.3}],
                     sd_room=0.0, phi0=0.5)
        p_bot, p_top = 1800.0, 1000.0
        for _ in range(2000):
            info = col.step(3600, 20.0, p_bot, 20.0, p_top, False, 0.0)
        Z = 1 / 2e-8 + 0.3 / DELTA_AIR + 1 / 2e-8
        self.assertAlmostEqual(info["g_top"], (p_bot - p_top) / Z, delta=1e-3 * (p_bot - p_top) / Z)

    def test_mass_conservation_with_condensation(self):
        layers = [
            {"name": "Beton", "material": "beton", "d": 0.2, "pipe_height": 0.18},
            {"membrane": "PE", "sd": 100.0},
            {"name": "Daemmung", "material": "mineralwolle", "d": 0.1},
        ]
        col = column(layers=layers, R_pipe=0.01, phi0=0.7)
        m0 = np.sum(col.W * col.dx)
        net = 0.0
        dt = 600.0
        p_att = float(psat(22.0))  # Taupunkt 22 degC
        cond_max = 0.0
        for _ in range(24 * 6 * 5):
            info = col.step(dt, 25.0, 1600.0, 28.0, p_att, True, 14.0)
            net += (info["g_bot"] - info["g_top"]) * dt
            cond_max = max(cond_max, col.condensate_total())
        m1 = np.sum(col.W * col.dx)
        self.assertGreater(cond_max, 0.0)  # Tauwasser auf der Folie muss entstehen
        self.assertAlmostEqual(m1 - m0, net, delta=1e-6 + 1e-6 * abs(net))


if __name__ == "__main__":
    unittest.main()
