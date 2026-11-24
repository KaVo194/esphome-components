"""Realer Plan: Decke bis zum Einbautag offen (ungedaemmt), dann Zellulose auf den
berechneten Beton-Feuchteverlauf, danach Kuehlsommer ohne Taupunktwaechter.

Aufruf aus tools/attic-sim:
    python studies/einbau_mai.py <Vorlauf degC> <Taupunkt-Zuschlag K> [beton|beton_nass] [Einbau MM-TT]
Wetter: Nov. 2023 - Sept. 2024 (GeoSphere 16413) als Stellvertreter fuer 2026/27.
"""
import copy
import os
import sys
import tomllib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from attic_sim import weather as wx  # noqa: E402
from attic_sim.psychro import dewpoint, psat  # noqa: E402
from attic_sim.simulate import run, summarize  # noqa: E402

VL = float(sys.argv[1])
dTd = float(sys.argv[2])
mat = sys.argv[3] if len(sys.argv) > 3 else "beton"
md = sys.argv[4] if len(sys.argv) > 4 else "05-01"
root = os.path.join(HERE, "..")
with open(os.path.join(root, "config_bauablauf.toml"), "rb") as f:
    base = tomllib.load(f)
base["ceiling"]["layers"][0]["material"] = mat
base["operation"][3]["T_water_mean"] = VL + 1.5
base["cooling"]["R_pipe"] = 0.03
wall = wx.load_geosphere_daily(os.path.join(root, "data/geosphere_klima-v2-1d_16413_2021-2026.csv"), "16413")
wall["p"] = np.where(np.isin((wall["time"].astype("datetime64[M]").astype(int) % 12) + 1, [5, 6, 7, 8, 9]),
                     np.minimum(psat(dewpoint(wall["p"]) + dTd), 0.98 * psat(wall["T"])), wall["p"])

# Phase 1: ungedaemmt bis zum Einbau
c1 = copy.deepcopy(base)
c1["ceiling"]["layers"] = c1["ceiling"]["layers"][:1]
r1 = run(c1, wx.select(wall, "2023-11-09", f"2024-{md}"))
col1 = r1["column"]

# Phase 2: Zellulose (trocken, 60 % r.F.) auf den Endzustand
c2 = copy.deepcopy(base)
c2["init"]["phi0_other"] = 0.60
c2["init"]["T0"] = float(col1.T[-1])
r2 = run(c2, wx.select(wall, f"2024-{md}", "2024-10-01"), init_concrete=(col1.W.copy(), col1.T.copy()))
s = summarize(r2, c2, 0, months=(6, 7, 8, 9))
top = 0.20 - col1.z
prof = ", ".join(f"{d*100:.0f} cm {np.interp(d, top[::-1], col1.phi[::-1])*100:.0f} %" for d in (0.01, 0.02, 0.04, 0.08))
print(f"Einbau {md}, {mat}, VL {VL}, Td+{dTd} | Beton beim Einbau (Tiefe von oben): {prof} | Sommer: Zellulose unten max "
      f"{s['r.F. unterste Daemmlage max [%]']} %, h>=80 {s['Stunden r.F. unterste Daemmlage >= 80 %']}, "
      f"Zellulose max {s['Wassergehalt unterste Daemmlage max [M-%]']} M-%, "
      f"Tauwasser {s['Tauwasser max Beton Raumseite..Rohrebene / Beton oben / Daemmung [g/m2]']}")
