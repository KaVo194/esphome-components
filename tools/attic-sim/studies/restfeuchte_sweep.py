"""Restfeuchte-Studie: Welche Betonfeuchte beim Einbringen der Zellulose ist im Sommer unkritisch?

Aufruf aus tools/attic-sim:
    python studies/restfeuchte_sweep.py <phi0 Beton 0..1> <Taupunkt-Zuschlag K> [beton|beton_nass] [Vorlauf degC]
Annahmen: Zellulose wird am 1.5. trocken (60 % r.F.) eingebracht, Betonfeuchte gleichmaessig ueber
die Dicke, Wetter Mai-Sept. 2024 (GeoSphere 16413), Kuehlung ohne Taupunktwaechter,
Spreizung 3 K, R_pipe = 0,03 m2K/W.
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

phi0 = float(sys.argv[1])
dTd = float(sys.argv[2])
mat = sys.argv[3] if len(sys.argv) > 3 else "beton"
VL = float(sys.argv[4]) if len(sys.argv) > 4 else 18.0

root = os.path.join(HERE, "..")
with open(os.path.join(root, "config_bauablauf.toml"), "rb") as f:
    c = tomllib.load(f)
w0 = wx.select(wx.load_geosphere_daily(os.path.join(root, "data/geosphere_klima-v2-1d_16413_2021-2026.csv"),
                                       "16413"), "2024-05-01", "2024-10-01")
w = dict(w0)
w["p"] = np.minimum(psat(dewpoint(w0["p"]) + dTd), 0.98 * psat(w0["T"]))
c = copy.deepcopy(c)
c["ceiling"]["phi0_concrete"] = phi0
c["ceiling"]["layers"][0]["material"] = mat
c["init"]["T0"] = 20.0
c["init"]["phi0_other"] = 0.60
c["operation"][3]["T_water_mean"] = VL + 1.5
c["cooling"]["R_pipe"] = 0.03
r = run(c, w)
s = summarize(r, c, 0, months=(6, 7, 8, 9))
print(f"phi0 {phi0:.2f} {mat:10s} VL {VL} Td+{dTd}: Zellulose unten max {s['r.F. unterste Daemmlage max [%]']} %, "
      f"h>=80 {s['Stunden r.F. unterste Daemmlage >= 80 %']}, Beton oben max {s['r.F. Betonoberseite max [%]']} %, "
      f"Zellulose max {s['Wassergehalt unterste Daemmlage max [M-%]']} M-%, "
      f"Tauwasser {s['Tauwasser max Beton Raumseite..Rohrebene / Beton oben / Daemmung [g/m2]']}")
