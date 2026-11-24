"""Vorlauftemperatur-Studie (trockene Decke, Zellulose, Sommer 2024, ohne Taupunktwaechter).

Aufruf aus tools/attic-sim:  python studies/vorlauf_sweep.py <Vorlauf degC> <Taupunkt-Zuschlag K>
Annahmen: Spreizung 3 K (mittlere Wassertemperatur = VL + 1,5 K), R_pipe = 0,03 m2K/W,
Startfeuchte Beton 75 %. Taupunkt-Zuschlag erhoeht den Aussentaupunkt (Ersatz fuer
durch Tageswerte geglaettete Spitzen).
"""
import tomllib,sys,copy,numpy as np
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from attic_sim import weather as wx; from attic_sim.simulate import run, summarize
from attic_sim.psychro import psat, dewpoint
VL=float(sys.argv[1]); dTd=float(sys.argv[2]); spread=3.0
base=tomllib.load(open('config_bauablauf.toml','rb'))
w0=wx.select(wx.load_geosphere_daily('data/geosphere_klima-v2-1d_16413_2021-2026.csv','16413'),'2023-11-09','2024-10-01')
w=dict(w0); w['p']=np.minimum(psat(dewpoint(w0['p'])+dTd),0.98*psat(w0['T']))
c=copy.deepcopy(base); c['ceiling']['phi0_concrete']=0.75
c['operation'][3]['T_water_mean']=VL+spread/2; c['cooling']['R_pipe']=0.03
r=run(c,w); s=summarize(r,c,0,months=(6,7,8,9))
print(f"VL {VL:5.1f} Td+{dTd}: T_beton {s['T Betonoberseite min / mittel [degC]']}, Td_att max {s['Taupunkt Dachraum max / mittel [degC]'][0]}, h Td_att>T_beton {s['Stunden Taupunkt Dachraum > T Betonoberseite']}, RH Zell.unten max {s['r.F. unterste Daemmlage max [%]']}, h>=80 {s['Stunden r.F. unterste Daemmlage >= 80 %']}, h>=95(Beton) {s['Stunden r.F. Betonoberseite >= 95 %']}, Tauwasser {s['Tauwasser max Beton Raumseite..Rohrebene / Beton oben / Daemmung [g/m2]']} g/m2, h Td_Raum>T_Decke_unten {s['Stunden Taupunkt Raum > T Deckenunterseite']}, Kuehl {s['Rohrleistung mittel, nur Betriebsstunden [W/m2] (+ = Kuehlen, - = Heizen)']}")
