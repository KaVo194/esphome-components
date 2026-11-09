#!/usr/bin/env python3
"""Dachraum-/Deckensimulation: Tauwasser an der gekuehlten Decke im Sommer.

Beispiele:
  python run.py                                   # synthetisches Szenario aus config.toml
  python run.py --set attic.air_change_per_h=0.5  # Parameter ueberschreiben
  python run.py --compare attic.air_change_per_h=0.05,0.1,0.5
  python run.py --weather produkt_tu_stunde_XXXX.txt --format dwd --start 2025-05-01 --end 2025-09-01
  python run.py --weather aussen.csv --attic-csv dachraum.csv   # gemessenes Dachraumklima
"""
import argparse
import copy
import json
import os
import tomllib

import numpy as np

from attic_sim import weather as wx
from attic_sim.simulate import run, summarize


def set_path(cfg, dotted, value):
    """Parameter per Punkt-Pfad setzen, z.B. attic.air_change_per_h oder scenario.1.Td_mean."""
    keys = dotted.split(".")
    d = cfg
    for k in keys[:-1]:
        d = d[int(k)] if isinstance(d, list) else d[k]
    last = keys[-1]
    if isinstance(d, list):
        last = int(last)
        old = d[last]
    else:
        old = d.get(last)
    if dotted == "ceiling.membrane_sd":  # Kurzform: Folie auf dem Beton einfuegen
        lay = cfg["ceiling"]["layers"]
        lay[:] = [l for l in lay if "membrane" not in l]
        if float(value) > 0:
            lay.insert(1, {"membrane": "Folie", "sd": float(value)})
        return
    if isinstance(old, bool):
        value = value.lower() in ("1", "true", "yes", "ja")
    elif isinstance(old, (int, float)):
        value = float(value)
    d[last] = value


def plot(res, cfg, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = res["time"].astype("datetime64[m]").astype(object)
    fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    a = ax[0]
    a.plot(t, res["T_out"], color="0.7", lw=0.8, label="Aussen T")
    a.plot(t, res["Td_out"], color="0.7", lw=0.8, ls="--", label="Aussen Taupunkt")
    a.plot(t, res["T_att"], color="tab:orange", label="Dachraum T")
    a.plot(t, res["Td_att"], color="tab:blue", label="Dachraum Taupunkt")
    a.plot(t, res["T_conc_top"], color="tab:green", lw=2, label="Betonoberseite T")
    a.plot(t, res["T_ceil_bottom"], color="tab:purple", lw=0.8, label="Deckenunterseite T")
    a.set_ylabel("degC")
    a.legend(ncol=3, fontsize=8)
    a.grid(alpha=0.3)
    a.set_title(title)

    a = ax[1]
    a.plot(t, 100 * res["RH_conc_top"], color="tab:green", label="r.F. Betonoberseite")
    a.plot(t, 100 * res["RH_ins_bot"], color="tab:red", label="r.F. unterste Daemmlage")
    a.plot(t, 100 * res["RH_att"], color="tab:blue", lw=0.8, label="r.F. Dachraumluft")
    a.axhline(80, color="k", ls=":", lw=0.8)
    a.axhline(95, color="k", ls="--", lw=0.8)
    a.set_ylabel("rel. Feuchte %")
    a.set_ylim(30, 102)
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    a = ax[2]
    a.plot(t, res["cond_g"], color="tab:red", label="freies Tauwasser in der Decke")
    a.set_ylabel("g/m2")
    on = res["pipe_on"].astype(float)
    a2 = a.twinx()
    a2.fill_between(t, 0, on, color="tab:cyan", alpha=0.15, step="post", label="Kuehlung an")
    a2.set_ylim(0, 1.05)
    a2.set_yticks([])
    a.legend(loc="upper left", fontsize=8)
    a2.legend(loc="upper right", fontsize=8)
    a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", nargs="?", default=os.path.join(here, "config.toml"))
    ap.add_argument("--weather", help="Aussenklima-CSV")
    ap.add_argument("--format", choices=["csv", "dwd"], default="csv")
    ap.add_argument("--attic-csv", help="gemessenes Dachraumklima (time,temp_c,rh)")
    ap.add_argument("--start")
    ap.add_argument("--end")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VAL")
    ap.add_argument("--compare", metavar="KEY=V1,V2,...")
    ap.add_argument("--out", default=os.path.join(here, "results"))
    args = ap.parse_args()

    with open(args.config, "rb") as f:
        cfg = tomllib.load(f)
    for s in args.set:
        k, v = s.split("=", 1)
        set_path(cfg, k, v)

    if args.weather:
        w = wx.load_dwd_hourly(args.weather) if args.format == "dwd" else wx.load_csv(args.weather)
        w = wx.select(w, args.start, args.end)
        cfg["numerics"]["start_hour_of_day"] = int(str(w["time"][0])[11:13])
    else:
        w = wx.synthetic(cfg["scenario"])
    attic = wx.load_csv(args.attic_csv) if args.attic_csv else None

    os.makedirs(args.out, exist_ok=True)
    variants = [("basis", cfg)]
    if args.compare:
        k, vals = args.compare.split("=", 1)
        variants = []
        for v in vals.split(","):
            c = copy.deepcopy(cfg)
            set_path(c, k, v)
            variants.append((f"{k}={v}", c))

    all_sum = {}
    for name, c in variants:
        if not args.weather:  # Szenario kann Teil der Variante sein
            w = wx.synthetic(c["scenario"])
        res = run(c, w, attic)
        summ = summarize(res, c, c["numerics"]["eval_from_day"])
        all_sum[name] = summ
        tag = name.replace("=", "_").replace(".", "-")
        plot(res, c, os.path.join(args.out, f"verlauf_{tag}.png"), name)
        np.savetxt(os.path.join(args.out, f"verlauf_{tag}.csv"),
                   np.column_stack([res["t_h"], res["T_out"], res["Td_out"], res["T_att"],
                                    res["Td_att"], res["T_conc_top"], res["RH_conc_top"],
                                    res["RH_ins_bot"], res["cond_g"], res["pipe_on"]]),
                   delimiter=",", fmt="%.4f",
                   header="t_h,T_out,Td_out,T_att,Td_att,T_conc_top,RH_conc_top,RH_ins_bot,cond_g_m2,pipe_on",
                   comments="")
        print(f"\n=== {name} ===")
        for k, v in summ.items():
            print(f"  {k}: {v}")
    with open(os.path.join(args.out, "zusammenfassung.json"), "w", encoding="utf-8") as f:
        json.dump(all_sum, f, ensure_ascii=False, indent=2, default=str)
    print(f"\nErgebnisse in {args.out}")


if __name__ == "__main__":
    main()
