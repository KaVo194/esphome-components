"""Wetterdaten: synthetische Szenarien oder CSV-Import.

Rueckgabe immer als dict mit stuendlichen numpy-Arrays:
    time (datetime64[s]), T (degC), p (Dampfdruck Pa)
"""
import csv
from datetime import datetime

import numpy as np

from .psychro import dewpoint, p_from_rh, psat


def synthetic(phases, start="2026-06-01T00:00", seed=1):
    """Tagesgang-Szenario aus Phasen.

    phase = {"days": 14, "T_mean": 25, "T_amp": 6, "Td_mean": 19, "Td_amp": 1, "label": "..."}
    T_amp ist die halbe Tagesschwankung, Maximum um 15 Uhr.
    """
    rng = np.random.default_rng(seed)
    T, Td = [], []
    for ph in phases:
        h = np.arange(int(ph["days"] * 24))
        diurnal = np.cos(2 * np.pi * ((h % 24) - 15) / 24)
        noise_T = rng.normal(0, ph.get("noise", 0.0), h.size)
        noise_Td = rng.normal(0, ph.get("noise", 0.0) / 2, h.size)
        t = ph["T_mean"] + ph["T_amp"] * diurnal + noise_T
        td = ph["Td_mean"] + ph.get("Td_amp", 1.0) * diurnal + noise_Td
        T.append(t)
        Td.append(np.minimum(td, t - 0.5))
    T = np.concatenate(T)
    Td = np.concatenate(Td)
    time = np.datetime64(start, "s") + np.arange(T.size) * np.timedelta64(3600, "s")
    return {"time": time, "T": T, "p": psat(Td)}


def _finalize(times, T, rh):
    order = np.argsort(times)
    times = np.array(times, dtype="datetime64[s]")[order]
    T = np.array(T, float)[order]
    rh = np.array(rh, float)[order]
    # auf ganzes Stundenraster interpolieren (fuellt Luecken linear)
    t0 = times[0].astype("datetime64[h]")
    t1 = times[-1].astype("datetime64[h]")
    grid = np.arange(t0, t1 + 1).astype("datetime64[s]")
    xs = times.astype("int64")
    xg = grid.astype("int64")
    T = np.interp(xg, xs, T)
    rh = np.clip(np.interp(xg, xs, rh), 0.01, 1.0)
    return {"time": grid, "T": T, "p": p_from_rh(T, rh)}


def load_csv(path, time_col="time", temp_col="temp_c", rh_col="rh", delimiter=","):
    """Generisches CSV: Zeit (ISO 8601), Temperatur [degC], rel. Feuchte [% oder 0..1]."""
    times, T, rh = [], [], []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f, delimiter=delimiter):
            try:
                t = datetime.fromisoformat(row[time_col].strip().replace("Z", "+00:00"))
                temp = float(row[temp_col])
                r = float(row[rh_col])
            except (ValueError, KeyError, TypeError):
                continue  # z.B. "unavailable" in Home-Assistant-Exporten
            times.append(np.datetime64(t.replace(tzinfo=None), "s"))
            T.append(temp)
            rh.append(r)
    if not times:
        raise ValueError(f"Keine gueltigen Zeilen in {path}")
    rh = np.array(rh)
    if np.nanmax(rh) > 1.5:
        rh = rh / 100.0
    return _finalize(times, T, rh)


def load_dwd_hourly(path):
    """DWD-Stundenwerte Lufttemperatur ('produkt_tu_stunde_*.txt').

    Erwartete Spalten (bitte mit der aktuellen DWD-Beschreibung pruefen):
    MESS_DATUM (YYYYMMDDHH, UTC), TT_TU (degC), RF_TU (%); Fehlwerte -999.
    """
    times, T, rh = [], [], []
    with open(path, newline="", encoding="latin-1") as f:
        reader = csv.DictReader(f, delimiter=";")
        for row in reader:
            row = {k.strip(): (v.strip() if isinstance(v, str) else v) for k, v in row.items()}
            try:
                temp = float(row["TT_TU"])
                r = float(row["RF_TU"])
                t = datetime.strptime(row["MESS_DATUM"], "%Y%m%d%H")
            except (ValueError, KeyError):
                continue
            if temp <= -999 or r <= -999:
                continue
            times.append(np.datetime64(t, "s"))
            T.append(temp)
            rh.append(r / 100.0)
    if not times:
        raise ValueError(f"Keine gueltigen DWD-Zeilen in {path}")
    return _finalize(times, T, rh)


def select(weather, start=None, end=None):
    m = np.ones(weather["time"].size, bool)
    if start:
        m &= weather["time"] >= np.datetime64(start, "s")
    if end:
        m &= weather["time"] < np.datetime64(end, "s")
    return {k: v[m] for k, v in weather.items()}


def dewpoint_of(weather):
    return dewpoint(weather["p"])


def _cos_interp(xq, xp, fp):
    """Interpolation mit Kosinus-Uebergang zwischen Stuetzstellen (glatter Tagesgang)."""
    i = np.clip(np.searchsorted(xp, xq) - 1, 0, len(xp) - 2)
    f = (xq - xp[i]) / (xp[i + 1] - xp[i])
    w = 0.5 - 0.5 * np.cos(np.pi * np.clip(f, 0.0, 1.0))
    return fp[i] + (fp[i + 1] - fp[i]) * w


def load_geosphere_daily(path, station=None):
    """GeoSphere Austria 'Stationsdaten ... in taeglicher Aufloesung' (klima-v2-1d), CSV.

    Benutzt tlmin/tlmax (Tagesextreme) und dampf_i/ii/iii (Dampfdruck hPa zu den
    Klimaterminen). Daraus wird ein STUENDLICHER Verlauf konstruiert:
      * Temperatur: Minimum ca. 6 Uhr, Maximum ca. 15 Uhr, Kosinus-Uebergang
      * Dampfdruck: lineare Interpolation zwischen den Terminen
    ANNAHME: Termine I/II/III = 7/14/19 Uhr Ortszeit (bitte in der
    GeoSphere-Metadatenbeschreibung pruefen). Sommerzeit wird ignoriert.
    Der echte Tagesgang ist damit nur genaehert.
    """
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if station is not None and r["station"].strip() != str(station):
                continue
            try:
                vals = [float(r[k]) for k in ("tlmin", "tlmax", "dampf_i", "dampf_ii", "dampf_iii")]
            except (ValueError, KeyError):
                continue
            day = np.datetime64(r["time"][:10], "D")
            rows.append((day, *vals))
    if not rows:
        raise ValueError(f"Keine vollstaendigen Tageswerte in {path} (Station {station})")
    rows.sort(key=lambda x: x[0])
    days = np.array([r[0] for r in rows])
    if np.unique(days).size != days.size:
        raise ValueError("Mehrere Stationen in der Datei -> --station angeben")
    d0 = days[0]
    dh = (days - d0).astype(float) * 24.0
    tmin = np.array([r[1] for r in rows])
    tmax = np.array([r[2] for r in rows])
    e = np.array([r[3:6] for r in rows]) * 100.0  # hPa -> Pa

    xT = np.concatenate([dh + 6.0, dh + 15.0])
    fT = np.concatenate([tmin, tmax])
    o = np.argsort(xT)
    xT, fT = xT[o], fT[o]
    xe = np.concatenate([dh + 7.0, dh + 14.0, dh + 19.0])
    fe = np.concatenate([e[:, 0], e[:, 1], e[:, 2]])
    o = np.argsort(xe)
    xe, fe = xe[o], fe[o]

    hours = np.arange(0.0, dh[-1] + 24.0)
    T = _cos_interp(hours, xT, fT)
    p = np.interp(hours, xe, fe)
    p = np.minimum(p, 0.98 * psat(T))
    time = d0.astype("datetime64[s]") + (hours * 3600).astype("timedelta64[s]")
    return {"time": time, "T": T, "p": p}
