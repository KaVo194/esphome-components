"""Kopplung Wetter -> Dachraum -> Decke (mit Bauteilaktivierung) und Auswertung."""
import numpy as np

from .attic import Attic
from .ceiling import CeilingColumn
from .psychro import dewpoint, p_from_rh, psat


def _rolling_mean(x, n):
    c = np.cumsum(np.insert(x, 0, 0.0))
    out = np.empty_like(x)
    for i in range(x.size):
        j0 = max(0, i - n + 1)
        out[i] = (c[i + 1] - c[j0]) / (i + 1 - j0)
    return out


def build_ceiling(cfg):
    c = cfg["ceiling"]
    return CeilingColumn(
        layers=c["layers"], R_pipe=cfg["cooling"]["R_pipe"],
        h_room=c["h_room"], beta_room=c["beta_room"], sd_room=c["sd_room_surface"],
        h_top=c["h_top"], beta_top=c["beta_top"], sd_top=c["sd_top_surface"],
        phi0=c["phi0_concrete"], T0=cfg["init"]["T0"])


def _set_initial_phi(col, cfg):
    """Unterschiedliche Startfeuchte: Beton vs. uebrige Schichten."""
    phi_other = cfg["init"]["phi0_other"]
    for m, ix in col._groups.items():
        phi = cfg["ceiling"]["phi0_concrete"] if m.name == "Stahlbeton" else phi_other
        col.W[ix] = m.w(phi, col.T[ix])
    col.reset_moisture()


def run(cfg, weather, attic_measured=None):
    """Simulation. attic_measured: optional dict(time,T,p) -> ersetzt Dachraummodell."""
    dt = float(cfg["numerics"]["dt_s"])
    t_h = (weather["time"] - weather["time"][0]).astype("timedelta64[s]").astype(float) / 3600.0
    T_out_h, p_out_h = weather["T"], weather["p"]
    T_up_h = _rolling_mean(T_out_h, 24) + cfg["attic"]["dT_upper"]

    if attic_measured is not None:
        xs = attic_measured["time"].astype("int64")
        xg = weather["time"].astype("int64")
        Ta_meas = np.interp(xg, xs, attic_measured["T"])
        pa_meas = np.interp(xg, xs, attic_measured["p"])

    col = build_ceiling(cfg)
    _set_initial_phi(col, cfg)
    att = Attic(cfg["attic"], T0=cfg["init"]["T0"], phi0=cfg["init"]["phi0_other"])
    if attic_measured is not None:
        att.Ta, att.pa = Ta_meas[0], pa_meas[0]

    cool = cfg["cooling"]
    room = cfg["room"]
    conc_name = next(n for n, b in col.layer_bounds.items()
                     if col.mats[b[0]].name == "Stahlbeton")
    i_conc_top = col.layer_bounds[conc_name][1] - 1
    i_ins_bot = i_conc_top + 1 if i_conc_top + 1 < col.N else i_conc_top

    n_steps = int(t_h[-1] * 3600 / dt)
    steps_per_h = max(1, int(round(3600 / dt)))
    rec = {k: [] for k in (
        "t_h", "T_out", "Td_out", "T_att", "Td_att", "RH_att", "T_conc_top", "RH_conc_top",
        "RH_ins_bot", "cond_g", "pipe_on", "T_ceil_bottom", "Td_room", "phi_wood",
        "q_pipe", "g_top", "u_ins_bot")}  # g_top: kumuliert [kg/m2], Decke -> Dachraum
    pipe_on = False
    guard_off_h = 0.0
    g_cum = 0.0
    for k in range(n_steps + 1):
        th = k * dt / 3600.0
        T_out = float(np.interp(th, t_h, T_out_h))
        p_out = float(np.interp(th, t_h, p_out_h))
        T_up = float(np.interp(th, t_h, T_up_h))

        # Raumklima
        T_room = room["T"]
        if room["humidity_mode"] == "outdoor_plus":
            p_room = min(p_out + room["dp_moisture_Pa"], 0.95 * float(psat(T_room)))
        else:
            p_room = float(p_from_rh(T_room, room["rh"]))

        # Kuehlung: Zeitplan + Taupunktwaechter im Raum (unter der Decke)
        hour = (th + cfg["numerics"]["start_hour_of_day"]) % 24
        day = th / 24.0
        sched = cool["enabled"] and cool["start_day"] <= day < cool.get("end_day", 1e9) and (
            cool["hour_on"] <= hour < cool["hour_off"] if cool["hour_on"] < cool["hour_off"]
            else (hour >= cool["hour_on"] or hour < cool["hour_off"]))
        guard_trip = float(dewpoint(p_room)) > col.surface_temperature_bottom() - cool["guard_margin_K"]
        pipe_on = bool(sched and not guard_trip)
        if sched and guard_trip:
            guard_off_h += dt / 3600.0

        # Dachraum
        if attic_measured is not None:
            Ta = float(np.interp(th, t_h, Ta_meas))
            pa = float(np.interp(th, t_h, pa_meas))
        else:
            att.step(dt, T_out, p_out, T_up, float(col.T[-1]), float(col.p[-1]),
                     col.G_top, col.K_top)
            Ta, pa = att.Ta, att.pa

        info = col.step(dt, T_room, p_room, Ta, pa, pipe_on, cool["T_water_mean"])
        g_cum += info["g_top"] * dt

        if k % steps_per_h == 0:
            rec["t_h"].append(th)
            rec["T_out"].append(T_out)
            rec["Td_out"].append(float(dewpoint(p_out)))
            rec["T_att"].append(Ta)
            rec["Td_att"].append(float(dewpoint(pa)))
            rec["RH_att"].append(pa / float(psat(Ta)))
            rec["T_conc_top"].append(float(col.T[i_conc_top]))
            rec["RH_conc_top"].append(float(col.phi[i_conc_top]))
            rec["RH_ins_bot"].append(float(col.phi[i_ins_bot]))
            rec["cond_g"].append(1000.0 * col.condensate_total())
            rec["pipe_on"].append(pipe_on)
            rec["T_ceil_bottom"].append(col.surface_temperature_bottom())
            rec["Td_room"].append(float(dewpoint(p_room)))
            rec["phi_wood"].append(att.phi_wood() if attic_measured is None else np.nan)
            rec["q_pipe"].append(info["q_pipe"])
            rec["g_top"].append(g_cum)
            rec["u_ins_bot"].append(100.0 * col.W[i_ins_bot] / col.mats[i_ins_bot].rho)
    res = {k: np.array(v) for k, v in rec.items()}
    res["time"] = weather["time"][0] + (res["t_h"] * 3600).astype("timedelta64[s]")
    res["guard_off_h"] = guard_off_h
    res["column"] = col
    res["concrete_layer"] = conc_name
    res["fog_steps"] = att.fog_steps if attic_measured is None else 0
    return res


def summarize(res, cfg, eval_from_day=0.0):
    m = res["t_h"] >= eval_from_day * 24
    A = cfg["attic"]["floor_area"]
    rh_c = res["RH_conc_top"][m]
    rh_i = res["RH_ins_bot"][m]
    cond = res["cond_g"][m]
    td_gt = res["Td_att"][m] > res["T_conc_top"][m]
    return {
        "Auswertezeitraum [h]": int(m.sum()),
        "Kuehlung aktiv [h]": int(res["pipe_on"][m].sum()),
        "Kuehlung durch Taupunktwaechter aus [h, gesamt]": round(res["guard_off_h"], 1),
        "T Betonoberseite min / mittel [degC]": (round(float(res["T_conc_top"][m].min()), 1),
                                                  round(float(res["T_conc_top"][m].mean()), 1)),
        "Taupunkt Dachraum max / mittel [degC]": (round(float(res["Td_att"][m].max()), 1),
                                                   round(float(res["Td_att"][m].mean()), 1)),
        "Stunden Taupunkt Dachraum > T Betonoberseite": int(td_gt.sum()),
        "r.F. Betonoberseite max [%]": round(100 * float(rh_c.max()), 1),
        "Stunden r.F. Betonoberseite >= 80 %": int((rh_c >= 0.80).sum()),
        "Stunden r.F. Betonoberseite >= 95 %": int((rh_c >= 0.95).sum()),
        "r.F. unterste Daemmlage max [%]": round(100 * float(rh_i.max()), 1),
        "Stunden r.F. unterste Daemmlage >= 80 %": int((rh_i >= 0.80).sum()),
        "Wassergehalt unterste Daemmlage max [M-%]": round(float(res["u_ins_bot"][m].max()), 1),
        "Tauwasser max [g/m2]": round(float(cond.max()), 1),
        "Tauwasser am Ende [g/m2]": round(float(cond[-1]), 1),
        "Tauwasser max gesamte Decke [Liter]": round(float(cond.max()) * A / 1000.0, 2),
        "Feuchteaufnahme Decke aus Dachraum [g/m2, netto]": round(
            -1000.0 * float(res["g_top"][m][-1] - res["g_top"][m][0]), 1),
        "Max. Holzfeuchte-Gleichgewicht Dachschalung r.F. [%]": (
            round(100 * float(np.nanmax(res["phi_wood"][m])), 1) if np.isfinite(res["phi_wood"][m]).any()
            else None),
        "Nebel-/Saettigungsschritte Dachraumluft": res["fog_steps"],
    }
