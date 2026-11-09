# Dachraum-Simulation: Tauwasser an der gekühlten Geschossdecke

Dieses Programm simuliert, ob sich im Sommer an schwülen Tagen über der
Betondecke Feuchte niederschlägt. Die Decke wird über Rohre im Beton gekühlt
(Bauteilaktivierung), und die Feuchte käme aus der Dachraumluft.

> **Wichtig:** Das ist ein vereinfachtes Rechenmodell und **keine**
> normgerechte hygrothermische Simulation nach DIN EN 15026 (wie sie z. B.
> WUFI rechnet). Alle Stoffwerte und viele Geometriewerte sind **Annahmen**.
> In `config.toml` sind sie mit `ANNAHME` markiert. Die Ergebnisse zeigen
> Größenordnungen und Tendenzen. Für eine Planungsentscheidung sollte eine
> Fachperson (Bauphysik) prüfen.

## Was modelliert wird

```
 Außenluft ── Hinterlüftung ── Rauhschalung + Unterdeckbahn ──┐
                                                               │ Diffusion / Luftwechsel
                     Dachraumluft  ◄── Holz (Sorption), Kniestockwände
                           │
               Mineralwolle (Annahme ~30 cm)      ◄── hier oder auf einer Folie: Tauwasser?
               ═══ Betonoberseite ═══              ◄── kritische Grenzfläche
               Stahlbeton mit Kühlrohren (VL 18 °C)
                           │
                     Wohnraum (Taupunktwächter unter der Decke)
```

* **Decke (`attic_sim/ceiling.py`)**: Die Decke wird eindimensional über die
  Zeit berechnet: Wärmeleitung und Wasserdampfdiffusion, dazu die
  Feuchtespeicherung in den Baustoffen (Sorptionsisothermen). Gekühlt wird
  über einen Wärmeübergang zwischen Kühlwasser und Rohrebene. Wird an einer
  Stelle Sättigung erreicht, rechnet das Modell die überschüssige Feuchte als
  freies Tauwasser. Dabei geht keine Feuchte rechnerisch verloren.
* **Dachraum (`attic_sim/attic.py`)**: Der Dachraum ist als Zone
  vereinfacht, mit drei Knoten: Luft, Holz (Schalung und Sparren) und
  Kniestockwände. Unter der oberen Schalung liegt die Hinterlüftung. Das
  Gründach dämpft stark, deshalb wird die obere Schalung als Tagesmittel der
  Außentemperatur plus Zuschlag angesetzt.
* **Wetter (`attic_sim/weather.py`)**: Es gibt ein künstliches Szenario aus
  `config.toml`, einen Import von DWD-Stundenwerten oder eine eigene CSV-Datei.
  Statt des Dachraummodells können auch **gemessene Dachraumwerte** eingelesen
  werden.
* **Taupunktwächter**: Er schaltet die Kühlung ab, wenn der Taupunkt im Raum
  über `T_Deckenunterseite − guard_margin_K` steigt.

## Nicht berücksichtigt (Grenzen)

* Flüssigtransport im Beton (Kapillarleitung). Für die Feuchteaufnahme im
  Sommer liegt das Ergebnis dadurch eher auf der sicheren Seite. Für die
  **Austrocknung von nassem Beton nach oben unterschätzt es das Risiko**.
  Als grobe Näherung dient die Variante `beton_nass`.
* Luftströmungen in oder durch die Dämmung, z. B. durch Fugen.
* Wärmebrücken am Kniestock und Auflager, mehrdimensionale Effekte.
* Die frei werdende Wärme beim Kondensieren (Latentwärme).
* Ob Schimmel wächst, berechnet das Modell nicht. Es zählt nur Stunden mit
  einer relativen Luftfeuchte über 80 % bzw. 95 % als Hinweis.

## Benutzung

```bash
pip install -r requirements.txt
python run.py                                    # Szenario aus config.toml
python run.py --set cooling.T_water_mean=18.5    # einzelne Werte überschreiben
python run.py --compare scenario.1.Td_mean=19,20,21,22   # Parametervarianten vergleichen
python run.py --set ceiling.membrane_sd=100      # Variante: PE-Folie auf dem Beton
python run.py --weather data/geosphere_...csv --format geosphere --station 16413 --seasons
python run.py --weather produkt_tu_stunde_XXXXX.txt --format dwd --start 2025-05-01 --end 2025-09-01
python run.py --weather aussen.csv --attic-csv dachraum.csv   # gemessene Werte (time,temp_c,rh)
python -m unittest discover -s tests             # Plausibilitätstests
```

Die Ergebnisse landen in `results/`: Zeitverläufe als PNG und CSV sowie
`zusammenfassung.json`.

## Ergebnisse für den bestätigten Aufbau

Bestätigt sind:

- Neubau in 8051 Thal bei Graz
- 20 cm Stahlbeton, Rohrlage 8 cm von unten
- keine Folie auf dem Beton
- 30 cm geschüttete Zellulose
- keine Raumentfeuchtung
- 100 m² Grundfläche, ca. 10° Dachneigung

Alles Übrige sind weiterhin Annahmen, die Zahlen daher **Größenordnungen**.

Gerechnet wurde mit künstlichem Wetter, **nicht** mit Grazer Messdaten:
30 Tage zum Einschwingen, dann eine schwüle Hitzeperiode (Tag 26 ± 6 °C),
danach Abkühlung. Ausgewertet wurde ab Tag 30. Die Startfeuchte des Betons
ist über den ganzen Querschnitt gleich angenommen, denn wann die Decke
betoniert wurde, ist unbekannt.

Abkürzungen in der Tabelle: Td = Außentaupunkt, r.F. = relative Luftfeuchte,
M-% = Wassergehalt in Masseprozent.

| Fall | Kühlung an | max. r.F. Betonoberseite | max. r.F. unterste Zellulose (Stunden ≥ 80 %) | max. Wassergehalt Zellulose | freies Tauwasser |
|---|---|---|---|---|---|
| Basis: Td 19 °C 14 Tage, Beton 90 % | 575 h | 79 % | 80 % (0 h) | 13 M-% | 0 |
| Td 21 °C 21 Tage | 507 h | 81 % | 82 % (153 h) | 14 M-% | 0 |
| + kältere Decke (Wasser 18,5 °C, guter Rohrübergang) | 387 h | 83 % | 84 % (299 h) | 15 M-% | 0 |
| + Beton sehr nass (97 %) | 389 h | 86 % | 87 % (561 h) | 16 M-% | 0 |
| kältere Decke, Taupunktwächter schaltet **nie** ab | 744 h | 85 % | 87 % (511 h) | 16 M-% | 0 |
| Vergleich: Beton bereits trocken (75 %), Td 21 °C | 508 h | 76 % | 77 % (0 h) | 12 M-% | 0 |

Was sich daraus ablesen lässt, unter Vorbehalt der Annahmen:

1. **Kein freies Tauwasser** in allen Fällen. Beton und Zellulose nehmen die
   Feuchte auf und puffern sie.
2. Bei diesem Neubau trägt die **Baufeuchte des Betons** offenbar ähnlich viel
   oder mehr zur Feuchte an der Unterseite der Zellulose bei als die schwüle
   Dachraumluft. Vergleiche dazu die letzte Zeile: Mit bereits trockenem Beton
   gibt es keine Stunden über 80 %. Die Kühlung macht den Beton kälter und
   verschärft das zusätzlich.
3. Über 80 % r.F. über einige hundert Stunden ist für die organische Zellulose
   ein Hinweis auf mögliches Schimmelrisiko. Ob tatsächlich etwas wächst,
   berechnet das Modell nicht. Das hängt auch vom Produkt ab, bitte beim
   Hersteller nachfragen.
4. Die Simulation deckt nur ca. zwei Monate ab. Wie schnell der Beton über
   mehrere Jahre austrocknet, ist damit nicht abgebildet.

## Ergebnisse mit echtem Wetter (GeoSphere Austria, Station 16413)

Datei: `data/geosphere_klima-v2-1d_16413_2021-2026.csv`, Tageswerte vom
21.10.2021 bis 20.10.2026. Datenquelle: GeoSphere Austria. Die
Lizenzbedingungen bitte selbst prüfen.

Aus Tagesminimum, Tagesmaximum und den Dampfdrücken zu den Klimaterminen
wird ein stündlicher Verlauf **genähert**. Gerechnet wurde jeweils ein Jahr
**ab dem Betonieren**, vom 21. Oktober bis 30. September, mit fünf
verschiedenen Wetterjahren. Weitere Annahmen:

- Startfeuchte Beton 97 %
- Kühlung von Mai bis September
- übrige Monate ohne Kühlung, Raum 21 °C

```bash
python run.py --weather data/geosphere_klima-v2-1d_16413_2021-2026.csv --format geosphere \
  --station 16413 --start 2023-10-21 --end 2024-10-01 --set cooling.start_day=0 \
  --set numerics.eval_from_day=0 --set ceiling.phi0_concrete=0.97 --set init.phi0_other=0.6 \
  --set init.T0=15 --seasons
```

Abkürzungen in den Tabellen: r.F. = relative Luftfeuchte, M-% = Wassergehalt
in Masseprozent.

**Sommer (Juni–September), Unterseite der Zellulose:**

| Wetterjahr | max. r.F. | Stunden ≥ 80 % | freies Tauwasser |
|---|---|---|---|
| 2022 | 67 % | 0 | 0 |
| 2023 | 74 % | 0 | 0 |
| 2024 | 76 % | 0 | 0 |
| 2025 | 68 % | 0 | 0 |
| 2026 | 70 % | 0 | 0 |

**Winter (November–März), Oberseite der Zellulose und Dachschalung:**

| Wetterjahr | max. r.F. oberste Zellulose | Stunden ≥ 80 % | max. Holzfeuchte Schalungsoberfläche |
|---|---|---|---|
| 2021/22 | 88 % | 2074 | 20 M-% |
| 2022/23 | 90 % | 2230 | 21 M-% |
| 2023/24 | 88 % | 2036 | 20 M-% |
| 2024/25 | 88 % | 2745 | 20 M-% |
| 2025/26 | 91 % | 2666 | 24 M-% |
| 2023/24, Beton bereits **trocken** (75 %) | 82 % | 351 | 18 M-% |
| 2023/24, nasser Beton + Kapillartransport grob genähert (µ = 30) | 96 % | 3399 | 23 M-% |

Was sich daraus ablesen lässt, unter Vorbehalt der Annahmen:

1. **Sommer:** Mit dem tatsächlichen Grazer Wetter der letzten fünf Jahre
   entsteht kein Tauwasser. Die Unterseite der Zellulose erreicht nie 80 %
   r.F. Die schwülen Tage in den Messdaten sind kürzer und weniger extrem als
   im künstlichen Szenario. Dort gab es zwei bis drei Wochen lang einen
   Tagesmittel-Taupunkt um 19–21 °C. In den Messdaten liegt er nur an
   einzelnen Tagen über 20 °C.
2. **Erster Winter nach dem Betonieren:** Hier zeigt das Modell das größere
   Risiko. Die Baufeuchte des Betons wandert durch die Zellulose nach oben in
   den kalten Dachraum. Die **Oberseite der Zellulose** liegt dann über
   2000–3400 Stunden bei über 80 % r.F. Die **Oberfläche der Dachschalung**
   erreicht rechnerisch bis zu ca. 20–24 M-% Holzfeuchte. Rund 20 M-% gilt
   häufig als Grenze, ab der Pilzbefall an Holz möglich wird. Das ist ein
   Hinweis, keine Prognose.
3. Mit trockenem Beton fallen diese Werte deutlich niedriger aus. Der
   Unterschied zeigt, dass die **Baufeuchte** die Hauptursache ist.
4. Das Modell bildet den Kapillartransport im Beton nicht ab. Für die
   Austrocknung nach oben **unterschätzt es das Risiko** deshalb eher. Die
   grobe Näherung mit µ = 30 zeigt, dass die Werte dann höher liegen.

## Empfohlene Messungen (z. B. mit ESPHome)

1. Einen Temperatur-/Feuchtesensor **auf die Betonoberseite unter die Dämmung**
   legen (z. B. SHT4x oder BME280, kapselbar). Das ist die wichtigste
   Messstelle.
2. Temperatur und Feuchte der Dachraumluft messen.
3. Vor- und Rücklauftemperatur der Kühlung erfassen.

Mit diesen Messreihen lassen sich die Annahmen ersetzen bzw. kalibrieren
(`--attic-csv`).
