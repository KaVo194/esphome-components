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

## Bauablauf im ersten Jahr (`config_bauablauf.toml`)

| ab | Phase | Raum | Decke |
|---|---|---|---|
| 09.11. | Rohbau, unbeheizt | 72-h-Mittel außen + 2 K, Baufeuchte (außen + 300 Pa) | Rohre aus |
| 01.02. | Heizung an, Bauaustrocknung | 20 °C, außen + 600 Pa | Heizen, Wasser im Mittel 28 °C |
| 01.04. | Bewohnt, Heizbetrieb | 21 °C, außen + 400 Pa | Heizen, 26 °C |
| 01.05. | Kühlbetrieb **ohne Taupunktwächter** | 25 °C, außen + 150 Pa | Kühlen, 19,5 °C, durchgehend |

Die Zahlen für Raumfeuchte und Heizwasser sind **Annahmen**. Die Startfeuchte
des Betons ist mit 97 % angesetzt. Gerechnet wurde mit fünf Wetterjahren
(GeoSphere, Station 16413), jeweils vom 9.11. bis 30.9.

```bash
python run.py config_bauablauf.toml --weather data/geosphere_klima-v2-1d_16413_2021-2026.csv \
  --format geosphere --station 16413 --start 2023-11-09 --end 2024-10-01 --seasons
```

Abkürzungen in der Tabelle: r.F. = relative Luftfeuchte, M-% = Wassergehalt
in Masseprozent.

| Fall | Winter: max. r.F. oberste Zellulose (Std. ≥ 80 %) | Winter: max. Holzfeuchte Schalungsoberfläche | Sommer: max. r.F. unterste Zellulose (Std. ≥ 80 %) | freies Tauwasser |
|---|---|---|---|---|
| Wetter 2021/22 | 85 % (1023) | 16 M-% | 69 % (0) | 0 |
| Wetter 2022/23 | 91 % (1073) | 18 M-% | 76 % (0) | 0 |
| Wetter 2023/24 | 85 % (1049) | 18 M-% | 77 % (0) | 0 |
| Wetter 2024/25 | 93 % (1207) | 21 M-% | 69 % (0) | 0 |
| Wetter 2025/26 | 93 % (1300) | 21 M-% | 71 % (0) | 0 |
| 2023/24, nasser Beton + Kapillartransport genähert | 93 % (1347) | 22 M-% | 79 % (0) | 0 |
| 2023/24, **kalte Decke** (Wasser 18,5 °C, guter Rohrübergang) | 86 % (1188) | 19 M-% | **82 % (336)** | 0 |

Was sich daraus ablesen lässt, unter Vorbehalt der Annahmen:

1. Solange der Rohbau **unbeheizt** ist, bleibt der Beton kalt. Dann wandert
   weniger Feuchte nach oben als bei einer beheizten Decke. Ab Februar, wenn
   die Decke heizt, steigt die Feuchte an der Oberseite der Zellulose und an
   der Dachschalung deutlich. Die Werte über 80 % r.F. und die Holzfeuchte bis
   ca. 21 M-% treten vor allem **Februar bis März** auf.
2. **Sommer ohne Taupunktwächter:** Bei 19,5 °C mittlerer Wassertemperatur
   entsteht weder Tauwasser noch eine Stunde über 80 % r.F. Ist die Decke
   kälter (Wasser 18,5 °C, gute Wärmeübertragung), liegt die Unterseite der
   Zellulose über 336 Stunden bei mehr als 80 % r.F. Freies Wasser entsteht
   auch dann nicht. Entscheidend ist also die **tatsächliche Temperatur der
   Betonoberseite**.

## Variante: 3 × 100 mm XPS statt Zellulose

Die 300 mm XPS sind als eine Schicht mit µ = 150 gerechnet, möglichst
diffusionsdicht verlegt. Das ist eine Annahme, der echte Wert hängt vom
Produkt ab, bitte im Datenblatt prüfen. Für undichte Stöße gibt es die
Variante `xps_fugen` mit µ = 30 (grobe Annahme). Der Bauablauf ist derselbe
wie oben.

```bash
python run.py config_bauablauf.toml --weather data/geosphere_klima-v2-1d_16413_2021-2026.csv \
  --format geosphere --station 16413 --start 2023-11-09 --end 2024-10-01 --seasons \
  --set ceiling.layers.1.material=xps --set ceiling.layers.1.refine=both
```

Abkürzungen in der Tabelle: r.F. = relative Luftfeuchte, M-% = Wassergehalt
in Masseprozent.

| Fall | max. r.F. oberste Dämmlage (Std. ≥ 80 %) | max. Holzfeuchte Schalungsoberfläche | r.F. Betonoberseite | Wasser im Beton Start → Ende (kg/m²) | Tauwasser max. |
|---|---|---|---|---|---|
| **Zellulose**, Wetter 2023/24 (Vergleich) | 85 % (1049) | 18 M-% | meist < 80 % | 27,4 → 23,3 | 0 |
| **XPS**, Wetter 2021/22 – 2025/26 | 73–78 % (0) | 13–15 M-% | ganzjährig ≈ 95–97 % | 27,4 → 24,9–25,2 | 2–6 g/m² (Feb.–März, im XPS) |
| XPS mit undichten Fugen (µ = 30), 2023/24 | 74 % (0) | 13 M-% | ≈ 95–97 % | 27,4 → 24,8 | 1 g/m² |
| XPS, kalte Decke im Sommer, 2023/24 | 74 % (0) | 13 M-% | ≈ 95–97 % | 27,4 → 25,4 | 0,1 g/m² |

Was sich daraus ablesen lässt, unter Vorbehalt der Annahmen:

1. **Dachraum:** Mit XPS gelangt praktisch keine Feuchte mehr aus dem Beton in
   den Dachraum. Dachschalung und Oberseite der Dämmung bleiben trocken, auch
   im Winter. Auch im Sommer kann kaum Dachraumluft an den kalten Beton
   gelangen.
2. **Beton:** Er kann nicht mehr nach oben austrocknen, sondern nur noch nach
   unten in den Raum. Die Oberseite des Betons bleibt im ersten Jahr
   ganzjährig bei ca. 95–97 % r.F. Nach elf Monaten hat er ca. 2,2–2,5 kg/m²
   Wasser verloren, mit Zellulose waren es ca. 4,1 kg/m². Die Feuchte muss
   also über die Deckenunterseite in die Räume. Ein diffusionsoffener
   Anstrich bzw. Putz und gutes Lüften sind dann wichtiger.
3. **Heizbetrieb Februar/März:** Die beheizte, nasse Decke drückt Dampf in das
   XPS. Rechnerisch fallen dort 2–6 g/m² Tauwasser aus, im unteren bis
   mittleren Bereich des XPS. Bei mehrlagiger Verlegung sammelt es sich in
   der Praxis wahrscheinlich in den Lagenfugen. Die Menge ist klein und
   trocknet bis April wieder ab.
4. Die Oberseite des Betons bleibt dauerhaft feucht. Unproblematisch ist das
   nur, solange dort keine organischen Reste liegen, z. B. Holz, Schalungsreste
   oder Schmutz.

## Variante: Decke ganz ohne Dämmung

Alle Schichten über dem Beton entfallen (`--set ceiling.insulation=none`). Der
Bauablauf ist derselbe wie oben. In der Tabelle sind „Betonoberseite“ und
„unterste/oberste Dämmlage“ deshalb dieselbe Stelle.

```bash
python run.py config_bauablauf.toml --weather data/geosphere_klima-v2-1d_16413_2021-2026.csv \
  --format geosphere --station 16413 --start 2023-11-09 --end 2024-10-01 --seasons \
  --set ceiling.insulation=none
```

Abkürzungen in der Tabelle: r.F. = relative Luftfeuchte, M-% = Wassergehalt
in Masseprozent.

| Fall | Winter: max. Holzfeuchte Schalungsoberfläche | Winter: Std. r.F. Betonoberseite ≥ 80 % | Sommer: max. r.F. Betonoberseite (Std. ≥ 80 %) | Wasser im Beton Start → Ende (kg/m²) | Tauwasser |
|---|---|---|---|---|---|
| Ohne Dämmung, Wetter 2021/22 – 2025/26 | 15–17 M-% | 221–573 | 68–78 % (0) | 27,4 → 22,8–23,3 | 0 |
| Ohne Dämmung, kalte Decke im Sommer (2023/24) | 15 M-% | 221 | 84 % (469) | 27,4 → 23,5 | 0 |
| Ohne Dämmung, nasser Beton + Kapillartransport genähert | 17 M-% | 1051 | 78 % (0) | 27,4 → 19,3 | 0 |
| Zum Vergleich: Zellulose (2023/24) | 18 M-% | – | 77 % (0) | 27,4 → 23,3 | 0 |

Energie und Dachraumtemperatur, Wetter 2023/24:

| | ohne Dämmung | 30 cm Zellulose |
|---|---|---|
| Dachraum Winter min / mittel / max | −2 / 10 / 21 °C | −3 / 6 / 17 °C |
| Dachraum Sommer min / mittel / max | 16 / 22 / 26 °C | 11 / 23 / 29 °C |
| Wärmestrom Decke → Dachraum, Mittel Nov–März | 12,8 W/m² | 0,9 W/m² |
| Heizleistung der Rohre (Mittel über die Heizstunden) | 59 W/m² | 42 W/m² |
| Kühlleistung der Rohre (Mittel über die Kühlstunden) | 28 W/m² | 27 W/m² |

Was sich daraus ablesen lässt, unter Vorbehalt der Annahmen:

1. **Feuchte:** Ohne Dämmung entsteht kein Tauwasser. Im Winter wärmt die
   beheizte Decke den Dachraum mit. Dachschalung und Dachraumluft bleiben
   dadurch eher trockener als mit Zellulose. Der Beton trocknet ähnlich
   schnell wie mit Zellulose.
2. **Sommer:** Die Betonoberseite liegt direkt an der Dachraumluft. Bei
   19,5 °C Wassertemperatur gibt es keine Stunde über 80 % r.F. Bei kalter
   Decke sind es ca. 470 Stunden mit bis zu 84 %. Freies Wasser entsteht auch
   dann nicht, weil der Beton puffert.
3. **Energie:** Im Winter heizt die Decke den Dachraum mit. Über November bis
   März fließen im Mittel ca. 13 W/m² nach oben, bei 100 m² also ca. 1,3 kW.
   Mit Zellulose sind es ca. 0,1 kW. Das ist eine grobe Größenordnung aus dem
   vereinfachten Dachraummodell. Ob die Heizlast und die Bauvorschriften das
   zulassen (Wärmeschutz der obersten Geschossdecke), muss eine Fachplanung
   prüfen.

## Studie: Ab welcher Vorlauftemperatur entsteht Tauwasser? (trockene Decke)

Grundlage ist das Modell mit dem Bauablauf oben:

- 30 cm Zellulose
- Beton **trocken** (Startfeuchte 75 %)
- Wetter 2023/24, ausgewertet Juni–September 2024
- **ohne Taupunktwächter**
- Spreizung 3 K, also mittlere Wassertemperatur = Vorlauf + 1,5 K (Annahme)
- guter Wärmeübergang von den Rohren in den Beton (R_pipe = 0,03)

Weil die Stundenwerte aus Tageswerten genähert sind, wurde zusätzlich mit
einem um 1,5 K erhöhten Außentaupunkt gerechnet. Skript:
`studies/vorlauf_sweep.py`.

Abkürzungen in der Tabelle: VL = Vorlauftemperatur, r.F. = relative
Luftfeuchte, Td = Taupunkt.

| VL | Betonoberseite | Zellulose unten: max. r.F. (Std. ≥ 80 %) | Tauwasser in der Zellulose | Std. Td Raum > Deckenunterseite (Raumseite!) |
|---|---|---|---|---|
| 18 °C | 20,5 °C | 75 % (0) / 80 % (0) | 0 / 0 | 47 / 228 |
| 17 °C | 19,7 °C | 78 % (0) / 83 % (562) | 0 / 0 | 119 / 355 |
| 16 °C | 18,9 °C | 80 % (2) / 85 % (785) | 0 / 0 | 211 / 519 |
| 15 °C | 18,1 °C | 83 % (567) / 88 % (1428) | 0 / 0 | 332 / 750 |
| 14 °C | 17,3 °C | 85 % (771) / 91 % (1564) | 0 / 0 | 487 / 953 |
| 12 °C | 15,7 °C | 91 % (1564) / 96 % (2156) | 0 / 0 | 937 / 1455 |
| 11 °C | 14,9 °C | – / 99 % (2354) | – / 0 | – / 1642 |
| 10 °C | 14,1 °C | 96 % (2142) / 100 % | 0 / **40 g/m²** | 1431 / 1835 |
| 9 °C | 13,3 °C | 99 % (2306) / – | 0 / – | 1627 / – |
| 8 °C | 12,4 °C | 100 % / – | **38 g/m²** / – | 1828 / – |
| 7 °C | 11,6 °C | 100 % / – | **146 g/m²** / – | 1977 / – |

Jeweils: Wetter 2024 wie gemessen / Außentaupunkt + 1,5 K; „–“ = nicht gerechnet.

Was sich daraus ablesen lässt, unter Vorbehalt der Annahmen:

1. **Freies Tauwasser im Dachraum** (in der untersten Zelluloselage) entsteht
   bei trockener Decke erst unter ca. **9 °C Vorlauf**. Mit erhöhtem
   Taupunkt sind es ca. **11 °C**. So tief liegt die Grenze, weil Beton und
   Zellulose viel Feuchte aufnehmen, bevor flüssiges Wasser entsteht.
2. **Über 80 % r.F.** an der Zellulose-Unterseite über Hunderte Stunden
   treten schon deutlich früher auf: ab ca. **15 °C Vorlauf**, mit erhöhtem
   Taupunkt ab ca. **17 °C**. Für die organische Zellulose ist diese Grenze
   die relevantere.
3. **Ohne Taupunktwächter begrenzt die Raumseite, nicht der Dachraum.**
   Schon bei 18 °C Vorlauf liegt der Taupunkt der Raumluft im Modell über 47
   bzw. 228 Stunden über der Temperatur der Deckenunterseite. Angenommen ist
   dabei, dass die Raumluft so feucht ist wie außen plus 150 Pa. Dann droht
   Tauwasser an der Deckenunterseite im Wohnraum. Im Beton unterhalb der
   Rohrebene sammelt sich rechnerisch ab ca. 12–15 °C Vorlauf freies Wasser.
   Wie oft das eintritt, hängt stark von der Raumfeuchte ab: Lüftung,
   Personen, Entfeuchtung.

## Studie: Welche Restfeuchte des Betons beim Einbringen der Zellulose?

Für alle Läufe gilt:

- Zellulose wird trocken eingebracht (60 % r.F.)
- Kühlung ohne Taupunktwächter
- Spreizung 3 K, R_pipe = 0,03
- Auswertung Juni–September 2024
- zusätzlich ein Lauf mit Außentaupunkt + 1,5 K als Ersatz für geglättete Spitzen

Abkürzungen: r.F. = relative Luftfeuchte, Td = Taupunkt, VL = Vorlauftemperatur.

**a) Gleichmäßige Betonfeuchte beim Einbau am 1.5.** (`studies/restfeuchte_sweep.py`), Vorlauf 18 °C:

| Betonfeuchte (gleichmäßig) | Zellulose unten max. r.F. (Std. ≥ 80 %), Wetter gemessen | dto., Td + 1,5 K |
|---|---|---|
| 75 % | 79 % (0) | 83 % (606) |
| 80 % | 80 % (0) | 84 % (823) |
| 85 % | 81 % (124) | 85 % (1217) |
| 90 % | 82 % (581) | 87 % (1510) |
| 95 % | 84 % (1037) | 88 % (1654) |

Mit Td + 1,5 K reichen selbst 60 % Betonfeuchte nicht für 0 Stunden (22 h).
Dafür braucht es einen höheren Vorlauf: Bei 70 % Betonfeuchte genügen
19 °C, bei 80 % genügen 20 °C.

**b) Realer Plan: Decke bleibt bis 1.5. offen, dann Zellulose**
(`studies/einbau_mai.py`). Gerechnet ist der Feuchteverlauf im Beton beim
Einbau. Laut Modell ist dann nur die Oberfläche abgetrocknet:

| | 1 cm | 2 cm | 4 cm | 8 cm Tiefe |
|---|---|---|---|---|
| Beton (Standard) | 61 % | 83 % | 96 % | 97 % |
| Beton mit Kapillar-Näherung | 55 % | 66 % | 85 % | 96 % |

| Fall | Zellulose unten max. r.F. (Std. ≥ 80 %) |
|---|---|
| VL 16 °C, Wetter gemessen | 84 % (666) |
| VL 18 °C, Wetter gemessen | 79 % (0) |
| VL 18 °C, Td + 1,5 K | 82–83 % (496–625) |
| VL 20 °C, Td + 1,5 K | 78 % (0) |

Was sich daraus ablesen lässt, unter Vorbehalt der Annahmen:

1. Den Beton bis Mai 2027 über die ganze Dicke auf ≤ 80 % r.F. zu bringen,
   ist laut Modell **unrealistisch**. Der Kern bleibt bei ca. 96 %.
2. Im ersten Sommer zählt vor allem die **oberste Betonzone (ca. 0–4 cm)**,
   kaum der Kern. Mit dem Feuchteverlauf vom 1.5. und 18 °C Vorlauf bleibt
   die Zellulose bei gemessenem Wetter unter 80 %.
3. Für sehr schwüle Phasen braucht es zusätzlich eine Betriebsregel: an
   Tagen mit hohem Taupunkt **Vorlauf ca. 20 °C**.

## Empfohlene Messungen (z. B. mit ESPHome)

1. Einen Temperatur-/Feuchtesensor **auf die Betonoberseite unter die Dämmung**
   legen (z. B. SHT4x oder BME280, kapselbar). Das ist die wichtigste
   Messstelle.
2. Temperatur und Feuchte der Dachraumluft messen.
3. Vor- und Rücklauftemperatur der Kühlung erfassen.

Mit diesen Messreihen lassen sich die Annahmen ersetzen bzw. kalibrieren
(`--attic-csv`).
