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

* Flüssigtransport im Beton (Kapillarleitung). Ohne ihn bleibt die Feuchte
  eher an der Oberfläche, das Ergebnis liegt also eher auf der sicheren Seite.
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
python run.py --weather produkt_tu_stunde_XXXXX.txt --format dwd --start 2025-05-01 --end 2025-09-01
python run.py --weather aussen.csv --attic-csv dachraum.csv   # gemessene Werte (time,temp_c,rh)
python -m unittest discover -s tests             # Plausibilitätstests
```

Die Ergebnisse landen in `results/`: Zeitverläufe als PNG und CSV sowie
`zusammenfassung.json`.

## Ergebnisse für den bestätigten Aufbau

Bestätigt: keine Folie auf dem Beton, 30 cm geschüttete Zellulose, keine
Raumentfeuchtung, Grundfläche 100 m², Dachneigung ca. 10°. Alle übrigen Werte
sind weiterhin Annahmen. Die Zahlen sind deshalb **Größenordnungen**.

Gerechnet wurde mit künstlichem Wetter: 30 Tage zum Einschwingen, dann eine
schwüle Hitzeperiode (Tag 26 ± 6 °C) mit dem Außentaupunkt aus der Tabelle,
danach Abkühlung. Ausgewertet wurde ab Tag 30.

| Fall | Taupunkt außen | Kühlung läuft | max. r.F. Betonoberseite | max. r.F. unterste Zellulose | max. Holzfeuchte-Äquivalent Zellulose | freies Tauwasser |
|---|---|---|---|---|---|---|
| Basis (Wasser 19,5 °C, Beton 75 % Startfeuchte) | 19 °C, 14 Tage | 575 h | 74 % | 75 % | 11 M-% | 0 |
| Hitzeperiode verschärft | 21 °C, 21 Tage | 514 h | 76 % | 78 % | 12 M-% | 0 |
| + kalte Decke (Wasser 18,5 °C, guter Rohrübergang) | 21 °C, 21 Tage | 397 h | 78 % | 80 % | 13 M-% | 0 |
| + Beton noch baufeucht (90 % Startfeuchte) | 21 °C, 21 Tage | 395 h | 83 % | 84 % | 15 M-% | 0 |
| kalte Decke, Taupunktwächter schaltet **nie** ab | 21 °C, 21 Tage | 744 h | 80 % | 83 % | 14 M-% | 0 |

Was sich daraus ablesen lässt, unter Vorbehalt der Annahmen:

1. In keinem dieser Fälle entsteht **freies Tauwasser**. Der Beton und die
   Zellulose selbst nehmen die Feuchte auf, die aus dem Dachraum nach unten
   wandert. Beide puffern.
2. In den ungünstigen Fällen liegt die **unterste Zelluloselage** über einige
   hundert Stunden bei **über 80 % r.F.**. Zellulose ist organisch. Ob die
   Schutzsalze im Produkt bei dieser Feuchte und Dauer Schimmel sicher
   verhindern, kann das Modell nicht beurteilen. Bitte im Datenblatt bzw. beim
   Hersteller nachfragen.
3. Der **Taupunktwächter im Raum** schaltet die Kühlung an schwülen Tagen oft
   ab, weil die Raumluft nicht entfeuchtet wird. Das schützt indirekt auch die
   Deckenoberseite. Wie stark, hängt von seiner Schaltschwelle ab. Die ist
   noch unbekannt, angenommen sind 1 K Abstand zur Deckenoberfläche.
4. Am stärksten hängt das Ergebnis von der **Temperatur der Betonoberseite**
   und davon ab, **wie feucht der Beton noch ist** (Neubau?).

## Empfohlene Messungen (z. B. mit ESPHome)

1. Einen Temperatur-/Feuchtesensor **auf die Betonoberseite unter die Dämmung**
   legen (z. B. SHT4x oder BME280, kapselbar). Das ist die wichtigste
   Messstelle.
2. Temperatur und Feuchte der Dachraumluft messen.
3. Vor- und Rücklauftemperatur der Kühlung erfassen.

Mit diesen Messreihen lassen sich die Annahmen ersetzen bzw. kalibrieren
(`--attic-csv`).
