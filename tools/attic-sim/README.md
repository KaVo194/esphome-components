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

## Erste Ergebnisse (alle Annahmen siehe config.toml)

Für alle Fälle wurde ein künstliches Wetterszenario gerechnet: zuerst 30 Tage
zum Einschwingen, dann eine schwüle Hitzeperiode (Tag 26 ± 6 °C, Außentaupunkt
wie in der Tabelle), danach Abkühlung. Ausgewertet wurde ab Tag 30.

| Fall | Taupunkt außen | Betonoberseite | max. r.F. Betonoberseite | max. r.F. unterste Dämmlage | freies Tauwasser |
|---|---|---|---|---|---|
| Basis (Wasser 19,5 °C, Raumfeuchte folgt außen) | 19 °C, 14 Tage | ≈21,4 °C | 77 % | 81 % | 0 |
| Basis, Hitzeperiode verschärft | 21 °C, 21 Tage | ≈21–22 °C | 81 % | 85 % | 0 |
| Kalte Decke (Wasser 18,5 °C, guter Rohrübergang, Beton feucht) | 21 °C, 21 Tage | ≈19,6–21 °C | 88 % | 91 % | 0 |
| wie oben, **Raum entfeuchtet** (50 % r.F., Wächter löst nie aus) | 21 °C, 21 Tage | ≈19,7 °C | 93 % | 98 % | 0 (Beton puffert) |
| wie oben + **PE-Folie auf Beton** | 21 °C, 21 Tage | ≈19,7 °C | – | 100 % | **bis ≈86 g/m²** auf der Folie |

Was sich daraus ablesen lässt, unter Vorbehalt der Annahmen:

1. **Ohne Folie** nimmt der Beton die Feuchte auf, die aus dem Dachraum nach
   unten wandert. Selbst wenn der Taupunkt im Dachraum zeitweise über der
   Temperatur der Betonoberseite liegt, entsteht in den gerechneten Fällen
   kein freies Wasser. Über längere Zeit wird es an der Grenzfläche aber
   **> 80 % r.F.** feucht. Bei Staub oder organischen Resten kann das für
   Schimmel relevant sein.
2. **Mit dampfdichter Folie auf dem Beton** fällt dieser Puffer weg. Dann
   bildet sich direkt auf der Folie Tauwasser. Im ungünstigsten Fall waren
   es bis ≈86 g/m² (bei 100 m² ≈ 9 Liter). In der Abkühlphase trocknet es
   wieder ab.
3. Der **Taupunktwächter im Raum** schützt den Dachraum nur indirekt. Das
   klappt, solange die Raumluft mindestens so feucht ist wie die Luft im
   Dachraum. Wird der Raum entfeuchtet, löst der Wächter nicht mehr aus und
   die Decke bleibt an schwülen Tagen kalt. Das ist der ungünstigste Fall.
4. Am stärksten hängt das Ergebnis von der **Temperatur der Betonoberseite**
   ab, also von Wassertemperatur, Rohrlage und Wärmeübergang. Danach folgt,
   **ob eine Folie auf dem Beton liegt**. Der Luftwechsel im Dachraum spielt
   nur eine kleine Rolle, weil die Feuchte vor allem durch Schalung und Bahn
   ausgetauscht wird.

## Empfohlene Messungen (z. B. mit ESPHome)

1. Einen Temperatur-/Feuchtesensor **auf die Betonoberseite unter die Dämmung**
   legen (z. B. SHT4x oder BME280, kapselbar). Das ist die wichtigste
   Messstelle.
2. Temperatur und Feuchte der Dachraumluft messen.
3. Vor- und Rücklauftemperatur der Kühlung erfassen.

Mit diesen Messreihen lassen sich die Annahmen ersetzen bzw. kalibrieren
(`--attic-csv`).
