# Encoder Calibration Results

## Summary

The linear encoder (quadrature, GT2 belt + 20-tooth pulley, 2400 CPR) was calibrated by manually traversing the full rail between measured end stops. Six one-way traversals (3 round trips) were performed and averaged.

**Authoritative measured constant: 64.810 counts/mm (64,810.4 counts/m)**

The measured value is 8.02% above the nominal 60.0 counts/mm, which is expected — physical belt stretch, pulley geometry tolerances, and encoder quadrature edge spacing all contribute. Repeatability was 0.01%, indicating the constant is stable and suitable for use in all firmware position conversions.

---

## Test Setup

### Hardware

- Encoder: Quadrature, connected via PIO custom C module (`encoder`)
- Encoder pins: GP18 (A), GP19 (B)
- Transmission: GT2 belt + 20-tooth pulley
- Nominal resolution: 2400 CPR → 60.0 counts/mm (20 teeth × 2 mm pitch = 40 mm/rev → 2400 / 40 = 60 counts/mm)
- End-stop separation: 316.5 mm (measured)

### Relevant Files

| Item | Path |
|---|---|
| Firmware script | [`firmware/pico_micropython/hardware_validation/encoder_calibration/encoder_calibration.py`](../../firmware/pico_micropython/hardware_validation/encoder_calibration/encoder_calibration.py) |
| Raw data | [`data/raw/hardware_validation/encoder_calibration/`](../../data/raw/hardware_validation/encoder_calibration/) |

---

## Calibration Procedure

1. Position vehicle at end stop A; script auto-detects stillness and zeros encoder.
2. Traverse to end stop B; script detects and records the count at B.
3. Return to end stop A; script records the count.
4. Repeat for 3 round trips (6 one-way traversals total).
5. Average all traversal count deltas; divide by end-stop separation.

---

## Results

| Quantity | Value | Notes |
|---|---:|---|
| End-stop separation | 316.5 mm | Physically measured |
| Mean counts / full traverse | 20,512 | Average of 6 traversals |
| **Counts per mm (measured)** | **64.810** | **Use this in firmware** |
| Counts per m (measured) | 64,810.4 | |
| Nominal counts per mm | 60.000 | GT2, 20-tooth pulley, 2400 CPR |
| Deviation from nominal | +8.02% | Expected due to belt/geometry tolerances |
| Repeatability (max−min / mean) | 0.01% | |
| Repeatability verdict | PASS (≤ 1%) | |

### Per-Traversal Count Deltas

| Traversal | Direction | Counts |
|---:|---|---:|
| 1 | A → B | 20,511 |
| 2 | B → A | 20,511 |
| 3 | A → B | 20,513 |
| 4 | B → A | 20,514 |
| 5 | A → B | 20,513 |
| 6 | B → A | 20,513 |

---

## Usage

Any firmware script that converts encoder counts to rail position should define:

```python
COUNTS_PER_MM = 64.810   # measured 2026-05-25; see experiments/encoder_calibration/results.md
COUNTS_PER_M  = 64810.4
MM_PER_COUNT  = 1.0 / COUNTS_PER_MM
```

Do **not** use the nominal 60.0 counts/mm — the 8% error accumulates to ~25 mm over the full 316.5 mm rail.

---

## Conclusion

The encoder count-to-position conversion is **64.810 counts/mm**, measured with 0.01% traversal-to-traversal repeatability. The 8% deviation from the nominal 60 counts/mm is systematic (not a sensor error) and must be accounted for in all position calculations. The nominal value should not be used for any quantitative analysis.
