# Grass precision validation

Run from the vehicle repository root:

```sh
uv run --with numpy python tools/grass_precision/validate_source.py
python3 tools/grass_precision/validate_integrated.py
```

The first command exits 1 while any maximum-error or coverage check fails. The committed candidate is **not qualified for a 15 mm field maximum**: 149/156 component scenarios pass, worst 15.436 mm. All intended samples are scored at 0.5 m/s; no paint pause removes failing points. The synthetic chassis is not field-calibrated. Earlier validation failures were later used for development; only final_holdout was not used to fit parameters.

The integrated check executes the real PathTracker, includes its normal 0.4 m run-out, and scores the full intended 64 m line. Its 12 normal scenarios pass, worst 9.191 mm. It is not full LineMission paint-completion certification.

Generated traces and results go to ignored `results/`. The archived report and recorded-input summary are in `docs/validation/grass-15mm-20261012/`. Raw 27-job MCAP archives remain local; the summary does not claim changed historical physical trajectories. Software tests: 535 passed, 363 subtests passed; one RunupStart comparison also fails on the original baseline.
