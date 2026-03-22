# RocketSim

Rocket Simulator used as an exam project, together with hopefully helping simulate how an actual rocket would move and be affected by external and internal factors.

## Features (high-level)
- Simulates rocket motion over time (time-step based).
- Models common external/internal influences (examples):
  - Gravity
  - Aerodynamic drag (if enabled)
  - Thrust / fuel burn (if enabled)
  - Mass changes during burn (if enabled)
- Produces results you can inspect (e.g., position/velocity/acceleration vs. time).

> Note: Exact supported models depend on what’s implemented in the codebase.

---

## Project structure (typical)
This project is Python-only. Common layouts for this kind of repo include:
- `src/` for source code
- `main.py` (or similar) as the entrypoint
- `requirements.txt` for dependencies
- `data/` or `configs/` for scenario configuration

If your structure differs, update the commands below accordingly.

---

## Requirements
- Python 3.10+ recommended (3.8+ may work depending on your dependencies)

If you have a `requirements.txt`, install dependencies with:
```bash
python -m pip install -r requirements.txt
```

If you do **not** have external dependencies, you can run using only the Python standard library.

---

## How to run
### Option A: Run the main script directly
If the entrypoint is `main.py`:
```bash
python main.py
```

### Option B: Run a module
If you have a package (e.g., `src/rocketsim`):
```bash
python -m rocketsim
```

### Common CLI arguments (optional)
If your program supports arguments, consider documenting them here, for example:
```bash
python main.py --dt 0.01 --t-max 60 --scenario scenarios/basic.json
```

---

## Configuration
Many rocket simulators allow you to configure parameters like:
- Initial mass, dry mass, fuel mass
- Thrust curve / constant thrust
- Burn time
- Drag coefficient (Cd), reference area
- Air density model
- Launch angle / rail angle
- Time step (`dt`) and simulation duration

Document where configuration lives in this repo (e.g., JSON/YAML files, Python constants, or a UI).

---

## Output
Document what the simulator produces, for example:
- Console summary (max altitude, burnout time, impact velocity, etc.)
- CSV output of the time series
- Plots (altitude vs time, velocity vs time, etc.)

If plots are produced, include example images in an `assets/` folder and embed them here.

---

## Physics notes (optional but helpful)
If you implemented any of these, briefly describe them here:
- Numerical integration method (Euler / Semi-implicit Euler / RK4, etc.)
- Drag model (quadratic: `F_d = 0.5 * rho * v^2 * Cd * A`)
- Mass flow / fuel burn model
- Coordinate system and sign conventions

---

## Testing (if applicable)
If you have tests:
```bash
python -m pytest
```

---

## Contributing
This is primarily an exam project, but improvements and fixes are welcome:
1. Fork the repo
2. Create a branch
3. Open a pull request with a clear description

---

## License
Add a license if you plan to share/reuse this project. Common choices:
- MIT (permissive)
- Apache-2.0 (permissive + patent grant)
- GPL-3.0 (copyleft)

If you haven’t decided yet, you can leave this section as “TBD”.

---

## Acknowledgements
- Any libraries used (NumPy, Matplotlib, etc.)
- Course/institution (if you want to credit it)
- References (rocket equations, drag models, NASA/ESA resources, etc.)
