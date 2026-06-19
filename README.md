# AutoNav – Autonomous Navigation Simulation

A production-grade **autonomous navigation simulation** built with Python + Pygame-CE for a CSE final year project.

---

## Features

| Module | Details |
|---|---|
| **Environment** | Fully open, empty 2-D grid — border walls only; you design the layout |
| **Dynamic Obstacles** | Left-click to paint walls, right-click to erase in real-time |
| **User-Defined Target** | Press **T** to enter Target Mode, then click anywhere to drop the goal marker |
| **LIDAR Sensors** | 360° ray-cast at 45° intervals; colour-coded green → yellow → red |
| **A\* Pathfinding** | Octile heuristic, 8-directional, diagonal corner-cut prevention; auto-recalculates when path is blocked |
| **Vehicle Controller** | Smooth pixel-level motion; pre-aimed heading on every re-plan; emergency halt only on genuine threats |
| **Telemetry HUD** | Mode badge, speed, grid position, distance to target, 8 individual LIDAR ray readings |
| **Responsive Layout** | Auto-detects desktop resolution at startup — fills any laptop or external display |

---

## Prerequisites

- Python **3.10 +** (tested on 3.14)
- Windows / macOS / Linux

---

## Quick Start

```powershell
# 1. Clone the repository
git clone https://github.com/JadenBritto/AutoNav.git
cd AutoNav

# 2. Create & activate virtual environment
python -m venv venv
.\venv\Scripts\Activate.ps1        # Windows PowerShell
# source venv/bin/activate          # macOS / Linux

# 3. Install dependencies
pip install -r requirements.txt

# 4. Run the simulation
python simulation.py
```

---

## Controls

| Key / Button | Action |
|---|---|
| **T** | Toggle between **Obstacle Mode** and **Target Mode** |
| **Left-click** | Place obstacle *(Obstacle Mode)* / Set target *(Target Mode)* |
| **Right-click** | Remove obstacle (any mode) |
| **C** | Clear **all** obstacles |
| **SPACE** | Pause / Resume simulation |
| **R** | Reset vehicle to start & recalculate path |
| **ESC / Q** | Quit |

> **Tip:** The HUD badge in the sidebar always shows the current mode and a one-line hint.

---

## Architecture

```
simulation.py
│
├── SimulationEnv   – Empty grid world, border walls, dynamic obstacle & target management
├── Sensor          – LIDAR ray-caster (360°, 8 rays, ±22.5° forward cone for halt logic)
├── PathFinder      – A* search with octile heuristic & 8-directional movement
├── Vehicle         – Motion controller, heading pre-aim, waypoint-based safety checks
├── HUD             – Mode badge, telemetry sidebar, on-grid labels
└── Simulation      – Event loop, mode toggle, rendering orchestration
```

---

## Configuration (top of `simulation.py`)

| Constant | Value | Description |
|---|---|---|
| `VEHICLE_SPEED` | 3.5 px/fr | Movement speed |
| `SENSOR_RANGE` | 5 cells | Max LIDAR detection range |
| `SAFETY_THRESHOLD` | 0.7 cells | Emergency halt distance (forward cone only) |
| `RECALC_COOLDOWN` | 0.4 s | Minimum time between A\* re-plans |
| `FPS` | 60 | Target frame rate |
| `HUD_W` | 220 px | Sidebar width (fixed) |

> `CELL`, `COLS`, `ROWS`, `SCREEN_W`, `SCREEN_H` are computed **automatically** at runtime from the desktop resolution.

---

## Changelog / Bug Fixes

### v1.3 — Start-position false halt fix
**Problem:** When obstacles were placed beside the start cell, the vehicle would halt immediately without moving. The vehicle initialises with `heading = 0°` (East) and the forward sensor cone would detect the nearby side wall even though the planned path led in a completely different direction.

**Fix:** After every A\* recalculation a new `_aim_at_waypoint()` helper immediately rotates `heading` (and `_draw_angle`) to face the **first planned waypoint**. The sensor forward cone is therefore correct from frame 0.

---

### v1.2 — 1-cell corridor stuck fix
**Problem:** The vehicle would halt mid-journey when navigating through a 1-cell-wide gap. With a diagonal approach (e.g. heading NE into the gap), the NE sensor ray would hit the corridor side wall at ~1.44 cells — below the old threshold of 1.5 — causing a false emergency halt.

**Fixes applied:**

1. **`SAFETY_THRESHOLD` reduced `1.5 → 0.7`** — walls in a valid 1-cell gap are 0.5–1.0 cells from centre; 0.7 catches genuine imminent collisions only.

2. **Forward sensor cone narrowed to ±22.5°** — `front_distance()` now only uses a ray if it is within half of the 45° ray spacing of the vehicle's heading. Diagonal rays that belong to corridor side walls are silently ignored.

3. **Primary halt check changed to next-waypoint cell** — `Vehicle.update()` first checks `env.is_blocked(next_path_cell)` directly. This is the most reliable signal that a user-placed obstacle is blocking the path. The sensor distance is now a secondary emergency-only backup.

---

### v1.1 — Empty playground & user-defined target
**Changes:**

- Removed the hardcoded `RAW_MAP` maze; the grid now starts completely empty (border walls only).
- `env.target` is `Optional` — the vehicle idles with LIDAR active until the user places a target.
- New **Target Mode** (press **T**): left-click sets/moves the gold target marker; placing it on an existing obstacle automatically clears that cell.
- New **C** key clears all obstacles at once.
- HUD now shows a colour-coded mode badge with a context hint.

---

### v1.0 — Auto-resolution layout
**Change:** Replaced hardcoded `880×680` window with `_init_display_constants()` which queries the desktop resolution at startup and computes the largest square cell that fills the available screen, clamped between 24 px and 56 px per cell.

---

## License

MIT
