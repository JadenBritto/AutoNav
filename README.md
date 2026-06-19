# AutoNav – Autonomous Navigation Simulation

A production-grade **autonomous navigation simulation** built with Python + Pygame-CE for a CSE final year project.

---

## Features

| Module | Details |
|---|---|
| **Environment** | 2-D grid (22 × 17 cells @ 40 px each) with static walls, narrow corridors, and open spaces |
| **Dynamic Obstacles** | Left-click to place, right-click to remove in real-time |
| **LIDAR Sensors** | 360° ray-cast at 45° intervals; colour-coded green→yellow→red |
| **A\* Pathfinding** | Octile heuristic, 8-directional, diagonal corner-cut prevention; auto-recalculates on path block |
| **Vehicle Controller** | Smooth pixel-level motion with heading interpolation; emergency halt on front-sensor trigger |
| **Telemetry HUD** | Speed, grid position, distance to target, individual ray readings |

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
| **Left-click** | Place obstacle on grid cell |
| **Right-click** | Remove obstacle from grid cell |
| **SPACE** | Pause / Resume simulation |
| **R** | Reset vehicle to start & recalculate path |
| **ESC / Q** | Quit |

---

## Architecture

```
simulation.py
│
├── SimulationEnv   – Grid world, static & dynamic obstacle management, rendering
├── Sensor          – LIDAR ray-caster (360°, 8 rays, configurable max range)
├── PathFinder      – A* search with octile heuristic & 8-directional movement
├── Vehicle         – Motion controller, sensor integration, telemetry properties
├── HUD             – Sidebar + on-grid text overlays
└── Simulation      – Event loop, rendering orchestration
```

---

## Configuration (top of `simulation.py`)

| Constant | Default | Description |
|---|---|---|
| `CELL` | 40 px | Grid cell size |
| `VEHICLE_SPEED` | 3.5 px/fr | Movement speed |
| `SENSOR_RANGE` | 5 cells | Max LIDAR range |
| `SAFETY_THRESHOLD` | 1.5 cells | Emergency halt distance |
| `RECALC_COOLDOWN` | 0.4 s | Minimum time between re-plans |
| `FPS` | 60 | Target frame rate |

---

## License

MIT
