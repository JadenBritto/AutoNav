"""
=============================================================================
  AUTONOMOUS NAVIGATION SIMULATION
  CSE Final Year Project
  Author  : AutoNav Project
  Python  : 3.10+
  Deps    : pygame-ce  (pip install pygame-ce)
            Compatible with both pygame and pygame-ce (API identical)
=============================================================================

CONTROLS
--------
  Left-click   → Place obstacle on grid
  Right-click  → Remove obstacle from grid
  R            → Reset vehicle to start & recalculate path
  SPACE        → Pause / Resume simulation
  ESC / Q      → Quit

CLASSES
-------
  SimulationEnv  – World grid, obstacle management, rendering primitives
  Sensor         – 360° LIDAR ray-caster
  PathFinder     – A* search with Octile heuristic (8-directional)
  Vehicle        – Movement controller, telemetry, integration hub
  HUD            – Real-time telemetry sidebar renderer
  Simulation     – Main loop & event orchestration
"""

from __future__ import annotations

import heapq
import math
import sys
import time
from typing import Dict, List, Optional, Set, Tuple

import pygame

# ---------------------------------------------------------------------------
# ─── CONSTANTS ──────────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

SCREEN_W: int = 880          # Window width  (px)
SCREEN_H: int = 680          # Window height (px)
CELL: int     = 40           # Grid cell size (px)

COLS: int = SCREEN_W // CELL
ROWS: int = SCREEN_H // CELL

# HUD sidebar width in pixels (drawn on the right)
HUD_W: int = 220
GRID_W: int = SCREEN_W - HUD_W   # area used by the grid

FPS: int = 60

# ── Colour palette ──────────────────────────────────────────────────────────
C_BG         = (15,  17,  26)    # dark background
C_GRID       = (30,  35,  50)    # grid lines
C_WALL       = (55,  62,  90)    # static wall cells
C_WALL_EDGE  = (80,  90, 130)    # wall edge highlight
C_OBSTACLE   = (200,  60,  60)   # dynamic obstacle
C_OBS_EDGE   = (255, 100, 100)
C_START      = ( 60, 200, 120)   # start cell
C_TARGET     = (255, 215,   0)   # target cell
C_PATH       = ( 70, 130, 255)   # A* path
C_PATH_FADE  = ( 30,  70, 180)
C_SENSOR_OK  = ( 50, 230,  80)   # sensor ray – clear
C_SENSOR_WARN= (255, 200,   0)   # sensor ray – warning zone
C_SENSOR_HIT = (255,  60,  60)   # sensor ray – hazard
C_VEHICLE    = ( 30, 180, 255)   # vehicle body
C_VEHICLE_HL = (180, 240, 255)   # vehicle highlight
C_HUD_BG     = (12,  14,  22)
C_HUD_LINE   = (40,  50,  75)
C_TEXT_PRI   = (220, 230, 255)
C_TEXT_SEC   = (120, 140, 180)
C_TEXT_OK    = ( 60, 210, 100)
C_TEXT_WARN  = (255, 180,  40)
C_TEXT_ERR   = (255,  80,  80)
C_ACCENT     = ( 80, 160, 255)

# ── Vehicle / sensor tuning ─────────────────────────────────────────────────
VEHICLE_SPEED     = 3.5          # pixels per frame
SENSOR_RANGE      = 5            # cells
SAFETY_THRESHOLD  = 1.5          # cells – halt distance in front
RECALC_COOLDOWN   = 0.4          # seconds between forced recalculations
RAY_ANGLES_DEG    = [0, 45, 90, 135, 180, 225, 270, 315]   # 360° at 45° steps


# ---------------------------------------------------------------------------
# ─── PREDEFINED MAP ─────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------
# '.' = open, 'W' = wall, 'S' = start, 'T' = target
# Grid is COLS × ROWS (22 × 17)

RAW_MAP = """
......................
.WWWWWWWW............
.W......W............
.W..WW..WWWWWWWWWWW.
.W..W..............W.
.W..WWWWWWW.....W..W.
.W.........W....W..W.
.W..WWWWW..WWWWWW..W.
.W..W....W.........W.
.W..W.WW.WWWWWWWWWW.
.W..W.W............W.
.W..W.WWWWWWWWW.WW.W.
.W..W...........W..W.
.WWWW.WWWWWWWW.WW..W.
.S........T.......WW.
....................W.
....................W.
""".strip().splitlines()

# Ensure the map fits our grid (pad / trim)
def _build_wall_set() -> Set[Tuple[int, int]]:
    walls: Set[Tuple[int, int]] = set()
    for row_idx, line in enumerate(RAW_MAP):
        if row_idx >= ROWS:
            break
        for col_idx, ch in enumerate(line):
            if col_idx >= COLS:
                break
            if ch == 'W':
                walls.add((col_idx, row_idx))
    return walls


def _find_char(ch: str) -> Tuple[int, int]:
    for row_idx, line in enumerate(RAW_MAP):
        for col_idx, c in enumerate(line):
            if c == ch:
                return (col_idx, row_idx)
    return (1, 1)


# ---------------------------------------------------------------------------
# ─── SimulationEnv ──────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

class SimulationEnv:
    """
    Manages the 2-D grid world: static walls, dynamic obstacles,
    start/target positions, and all low-level rendering of the map.
    """

    def __init__(self) -> None:
        self.cols: int = COLS
        self.rows: int = ROWS
        self.cell: int = CELL

        self.static_walls: Set[Tuple[int, int]] = _build_wall_set()
        self.dynamic_obstacles: Set[Tuple[int, int]] = set()

        self.start:  Tuple[int, int] = _find_char('S')
        self.target: Tuple[int, int] = _find_char('T')

        # Pre-build border walls
        for c in range(self.cols):
            self.static_walls.add((c, 0))
            self.static_walls.add((c, self.rows - 1))
        for r in range(self.rows):
            self.static_walls.add((0, r))
            self.static_walls.add((COLS - 1, r))

    # ── Helpers ─────────────────────────────────────────────────────────────

    def is_blocked(self, col: int, row: int) -> bool:
        """Return True if the cell is impassable."""
        return (col, row) in self.static_walls or (col, row) in self.dynamic_obstacles

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.cols and 0 <= row < self.rows

    def pixel_to_grid(self, px: int, py: int) -> Tuple[int, int]:
        return px // self.cell, py // self.cell

    def grid_to_pixel_center(self, col: int, row: int) -> Tuple[float, float]:
        return (col * self.cell + self.cell / 2,
                row * self.cell + self.cell / 2)

    def toggle_obstacle(self, col: int, row: int, add: bool) -> bool:
        """
        Add or remove a dynamic obstacle.
        Returns True if the cell set changed (ignore start/target/wall).
        """
        if (col, row) in self.static_walls:
            return False
        if (col, row) == self.start or (col, row) == self.target:
            return False
        if not self.in_bounds(col, row):
            return False
        if add:
            if (col, row) not in self.dynamic_obstacles:
                self.dynamic_obstacles.add((col, row))
                return True
        else:
            if (col, row) in self.dynamic_obstacles:
                self.dynamic_obstacles.discard((col, row))
                return True
        return False

    @property
    def all_blocked(self) -> Set[Tuple[int, int]]:
        return self.static_walls | self.dynamic_obstacles

    # ── Rendering ───────────────────────────────────────────────────────────

    def draw(self, surface: pygame.Surface) -> None:
        """Render the entire grid area."""
        surface.fill(C_BG, (0, 0, GRID_W, SCREEN_H))

        # Draw cells
        for row in range(self.rows):
            for col in range(self.cols):
                rect = pygame.Rect(col * CELL, row * CELL, CELL, CELL)

                if (col, row) in self.static_walls:
                    pygame.draw.rect(surface, C_WALL, rect)
                    pygame.draw.rect(surface, C_WALL_EDGE, rect, 1)
                elif (col, row) in self.dynamic_obstacles:
                    pygame.draw.rect(surface, C_OBSTACLE, rect)
                    pygame.draw.rect(surface, C_OBS_EDGE, rect, 1)
                else:
                    # Grid lines only for open cells
                    pygame.draw.rect(surface, C_GRID, rect, 1)

        # Start marker
        sc, sr = self.start
        s_rect = pygame.Rect(sc * CELL + 2, sr * CELL + 2, CELL - 4, CELL - 4)
        pygame.draw.rect(surface, C_START, s_rect, border_radius=6)
        pygame.draw.rect(surface, (200, 255, 220), s_rect, 2, border_radius=6)

        # Target marker (pulsing ring drawn by Simulation)
        tc, tr = self.target
        t_rect = pygame.Rect(tc * CELL + 2, tr * CELL + 2, CELL - 4, CELL - 4)
        pygame.draw.rect(surface, C_TARGET, t_rect, border_radius=6)
        pygame.draw.rect(surface, (255, 240, 120), t_rect, 2, border_radius=6)


# ---------------------------------------------------------------------------
# ─── PathFinder (A*) ────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

class PathFinder:
    """
    Implements the A* search algorithm on the SimulationEnv grid.
    Uses Manhattan distance as the heuristic.
    Allows 8-directional movement (diagonal cost = √2).
    """

    def __init__(self, env: SimulationEnv) -> None:
        self.env = env

    # ── Heuristic ───────────────────────────────────────────────────────────

    @staticmethod
    def _heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        # Octile distance – works well with 8-directional movement
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    # ── Neighbours ──────────────────────────────────────────────────────────

    def _neighbours(self, node: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
        cx, cy = node
        results: List[Tuple[Tuple[int, int], float]] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = cx + dx, cy + dy
                if not self.env.in_bounds(nx, ny):
                    continue
                if self.env.is_blocked(nx, ny):
                    continue
                # Prevent diagonal corner-cutting
                if dx != 0 and dy != 0:
                    if self.env.is_blocked(cx + dx, cy) or self.env.is_blocked(cx, cy + dy):
                        continue
                cost = math.sqrt(2) if (dx != 0 and dy != 0) else 1.0
                results.append(((nx, ny), cost))
        return results

    # ── Search ──────────────────────────────────────────────────────────────

    def find_path(
        self,
        start: Tuple[int, int],
        goal:  Tuple[int, int],
    ) -> Optional[List[Tuple[int, int]]]:
        """
        Returns the list of grid cells (inclusive of start and goal)
        forming the optimal path, or None if no path exists.
        """
        if self.env.is_blocked(*goal):
            return None

        # Priority queue: (f, tie-breaker, node)
        counter = 0
        open_heap: List[Tuple[float, int, Tuple[int, int]]] = []
        heapq.heappush(open_heap, (0.0, counter, start))

        g_score: Dict[Tuple[int, int], float] = {start: 0.0}
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        closed: Set[Tuple[int, int]] = set()

        while open_heap:
            _, _, current = heapq.heappop(open_heap)

            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                # Reconstruct path
                path: List[Tuple[int, int]] = []
                node: Optional[Tuple[int, int]] = current
                while node is not None:
                    path.append(node)
                    node = came_from.get(node)
                path.reverse()
                return path

            for neighbour, move_cost in self._neighbours(current):
                if neighbour in closed:
                    continue
                tentative_g = g_score[current] + move_cost
                if tentative_g < g_score.get(neighbour, float('inf')):
                    g_score[neighbour] = tentative_g
                    f = tentative_g + self._heuristic(neighbour, goal)
                    counter += 1
                    heapq.heappush(open_heap, (f, counter, neighbour))
                    came_from[neighbour] = current

        return None   # No path found

    # ── Path draw ───────────────────────────────────────────────────────────

    def draw_path(
        self,
        surface: pygame.Surface,
        path: List[Tuple[int, int]],
        env: SimulationEnv,
    ) -> None:
        if len(path) < 2:
            return
        pts = [env.grid_to_pixel_center(c, r) for c, r in path]
        # Glow / shadow
        pygame.draw.lines(surface, C_PATH_FADE, False,
                          [(int(p[0]), int(p[1])) for p in pts], 6)
        pygame.draw.lines(surface, C_PATH, False,
                          [(int(p[0]), int(p[1])) for p in pts], 2)
        # Dots at nodes
        for i, pt in enumerate(pts):
            r = 4 if i in (0, len(pts) - 1) else 2
            pygame.draw.circle(surface, C_PATH, (int(pt[0]), int(pt[1])), r)


# ---------------------------------------------------------------------------
# ─── Sensor (LIDAR) ─────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

class Sensor:
    """
    Simulates a 360° LIDAR array with rays cast at configurable angles.
    Stores the last scan results for querying by the Vehicle controller.
    """

    def __init__(self, env: SimulationEnv, max_range_cells: float = SENSOR_RANGE) -> None:
        self.env = env
        self.max_range: float = max_range_cells * CELL   # in pixels
        self.angles_deg: List[float] = list(RAY_ANGLES_DEG)
        # Latest results: angle → distance (in cells, -1 = no hit)
        self.readings: Dict[float, float] = {a: -1.0 for a in self.angles_deg}
        # Hit points in pixel coordinates
        self.hit_points: Dict[float, Optional[Tuple[float, float]]] = {}

    # ── Cast a single ray ───────────────────────────────────────────────────

    def _cast_ray(
        self,
        px: float,
        py: float,
        angle_deg: float,
    ) -> Tuple[float, Optional[Tuple[float, float]]]:
        """
        Walk along the ray in small steps.
        Returns (distance_in_cells, hit_pixel_or_None).
        """
        rad = math.radians(angle_deg)
        dx = math.cos(rad)
        dy = -math.sin(rad)   # Pygame Y is inverted

        step = 2.0   # pixel step size (smaller → more accurate but slower)
        x, y = px, py

        for _ in range(int(self.max_range / step)):
            x += dx * step
            y += dy * step
            col = int(x // CELL)
            row = int(y // CELL)
            if not self.env.in_bounds(col, row):
                dist = math.hypot(x - px, y - py) / CELL
                return dist, (x, y)
            if self.env.is_blocked(col, row):
                dist = math.hypot(x - px, y - py) / CELL
                return dist, (x, y)

        return -1.0, None   # ray reached max range without hitting anything

    # ── Full scan ───────────────────────────────────────────────────────────

    def scan(self, px: float, py: float) -> None:
        """Run all rays from vehicle pixel position (px, py)."""
        for angle in self.angles_deg:
            dist, hit = self._cast_ray(px, py, angle)
            self.readings[angle] = dist
            self.hit_points[angle] = hit

    # ── Query helpers ───────────────────────────────────────────────────────

    def front_distance(self, heading_deg: float) -> float:
        """Return sensor reading closest to the vehicle's heading."""
        # Find the angle in our set closest to heading
        best = min(self.angles_deg, key=lambda a: abs((a - heading_deg + 360) % 360))
        d = self.readings[best]
        return d if d >= 0 else float('inf')

    @property
    def any_hazard(self) -> bool:
        return any(
            0 < d < SAFETY_THRESHOLD for d in self.readings.values()
        )

    @property
    def status_text(self) -> str:
        if any(0 < d < SAFETY_THRESHOLD for d in self.readings.values()):
            return "HAZARD"
        if any(0 < d < SENSOR_RANGE * 0.5 for d in self.readings.values()):
            return "WARNING"
        return "CLEAR"

    # ── Rendering ───────────────────────────────────────────────────────────

    def draw(self, surface: pygame.Surface, px: float, py: float) -> None:
        for angle in self.angles_deg:
            dist = self.readings[angle]
            hit  = self.hit_points.get(angle)

            if dist < 0:
                # No hit – draw full-range ghost line
                rad = math.radians(angle)
                ex = px + math.cos(rad) * self.max_range
                ey = py - math.sin(rad) * self.max_range
                color = C_SENSOR_OK
                end = (int(ex), int(ey))
            else:
                end = (int(hit[0]), int(hit[1])) if hit else (int(px), int(py))
                if dist < SAFETY_THRESHOLD:
                    color = C_SENSOR_HIT
                elif dist < SENSOR_RANGE * 0.6:
                    color = C_SENSOR_WARN
                else:
                    color = C_SENSOR_OK

            # Thin transparent-looking line (alpha sim via dim colour)
            dim_color = tuple(int(c * 0.35) for c in color)
            pygame.draw.line(surface, dim_color, (int(px), int(py)), end, 1)
            if hit:
                pygame.draw.circle(surface, color, end, 3)


# ---------------------------------------------------------------------------
# ─── Vehicle ────────────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

class Vehicle:
    """
    Autonomous vehicle that follows an A* path, reacts to sensor data,
    and exposes telemetry for the HUD.
    """

    def __init__(self, env: SimulationEnv) -> None:
        self.env      = env
        self.sensor   = Sensor(env)
        self.pathfinder = PathFinder(env)

        # Pixel position (float for smooth movement)
        cx, cy = env.grid_to_pixel_center(*env.start)
        self.px: float = float(cx)
        self.py: float = float(cy)

        # Movement state
        self.speed: float   = VEHICLE_SPEED
        self.heading: float = 0.0     # degrees, 0 = right
        self.halted: bool   = False

        # Path state
        self.path: List[Tuple[int, int]] = []
        self.path_index: int = 0
        self.reached_target: bool = False

        # Recalculation throttle
        self._last_recalc: float = 0.0

        # Visual angle (smooth rotation)
        self._draw_angle: float = 0.0

        # Calculate initial path
        self._recalculate_path()

    # ── Grid position ────────────────────────────────────────────────────────

    @property
    def grid_pos(self) -> Tuple[int, int]:
        return int(self.px // CELL), int(self.py // CELL)

    # ── Pathfinding ─────────────────────────────────────────────────────────

    def _recalculate_path(self) -> None:
        now = time.monotonic()
        if now - self._last_recalc < RECALC_COOLDOWN:
            return
        self._last_recalc = now

        gp = self.grid_pos
        path = self.pathfinder.find_path(gp, self.env.target)
        if path and len(path) > 1:
            self.path = path
            self.path_index = 1   # skip current cell (index 0)
            self.halted = False
        else:
            self.path = []
            self.halted = True   # no path available

    def notify_obstacle_placed(self) -> None:
        """Called by Simulation when a dynamic obstacle is placed."""
        # Check if the path is now blocked
        blocked_cells = self.env.all_blocked
        path_blocked = any(cell in blocked_cells for cell in self.path)
        if path_blocked or not self.path:
            self._recalculate_path()

    # ── Update (called every frame) ─────────────────────────────────────────

    def update(self) -> None:
        if self.reached_target:
            return

        # ── LIDAR scan ──────────────────────────────────────────────────────
        self.sensor.scan(self.px, self.py)

        # ── Front-sensor safety check ────────────────────────────────────────
        front_dist = self.sensor.front_distance(self.heading)
        if 0 < front_dist < SAFETY_THRESHOLD:
            self.halted = True
            self._recalculate_path()
            return

        self.halted = False

        # ── Follow path ──────────────────────────────────────────────────────
        if not self.path or self.path_index >= len(self.path):
            self._recalculate_path()
            return

        target_cell = self.path[self.path_index]
        tx, ty = self.env.grid_to_pixel_center(*target_cell)

        # Direction vector to next waypoint
        dx = tx - self.px
        dy = ty - self.py
        dist = math.hypot(dx, dy)

        if dist < 2.0:
            # Arrived at waypoint
            self.px, self.py = tx, ty
            self.path_index += 1

            if target_cell == self.env.target:
                self.reached_target = True
                self.halted = True
                return
        else:
            # Move towards waypoint
            step = min(self.speed, dist)
            self.px += (dx / dist) * step
            self.py += (dy / dist) * step
            self.heading = math.degrees(math.atan2(-dy, dx))

        # Smooth angle interpolation
        angle_diff = (self.heading - self._draw_angle + 360) % 360
        if angle_diff > 180:
            angle_diff -= 360
        self._draw_angle += angle_diff * 0.15

    # ── Telemetry ────────────────────────────────────────────────────────────

    @property
    def distance_to_target(self) -> float:
        tc, tr = self.env.target
        tx, ty = self.env.grid_to_pixel_center(tc, tr)
        return math.hypot(tx - self.px, ty - self.py) / CELL

    @property
    def effective_speed(self) -> float:
        return 0.0 if self.halted else self.speed

    # ── Rendering ───────────────────────────────────────────────────────────

    def draw(self, surface: pygame.Surface) -> None:
        # Sensor rays
        self.sensor.draw(surface, self.px, self.py)

        # Vehicle body – small arrow/chevron shape
        rad = math.radians(self._draw_angle)
        size = CELL * 0.48

        # Define a chevron in local space (pointing right)
        local_pts = [
            ( size * 0.9,  0),
            (-size * 0.5,  size * 0.55),
            (-size * 0.2,  0),
            (-size * 0.5, -size * 0.55),
        ]

        def rotate_pt(lx: float, ly: float) -> Tuple[int, int]:
            rx = lx * math.cos(rad) - ly * math.sin(rad)
            ry = lx * math.sin(rad) + ly * math.cos(rad)
            return (int(self.px + rx), int(self.py + ry))

        pts = [rotate_pt(lx, ly) for lx, ly in local_pts]

        # Glow ring
        pygame.draw.circle(surface, (20, 60, 120), (int(self.px), int(self.py)), int(size) + 4)
        # Body
        pygame.draw.polygon(surface, C_VEHICLE, pts)
        pygame.draw.polygon(surface, C_VEHICLE_HL, pts, 2)
        # Centre dot
        pygame.draw.circle(surface, C_VEHICLE_HL, (int(self.px), int(self.py)), 3)


# ---------------------------------------------------------------------------
# ─── HUD ────────────────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

class HUD:
    """
    Renders the telemetry sidebar and on-screen labels.
    """

    def __init__(self) -> None:
        pygame.font.init()
        self._font_big   = pygame.font.SysFont("Consolas", 18, bold=True)
        self._font_med   = pygame.font.SysFont("Consolas", 14)
        self._font_small = pygame.font.SysFont("Consolas", 12)
        self._font_title = pygame.font.SysFont("Consolas", 22, bold=True)

    def _text(
        self,
        surface: pygame.Surface,
        text: str,
        x: int,
        y: int,
        color=C_TEXT_PRI,
        font: Optional[pygame.font.Font] = None,
    ) -> int:
        """Render text and return new Y offset."""
        fnt = font or self._font_med
        img = fnt.render(text, True, color)
        surface.blit(img, (x, y))
        return y + img.get_height() + 4

    def draw(
        self,
        surface: pygame.Surface,
        vehicle: Vehicle,
        paused: bool,
        fps: float,
        elapsed: float,
    ) -> None:
        # ── Sidebar background ──────────────────────────────────────────────
        sidebar = pygame.Rect(GRID_W, 0, HUD_W, SCREEN_H)
        pygame.draw.rect(surface, C_HUD_BG, sidebar)
        pygame.draw.line(surface, C_ACCENT, (GRID_W, 0), (GRID_W, SCREEN_H), 2)

        x0 = GRID_W + 12
        y  = 16

        # Title
        y = self._text(surface, "AUTONAV", x0, y, C_ACCENT, self._font_title)
        y = self._text(surface, "Simulation v1.0", x0, y, C_TEXT_SEC, self._font_small)
        y += 8
        pygame.draw.line(surface, C_HUD_LINE, (GRID_W + 8, y), (SCREEN_W - 8, y))
        y += 10

        # ── Telemetry ───────────────────────────────────────────────────────
        def section(title: str) -> None:
            nonlocal y
            y = self._text(surface, title, x0, y, C_TEXT_SEC, self._font_small)
            y += 2

        gc, gr = vehicle.grid_pos
        section("POSITION")
        y = self._text(surface, f"  Grid   ({gc:2d}, {gr:2d})", x0, y)
        y = self._text(surface, f"  Px   ({vehicle.px:5.1f}, {vehicle.py:5.1f})", x0, y)
        y += 4

        section("VELOCITY")
        spd = vehicle.effective_speed
        spd_color = C_TEXT_WARN if vehicle.halted else C_TEXT_OK
        y = self._text(surface, f"  Speed  {spd:.2f} px/fr", x0, y, spd_color)
        y += 4

        section("NAVIGATION")
        dtarget = vehicle.distance_to_target
        y = self._text(surface, f"  Dist   {dtarget:.2f} cells", x0, y)
        nodes_left = max(0, len(vehicle.path) - vehicle.path_index)
        y = self._text(surface, f"  Nodes  {nodes_left}", x0, y)
        y += 4

        section("SENSOR STATUS")
        s_txt = vehicle.sensor.status_text
        s_col = (C_TEXT_ERR if s_txt == "HAZARD"
                 else C_TEXT_WARN if s_txt == "WARNING"
                 else C_TEXT_OK)
        y = self._text(surface, f"  {s_txt}", x0, y, s_col, self._font_big)
        y += 4

        # Individual ray readings
        section("LIDAR READINGS")
        angle_names = {
            0: "E ", 45: "NE", 90: "N ", 135: "NW",
            180: "W ", 225: "SW", 270: "S ", 315: "SE",
        }
        for ang in RAY_ANGLES_DEG:
            d = vehicle.sensor.readings[ang]
            name = angle_names.get(ang, f"{ang:3d}")
            if d < 0:
                val_str = " ----"
                col = C_TEXT_SEC
            else:
                val_str = f"{d:5.2f}c"
                col = (C_TEXT_ERR if d < SAFETY_THRESHOLD
                       else C_TEXT_WARN if d < SENSOR_RANGE * 0.6
                       else C_TEXT_OK)
            y = self._text(surface, f"  {name}: {val_str}", x0, y, col, self._font_small)

        y += 4
        pygame.draw.line(surface, C_HUD_LINE, (GRID_W + 8, y), (SCREEN_W - 8, y))
        y += 8

        # ── Status flags ────────────────────────────────────────────────────
        section("SYSTEM")
        halted_col = C_TEXT_ERR if vehicle.halted else C_TEXT_OK
        halted_str = "HALTED" if vehicle.halted else "MOVING"
        y = self._text(surface, f"  Vehicle  {halted_str}", x0, y, halted_col)

        if vehicle.reached_target:
            y = self._text(surface, "  TARGET REACHED!", x0, y, C_TEXT_OK, self._font_big)
        elif not vehicle.path:
            y = self._text(surface, "  NO PATH FOUND", x0, y, C_TEXT_ERR)

        paused_col = C_TEXT_WARN if paused else C_TEXT_SEC
        y = self._text(surface, f"  Paused   {'YES' if paused else 'no'}", x0, y, paused_col)
        y = self._text(surface, f"  FPS      {fps:.1f}", x0, y, C_TEXT_SEC)
        y = self._text(surface, f"  Time     {elapsed:.1f}s", x0, y, C_TEXT_SEC)

        pygame.draw.line(surface, C_HUD_LINE, (GRID_W + 8, y), (SCREEN_W - 8, y))
        y += 8

        # ── Key hints ───────────────────────────────────────────────────────
        section("CONTROLS")
        hints = [
            ("LClick", "Place obstacle"),
            ("RClick", "Clear obstacle"),
            ("SPACE",  "Pause / Resume"),
            ("R",      "Reset vehicle"),
            ("ESC/Q",  "Quit"),
        ]
        for key, desc in hints:
            y = self._text(surface, f"  {key:<7} {desc}", x0, y, C_TEXT_SEC, self._font_small)

        # ── On-grid labels ───────────────────────────────────────────────────
        # Start label
        sc, sr = vehicle.env.start
        sx = sc * CELL
        sy = sr * CELL - 18
        lbl = self._font_small.render("START", True, C_START)
        surface.blit(lbl, (sx + 2, max(2, sy)))

        # Target label
        tc, tr = vehicle.env.target
        tx2 = tc * CELL
        ty2 = tr * CELL - 18
        lbl2 = self._font_small.render("TARGET", True, C_TARGET)
        surface.blit(lbl2, (tx2 + 2, max(2, ty2)))


# ---------------------------------------------------------------------------
# ─── Simulation (Main Loop) ─────────────────────────────────────────────────
# ---------------------------------------------------------------------------

class Simulation:
    """
    Orchestrates the event loop, rendering order, and user interaction.
    """

    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        pygame.display.set_caption("AutoNav – Autonomous Navigation Simulation")

        self.clock  = pygame.time.Clock()
        self.env     = SimulationEnv()
        self.vehicle = Vehicle(self.env)
        self.hud     = HUD()

        self.paused  = False
        self.running = True
        self._start_time = time.monotonic()
        self._fps_display: float = 0.0

        # Pulse animation for target marker
        self._pulse = 0.0

    # ── Reset ────────────────────────────────────────────────────────────────

    def _reset_vehicle(self) -> None:
        self.vehicle = Vehicle(self.env)

    # ── Events ───────────────────────────────────────────────────────────────

    def _handle_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self._reset_vehicle()

        # Continuous mouse button held
        mb = pygame.mouse.get_pressed()
        if mb[0] or mb[2]:
            mx, my = pygame.mouse.get_pos()
            if mx < GRID_W:   # only within grid area
                col, row = self.env.pixel_to_grid(mx, my)
                changed = self.env.toggle_obstacle(col, row, add=bool(mb[0]))
                if changed:
                    self.vehicle.notify_obstacle_placed()

    # ── Target pulse ─────────────────────────────────────────────────────────

    def _draw_target_pulse(self) -> None:
        self._pulse = (self._pulse + 0.05) % (2 * math.pi)
        tc, tr = self.env.target
        cx2 = tc * CELL + CELL // 2
        cy2 = tr * CELL + CELL // 2
        pulse_r = int(CELL * 0.7 + math.sin(self._pulse) * 6)
        alpha_surf = pygame.Surface((pulse_r * 2 + 4, pulse_r * 2 + 4), pygame.SRCALPHA)
        pygame.draw.circle(alpha_surf, (255, 215, 0, 60),
                           (pulse_r + 2, pulse_r + 2), pulse_r)
        self.screen.blit(alpha_surf, (cx2 - pulse_r - 2, cy2 - pulse_r - 2))

    # ── Main loop ────────────────────────────────────────────────────────────

    def run(self) -> None:
        while self.running:
            dt = self.clock.tick(FPS)
            self._fps_display = self.clock.get_fps()

            self._handle_events()

            if not self.paused:
                self.vehicle.update()

            # ── Draw ──────────────────────────────────────────────────────────
            self.env.draw(self.screen)

            # Path
            if self.vehicle.path:
                self.vehicle.pathfinder.draw_path(
                    self.screen, self.vehicle.path, self.env
                )

            # Target pulse effect
            self._draw_target_pulse()

            # Vehicle (sensors + body)
            self.vehicle.draw(self.screen)

            # HUD
            elapsed = time.monotonic() - self._start_time
            self.hud.draw(
                self.screen,
                self.vehicle,
                self.paused,
                self._fps_display,
                elapsed,
            )

            # Pause overlay
            if self.paused:
                overlay = pygame.Surface((GRID_W, SCREEN_H), pygame.SRCALPHA)
                overlay.fill((0, 0, 0, 80))
                self.screen.blit(overlay, (0, 0))
                font = pygame.font.SysFont("Consolas", 48, bold=True)
                lbl  = font.render("PAUSED", True, C_TEXT_WARN)
                self.screen.blit(lbl, (GRID_W // 2 - lbl.get_width() // 2,
                                       SCREEN_H // 2 - lbl.get_height() // 2))

            pygame.display.flip()

        pygame.quit()
        sys.exit(0)


# ---------------------------------------------------------------------------
# ─── Entry point ────────────────────────────────────────────────────────────
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    sim = Simulation()
    sim.run()
