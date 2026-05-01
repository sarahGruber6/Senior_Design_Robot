import heapq
import math
from typing import List, Tuple, Optional


class OccupancyGrid:
    def __init__(
        self,
        mapbytes: bytearray,
        width: int,
        height: int,
        meters: float,
        robot_radius_mm: float = 350.0,
    ):
        self.width = width
        self.height = height
        self.meters = meters
        self.mm_per_pixel = (meters * 1000) / width

        # BreezySLAM map values are low for obstacles and high for free space.
        # Unknown cells sit near the middle, so demo planning treats them as
        # passable and relies on the robot's live LiDAR stop for safety.
        self._occupancy = bytearray(1 if b < 50 else 0 for b in mapbytes)

        self.inflation_pixels = max(1, int(round(robot_radius_mm / self.mm_per_pixel)))
        self.planner_step_pixels = max(2, int(round(200.0 / self.mm_per_pixel)))
        self._inflation_offsets = [
            (dx, dy)
            for dy in range(-self.inflation_pixels, self.inflation_pixels + 1)
            for dx in range(-self.inflation_pixels, self.inflation_pixels + 1)
            if dx * dx + dy * dy <= self.inflation_pixels * self.inflation_pixels
        ]
        self._collision_cache = {}

    def _raw_occupied(self, x_px: int, y_px: int) -> bool:
        return bool(self._occupancy[y_px * self.width + x_px])

    def is_collision_free_px(self, x_px: int, y_px: int) -> bool:
        if not (0 <= x_px < self.width and 0 <= y_px < self.height):
            return False

        key = (x_px, y_px)
        cached = self._collision_cache.get(key)
        if cached is not None:
            return cached

        for dx, dy in self._inflation_offsets:
            nx, ny = x_px + dx, y_px + dy
            if 0 <= nx < self.width and 0 <= ny < self.height and self._raw_occupied(nx, ny):
                self._collision_cache[key] = False
                return False

        self._collision_cache[key] = True
        return True

    def is_collision_free(self, x_mm: float, y_mm: float) -> bool:
        x_px = int(x_mm / self.mm_per_pixel)
        y_px = int(y_mm / self.mm_per_pixel)
        return self.is_collision_free_px(x_px, y_px)

    def is_line_collision_free(
        self,
        start_mm: Tuple[float, float],
        goal_mm: Tuple[float, float],
        step_mm: float = 100.0,
    ) -> bool:
        distance = math.dist(start_mm, goal_mm)
        steps = max(1, int(math.ceil(distance / step_mm)))

        for i in range(steps + 1):
            t = i / steps
            x = start_mm[0] + (goal_mm[0] - start_mm[0]) * t
            y = start_mm[1] + (goal_mm[1] - start_mm[1]) * t
            if not self.is_collision_free(x, y):
                return False
        return True


class AStarPathfinder:
    def __init__(self, grid: OccupancyGrid):
        self.grid = grid

    def _heuristic(self, start: Tuple[int, int], goal: Tuple[int, int]) -> float:
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        return math.sqrt(dx**2 + dy**2)

    def _get_cost(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        dx = abs(b[0] - a[0])
        dy = abs(b[1] - a[1])
        return math.sqrt(2) if (dx == 1 and dy == 1) else 1.0

    def plan(
        self,
        start_mm: Tuple[float, float],
        goal_mm: Tuple[float, float],
        max_iterations: int = 80000,
    ) -> Optional[List[Tuple[float, float]]]:
        if self.grid.is_line_collision_free(start_mm, goal_mm):
            return [start_mm, goal_mm]

        step = self.grid.planner_step_pixels
        cells_w = math.ceil(self.grid.width / step)
        cells_h = math.ceil(self.grid.height / step)

        def mm_to_cell(point_mm: Tuple[float, float]) -> Tuple[int, int]:
            x_px = int(point_mm[0] / self.grid.mm_per_pixel)
            y_px = int(point_mm[1] / self.grid.mm_per_pixel)
            return (round(x_px / step), round(y_px / step))

        def cell_to_px(cell: Tuple[int, int]) -> Tuple[int, int]:
            return (
                max(0, min(self.grid.width - 1, int(cell[0] * step))),
                max(0, min(self.grid.height - 1, int(cell[1] * step))),
            )

        def cell_to_mm(cell: Tuple[int, int]) -> Tuple[float, float]:
            x_px, y_px = cell_to_px(cell)
            return (x_px * self.grid.mm_per_pixel, y_px * self.grid.mm_per_pixel)

        def cell_free(cell: Tuple[int, int]) -> bool:
            if not (0 <= cell[0] < cells_w and 0 <= cell[1] < cells_h):
                return False
            return self.grid.is_collision_free_px(*cell_to_px(cell))

        def nearest_free(cell: Tuple[int, int], radius_cells: int = 8) -> Optional[Tuple[int, int]]:
            if cell_free(cell):
                return cell
            for r in range(1, radius_cells + 1):
                best = None
                best_dist = float("inf")
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        if max(abs(dx), abs(dy)) != r:
                            continue
                        candidate = (cell[0] + dx, cell[1] + dy)
                        if cell_free(candidate):
                            dist = dx * dx + dy * dy
                            if dist < best_dist:
                                best = candidate
                                best_dist = dist
                if best is not None:
                    return best
            return None

        start_cell = nearest_free(mm_to_cell(start_mm))
        goal_cell = nearest_free(mm_to_cell(goal_mm))

        if start_cell is None or goal_cell is None:
            return None
        if start_cell == goal_cell:
            return [start_mm, goal_mm]

        open_set = [(0, start_cell)]
        closed_set = set()
        g_score = {start_cell: 0}
        came_from = {}
        iteration = 0

        while open_set and iteration < max_iterations:
            iteration += 1
            _f, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            closed_set.add(current)

            if current == goal_cell:
                cell_path = []
                node = goal_cell
                while node in came_from:
                    cell_path.append(node)
                    node = came_from[node]
                cell_path.append(start_cell)
                cell_path.reverse()

                cell_path = self._shortcut_cells(cell_path, cell_to_mm)
                path = self._simplify_cells(cell_path, cell_to_mm)
                path[0] = start_mm
                path[-1] = goal_mm
                return path

            for neighbor in self._neighbors(current, cells_w, cells_h, cell_free):
                if neighbor in closed_set:
                    continue

                tentative_g = g_score[current] + self._get_cost(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = self._heuristic(neighbor, goal_cell)
                    f = tentative_g + h
                    heapq.heappush(open_set, (f, neighbor))

        return None

    def _neighbors(self, cell, cells_w, cells_h, cell_free):
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (cell[0] + dx, cell[1] + dy)
                if 0 <= neighbor[0] < cells_w and 0 <= neighbor[1] < cells_h and cell_free(neighbor):
                    yield neighbor

    def _shortcut_cells(self, cells, cell_to_mm):
        if len(cells) <= 2:
            return cells

        shortcut = [cells[0]]
        anchor = 0

        while anchor < len(cells) - 1:
            farthest = len(cells) - 1
            while farthest > anchor + 1:
                if self.grid.is_line_collision_free(cell_to_mm(cells[anchor]), cell_to_mm(cells[farthest])):
                    break
                farthest -= 1
            shortcut.append(cells[farthest])
            anchor = farthest

        return shortcut

    def _simplify_cells(self, cells, cell_to_mm, spacing_mm: float = 450.0):
        if len(cells) <= 2:
            return [cell_to_mm(c) for c in cells]

        keep = [cells[0]]
        last_dir = None
        last_kept_mm = cell_to_mm(cells[0])

        for i in range(1, len(cells) - 1):
            prev_cell = cells[i - 1]
            cell = cells[i]
            next_cell = cells[i + 1]
            direction = (
                max(-1, min(1, next_cell[0] - prev_cell[0])),
                max(-1, min(1, next_cell[1] - prev_cell[1])),
            )
            point_mm = cell_to_mm(cell)
            dist_since_keep = math.dist(last_kept_mm, point_mm)

            if last_dir is not None and direction != last_dir:
                keep.append(cell)
                last_kept_mm = point_mm
            elif dist_since_keep >= spacing_mm:
                keep.append(cell)
                last_kept_mm = point_mm

            last_dir = direction

        keep.append(cells[-1])
        return [cell_to_mm(c) for c in keep]
