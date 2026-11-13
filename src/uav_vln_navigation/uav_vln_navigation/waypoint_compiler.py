#!/usr/bin/env python3

import heapq
import json
import math
import time
from pathlib import Path


class SafetyError(ValueError):
    pass


def load_config(path):

    config = json.loads(
        Path(path).read_text(
            encoding="utf-8"
        )
    )

    bounds = config["bounds"]
    fence = config["geofence"]
    limits = config["limits"]

    for section in (
        bounds,
        fence,
        limits,
    ):

        for key, value in section.items():

            if not isinstance(
                value,
                (int, float),
            ) or not math.isfinite(value):

                raise SafetyError(
                    f"Invalid config: {key}"
                )

    for axis in ("x", "y"):

        if not (
            bounds[f"{axis}min"]
            <= fence[f"{axis}min"]
            < fence[f"{axis}max"]
            <= bounds[f"{axis}max"]
        ):

            raise SafetyError(
                "Invalid map/geofence bounds"
            )

    if not (
        0 < fence["zmin"] < fence["zmax"]
    ):

        raise SafetyError(
            "Invalid altitude bounds"
        )

    if not (
        0 < config["resolution"] <= 1
    ):

        raise SafetyError(
            "Invalid map resolution"
        )

    if limits["clearance"] <= 0:

        raise SafetyError(
            "Invalid obstacle clearance"
        )

    for obstacle in config["obstacles"]:

        if not all(
            math.isfinite(
                float(obstacle[key])
            )

            for key in (
                "xmin", "xmax",
                "ymin", "ymax",
                "height",
            )
        ):

            raise SafetyError(
                "Invalid obstacle coordinates"
            )

        if not (
            obstacle["xmin"]
            < obstacle["xmax"]

            and obstacle["ymin"]
            < obstacle["ymax"]

            and obstacle["height"] > 0
        ):

            raise SafetyError(
                "Invalid obstacle geometry"
            )

    return config


def finite_point(point):

    return (
        len(point) == 3

        and all(
            math.isfinite(float(v))
            for v in point
        )
    )


def rectangle_distance(x, y, rectangle):

    dx = max(
        rectangle["xmin"] - x,
        0.0,
        x - rectangle["xmax"],
    )

    dy = max(
        rectangle["ymin"] - y,
        0.0,
        y - rectangle["ymax"],
    )

    return math.hypot(dx, dy)


def segment_intersects_rectangle(
    start,
    end,
    rectangle,
    margin,
):

    """
    Exact 2-D segment versus expanded
    axis-aligned rectangle intersection.

    Uses the slab method:
    intersect parameter intervals on x and y.
    """

    t_min = 0.0
    t_max = 1.0

    for index, axis in enumerate(
        ("x", "y")
    ):

        low = (
            rectangle[f"{axis}min"]
            - margin
        )

        high = (
            rectangle[f"{axis}max"]
            + margin
        )

        origin = float(start[index])

        direction = (
            float(end[index])
            - origin
        )

        if abs(direction) < 1e-12:

            if origin < low or origin > high:
                return False

            continue

        t1 = (
            low - origin
        ) / direction

        t2 = (
            high - origin
        ) / direction

        if t1 > t2:
            t1, t2 = t2, t1

        t_min = max(
            t_min,
            t1,
        )

        t_max = min(
            t_max,
            t2,
        )

        if t_min > t_max:
            return False

    return True


class SafetyChecker:

    def __init__(
        self,
        config,
        target_guard=None,
    ):

        self.config = config

        self.fence = config[
            "geofence"
        ]

        self.limits = config[
            "limits"
        ]

        self.obstacles = config[
            "obstacles"
        ]

        self.target_guard = target_guard

    def check_point(
        self,
        point,
        allow_ground=False,
        extra_margin=0.0,
    ):

        if not finite_point(point):

            raise SafetyError(
                "Waypoint contains NaN, "
                "infinity or invalid dimensions"
            )

        x, y, z = map(
            float,
            point,
        )

        fence = self.fence

        if not (
            fence["xmin"]
            <= x
            <= fence["xmax"]
        ):

            raise SafetyError(
                "x outside geofence"
            )

        if not (
            fence["ymin"]
            <= y
            <= fence["ymax"]
        ):

            raise SafetyError(
                "y outside geofence"
            )

        zmin = (
            -0.3
            if allow_ground
            else fence["zmin"]
        )

        if not (
            zmin <= z <= fence["zmax"]
        ):

            raise SafetyError(
                "z outside altitude limits"
            )

        margin = (
            self.limits["clearance"]
            + extra_margin
        )

        for obstacle in self.obstacles:

            if segment_intersects_rectangle(
                point,
                point,
                obstacle,
                margin,
            ):

                raise SafetyError(
                    "Waypoint too close "
                    "to known obstacle: "
                    + obstacle["name"]
                )

        if self.target_guard:

            guard = self.target_guard

            tx, ty = guard["surface_xy"]
            nx, ny = guard["normal_xy"]

            signed_distance = (
                (x - tx) * nx
                + (y - ty) * ny
            )

            # Remain on the original
            # observation side of the target.
            if signed_distance > -(
                guard["minimum_distance"]
                + extra_margin
            ):

                raise SafetyError(
                    "Waypoint violates "
                    "visual target approach guard"
                )

        return True

    def check_segment(
        self,
        start,
        end,
        allow_ground=False,
        extra_margin=0.0,
    ):

        self.check_point(
            start,

            allow_ground=allow_ground,

            extra_margin=extra_margin,
        )

        self.check_point(
            end,

            allow_ground=allow_ground,

            extra_margin=extra_margin,
        )

        margin = (
            self.limits["clearance"]
            + extra_margin
        )

        for obstacle in self.obstacles:

            if segment_intersects_rectangle(

                start,
                end,
                obstacle,
                margin,

            ):

                raise SafetyError(
                    "Segment intersects "
                    "expanded obstacle: "
                    + obstacle["name"]
                )

        # Rectangular geofence and target
        # half-plane are convex.
        # Valid endpoints imply the complete
        # straight segment stays inside them.
        return True

    def segment_is_safe(
        self,
        start,
        end,
        extra_margin=0.0,
    ):

        try:

            self.check_segment(
                start,
                end,

                extra_margin=extra_margin,
            )

            return True

        except SafetyError:

            return False

    def check_path(self, points):

        if not points:

            raise SafetyError(
                "Empty path"
            )

        for point in points:

            self.check_point(point)

        for start, end in zip(
            points,
            points[1:],
        ):

            self.check_segment(
                start,
                end,
            )

        return True


class AStarPlanner:

    def __init__(
        self,
        config,
        checker,
    ):

        self.config = config
        self.checker = checker

        self.resolution = float(
            config["resolution"]
        )

        self.bounds = config[
            "bounds"
        ]

        self.xmin = self.bounds[
            "xmin"
        ]

        self.ymin = self.bounds[
            "ymin"
        ]

        self.width = math.ceil(
            (
                self.bounds["xmax"]
                - self.xmin
            )
            / self.resolution
        )

        self.height = math.ceil(
            (
                self.bounds["ymax"]
                - self.ymin
            )
            / self.resolution
        )

        # Additional tracking margin.
        # Not an extra map inflation.
        self.tracking_margin = 0.45

        self.neighbors = [

            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),

            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]

    def point_to_cell(self, point):

        return (

            math.floor(
                (point[0] - self.xmin)
                / self.resolution
            ),

            math.floor(
                (point[1] - self.ymin)
                / self.resolution
            ),
        )

    def cell_to_point(self, cell, altitude):

        ix, iy = cell

        return (

            self.xmin
            + (ix + 0.5)
            * self.resolution,

            self.ymin
            + (iy + 0.5)
            * self.resolution,

            altitude,
        )

    def cell_is_valid(
        self,
        cell,
        altitude,
    ):

        ix, iy = cell

        if not (
            0 <= ix < self.width

            and 0 <= iy < self.height
        ):

            return False

        point = self.cell_to_point(
            cell,
            altitude,
        )

        return self.checker.segment_is_safe(

            point,
            point,

            extra_margin=
                self.tracking_margin,
        )

    def connect_endpoint(
        self,
        point,
        altitude,
    ):

        cx, cy = self.point_to_cell(
            point
        )

        candidates = []

        # Connect the exact endpoint to a
        # nearby valid grid-cell center.
        # Never force an occupied cell free.
        for dx in range(-4, 5):

            for dy in range(-4, 5):

                cell = (
                    cx + dx,
                    cy + dy,
                )

                if not self.cell_is_valid(
                    cell,
                    altitude,
                ):

                    continue

                center = self.cell_to_point(
                    cell,
                    altitude,
                )

                if self.checker.segment_is_safe(

                    point,
                    center,

                    extra_margin=
                        self.tracking_margin,

                ):

                    candidates.append(

                        (
                            math.dist(
                                point,
                                center,
                            ),

                            cell,
                        )
                    )

        if not candidates:

            raise SafetyError(
                "Endpoint cannot connect "
                "to any safe grid cell"
            )

        candidates.sort()

        return candidates[0][1]

    def search(
        self,
        start,
        goal,
    ):

        start = tuple(
            map(float, start)
        )

        goal = tuple(
            map(float, goal)
        )

        self.checker.check_point(
            start,
            extra_margin=
                self.tracking_margin,
        )

        self.checker.check_point(
            goal,
            extra_margin=
                self.tracking_margin,
        )

        if abs(
            start[2] - goal[2]
        ) > 1e-6:

            raise SafetyError(
                "A* requires constant "
                "cruise altitude"
            )

        altitude = start[2]

        # Direct path if safe.
        if self.checker.segment_is_safe(

            start,
            goal,

            extra_margin=
                self.tracking_margin,

        ):

            return self.resample(
                [start, goal]
            )

        start_cell = self.connect_endpoint(
            start,
            altitude,
        )

        goal_cell = self.connect_endpoint(
            goal,
            altitude,
        )

        def heuristic(cell):

            return (
                self.resolution
                * math.hypot(

                    cell[0] - goal_cell[0],

                    cell[1] - goal_cell[1],
                )
            )

        open_set = [

            (
                heuristic(start_cell),
                0.0,
                start_cell,
            )
        ]

        g_cost = {
            start_cell: 0.0,
        }

        parents = {}

        expanded = 0

        found = False

        while open_set:

            _, cost, current = (
                heapq.heappop(open_set)
            )

            if cost > (
                g_cost.get(
                    current,
                    float("inf"),
                ) + 1e-9
            ):

                continue

            if current == goal_cell:

                found = True
                break

            expanded += 1

            if expanded > 100000:

                raise SafetyError(
                    "A* expansion limit reached"
                )

            p0 = self.cell_to_point(
                current,
                altitude,
            )

            for dx, dy in self.neighbors:

                neighbor = (

                    current[0] + dx,

                    current[1] + dy,
                )

                if not self.cell_is_valid(
                    neighbor,
                    altitude,
                ):

                    continue

                # Prevent diagonal corner cutting.
                if dx != 0 and dy != 0:

                    side_a = (
                        current[0] + dx,
                        current[1],
                    )

                    side_b = (
                        current[0],
                        current[1] + dy,
                    )

                    if not (

                        self.cell_is_valid(
                            side_a,
                            altitude,
                        )

                        and self.cell_is_valid(
                            side_b,
                            altitude,
                        )
                    ):

                        continue

                p1 = self.cell_to_point(
                    neighbor,
                    altitude,
                )

                # Check the complete segment.
                if not self.checker.segment_is_safe(

                    p0,
                    p1,

                    extra_margin=
                        self.tracking_margin,

                ):

                    continue

                new_cost = (
                    cost
                    + math.dist(p0, p1)
                )

                if new_cost >= g_cost.get(

                    neighbor,

                    float("inf"),
                ):

                    continue

                g_cost[neighbor] = (
                    new_cost
                )

                parents[neighbor] = (
                    current
                )

                heapq.heappush(

                    open_set,

                    (
                        new_cost
                        + heuristic(neighbor),

                        new_cost,

                        neighbor,
                    ),
                )

        if not found:

            raise SafetyError(
                "No collision-free path exists"
            )

        cells = [goal_cell]

        while cells[-1] != start_cell:

            cells.append(
                parents[cells[-1]]
            )

        cells.reverse()

        points = [start]

        points.extend(

            self.cell_to_point(
                cell,
                altitude,
            )

            for cell in cells
        )

        points.append(goal)

        simplified = self.simplify(
            points
        )

        final_path = self.resample(
            simplified
        )

        # Independent final check.
        self.checker.check_path(
            final_path
        )

        return final_path

    def simplify(self, points):

        result = [points[0]]

        index = 0

        while index < len(points) - 1:

            next_index = (
                len(points) - 1
            )

            while next_index > index:

                if self.checker.segment_is_safe(

                    points[index],

                    points[next_index],

                    extra_margin=
                        self.tracking_margin,

                ):

                    break

                next_index -= 1

            if next_index == index:

                raise SafetyError(
                    "Path simplification failed"
                )

            result.append(
                points[next_index]
            )

            index = next_index

        return result

    def resample(self, points):

        result = [points[0]]

        # Keep corners and subdivide
        # long straight segments.
        max_spacing = 3.0

        for start, end in zip(

            points,
            points[1:],
        ):

            distance = math.dist(
                start,
                end,
            )

            if distance < 1e-9:
                continue

            count = max(

                1,

                math.ceil(
                    distance / max_spacing
                ),
            )

            for index in range(
                1,
                count + 1,
            ):

                ratio = index / count

                result.append(

                    tuple(

                        start[k]
                        + ratio
                        * (
                            end[k] - start[k]
                        )

                        for k in range(3)
                    )
                )

        self.checker.check_path(result)

        return result


def path_length(points):

    return sum(

        math.dist(start, end)

        for start, end in zip(
            points,
            points[1:],
        )
    )


def make_target_guard(
    observation_position,
    target,
):

    if not finite_point(
        observation_position
    ) or not finite_point(target):

        raise SafetyError(
            "Invalid visual target"
        )

    dx = (
        target[0]
        - observation_position[0]
    )

    dy = (
        target[1]
        - observation_position[1]
    )

    distance = math.hypot(
        dx,
        dy,
    )

    if distance < 0.5:

        raise SafetyError(
            "Visual target range too short"
        )

    return {

        "surface_xy": [
            target[0],
            target[1],
        ],

        "normal_xy": [

            dx / distance,

            dy / distance,
        ],

        "minimum_distance": 1.2,
    }


def compile_mission(
    semantic_plan,
    home,
    current,
    visual_target,
    config,
):

    start_time = time.monotonic()

    actions = {

        action["action_type"]: action

        for action in
            semantic_plan["actions"]
    }

    altitude = float(
        actions["takeoff"]["altitude"]
    )

    goto = actions["goto"]

    return_action = actions[
        "return_home"
    ]

    standoff = float(
        goto["standoff_distance"]
    )

    guard = make_target_guard(

        current,

        visual_target,
    )

    nx, ny = guard["normal_xy"]

    tx, ty = guard["surface_xy"]

    approach = (

        tx - standoff * nx,

        ty - standoff * ny,

        altitude,
    )

    home_air = (

        float(home[0]),

        float(home[1]),

        altitude,
    )

    # Use the actual altitude reached
    # by the takeoff action for planning.
    # Ensure the transition to the requested
    # cruise altitude is separately safe.
    start = (

        float(current[0]),

        float(current[1]),

        altitude,
    )

    checker = SafetyChecker(
        config,

        target_guard=guard,
    )

    checker.check_segment(
        current,
        start,
    )

    planner = AStarPlanner(
        config,
        checker,
    )

    # Search the visible-side standoff circle.
    # Do not reduce the required target distance.
    # Both outbound and return paths must be safe.

    outbound = None
    inbound = None
    approach_offset_deg = None
    rejected_approaches = []

    for offset_deg in (
        0,
        -15, 15,
        -30, 30,
        -45, 45,
        -55, 55,
    ):

        angle = math.radians(offset_deg)

        direction_x = (
            nx * math.cos(angle)
            - ny * math.sin(angle)
        )

        direction_y = (
            ny * math.cos(angle)
            + nx * math.sin(angle)
        )

        candidate = (
            tx - standoff * direction_x,
            ty - standoff * direction_y,
            altitude,
        )

        try:

            candidate_outbound = planner.search(
                start,
                candidate,
            )

            candidate_inbound = planner.search(
                candidate,
                home_air,
            )

        except SafetyError as exc:

            rejected_approaches.append({
                "offset_deg": offset_deg,
                "reason": str(exc),
            })

            continue

        approach = candidate
        outbound = candidate_outbound
        inbound = candidate_inbound

        approach_offset_deg = offset_deg

        break

    if outbound is None or inbound is None:

        raise SafetyError(
            "No safe approach found at required "
            f"standoff {standoff:.2f} m: "
            f"{rejected_approaches}"
        )

    # Include current position to first
    # cruise-altitude point in the plan.
    if math.dist(
        current,
        start,
    ) > 0.05:

        outbound.insert(
            0,
            tuple(current),
        )

    checker.check_path(
        outbound
    )

    checker.check_path(
        inbound
    )

    # Landing column must also be clear.
    checker.check_segment(

        home_air,

        (
            home_air[0],
            home_air[1],
            0.0,
        ),

        allow_ground=True,
    )

    lengths = {}

    estimates = {}

    for name, path, action in (

        (
            "outbound",
            outbound,
            goto,
        ),

        (
            "return",
            inbound,
            return_action,
        ),
    ):

        distance = path_length(
            path
        )

        speed = float(
            action["velocity_limit"]
        )

        timeout = float(
            action["timeout"]
        )

        minimum_time = (
            distance / speed
        )

        # First-order necessary condition.
        if minimum_time > timeout - 3.0:

            raise SafetyError(
                f"{name}: timeout shorter "
                "than minimum travel time"
            )

        # Conservative execution estimate
        # for stop-and-go position tracking.
        nominal_speed = (
            0.6 * speed
        )

        estimated_time = (

            distance / nominal_speed

            + 2.0
            * max(
                0,
                len(path) - 1,
            )

            + 3.0
        )

        if estimated_time > timeout:

            raise SafetyError(
                f"{name}: estimated execution "
                "time exceeds requested timeout"
            )

        lengths[name] = (
            distance
        )

        estimates[name] = (
            estimated_time
        )

    yaw = math.atan2(
        ty - approach[1],
        tx - approach[0],
    )

    return {

        "frame_id": "map_enu",

        "target_source": "RGB-D",

        "target_surface": list(
            visual_target
        ),

        "target_guard": guard,

        "approach_point": list(
            approach
        ),

        "approach_offset_deg":
            approach_offset_deg,

        "rejected_approaches":
            rejected_approaches,

        "home_air": list(
            home_air
        ),

        "yaw": yaw,

        "outbound_waypoints": [

            list(point)

            for point in outbound
        ],

        "return_waypoints": [

            list(point)

            for point in inbound
        ],

        "lengths_m": lengths,

        "estimated_times_s": estimates,

        "compile_time_s": (
            time.monotonic()
            - start_time
        ),

        "clearance_m":
            config["limits"][
                "clearance"
            ],

        "planning_tracking_margin_m":
            planner.tracking_margin,
    }
