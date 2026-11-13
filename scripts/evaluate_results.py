#!/usr/bin/env python3

import argparse
import csv
import json
import math
import xml.etree.ElementTree as ET

from pathlib import Path


ROOT = (
    Path.home()
    / "px4_ros2_ws"
    / "results"
)


def read_json(path):

    if path.is_file():

        return json.loads(
            path.read_text(
                encoding="utf-8"
            )
        )

    return {}


def get_boxes(scene_file):

    boxes = []

    if not scene_file.is_file():

        return boxes

    root = ET.parse(
        scene_file
    ).getroot()

    for link in root.findall(
        ".//link"
    ):

        pose = link.find(
            "pose"
        )

        collision = link.find(

            "collision/geometry/box/size"
        )

        if (
            pose is None

            or collision is None
        ):

            continue

        x, y, z, roll, pitch, yaw = (

            map(
                float,
                pose.text.split(),
            )
        )

        sx, sy, sz = map(

            float,

            collision.text.split(),
        )

        if max(

            abs(roll),

            abs(pitch),

            abs(yaw),

        ) > 1e-8:

            raise ValueError(
                "Evaluation expects "
                "axis-aligned boxes"
            )

        boxes.append({

            "name": link.attrib[
                "name"
            ],

            "xmin": x - sx / 2,

            "xmax": x + sx / 2,

            "ymin": y - sy / 2,

            "ymax": y + sy / 2,

            "zmin": z - sz / 2,

            "zmax": z + sz / 2,
        })

    return boxes


def distance_to_box(
    point,
    box,
):

    distances = []

    for value, axis in zip(

        point,

        ("x", "y", "z"),
    ):

        distances.append(

            max(

                box[f"{axis}min"]
                - value,

                0.0,

                value
                - box[f"{axis}max"],
            )
        )

    return math.sqrt(

        sum(
            value * value

            for value in distances
        )
    )


def surface_distance(
    point,
    box,
):

    outside = distance_to_box(
        point,
        box,
    )

    if outside > 0:

        return outside

    # Interior point:
    # nearest actual box face.
    return min(

        min(

            point[index]
            - box[f"{axis}min"],

            box[f"{axis}max"]
            - point[index],
        )

        for index, axis in enumerate(

            ("x", "y", "z")
        )
    )


def evaluate(folder):

    report = read_json(

        folder
        / "mission_report.json"
    )

    metrics = read_json(

        folder
        / "metrics.json"
    )

    safe_plan = read_json(

        folder
        / "safe_plan.json"
    )

    visual = read_json(

        folder
        / "visual_target.json"
    )

    config = read_json(

        folder
        / "scene_config.json"
    )

    boxes = get_boxes(

        folder
        / "scene_truth.sdf"
    )

    csv_file = (

        folder
        / "trajectory.csv"
    )

    rows = []

    if csv_file.is_file():

        with csv_file.open(

            encoding="utf-8"
        ) as file:

            rows = list(

                csv.DictReader(file)
            )

    points = [

        (

            float(row["x"]),

            float(row["y"]),

            float(row["z"]),
        )

        for row in rows

        if row["armed"] == "1"

        and row["position_valid"] == "1"

        and row["connected"] == "1"
    ]

    result = {

        "mission_id":

            folder.name,

        "success":

            bool(
                report.get("success")
            ),

        "parser_mode":

            report.get(
                "parser_mode"
            ),

        "failure_category":

            report.get(
                "failure_category"
            ),

        "failure_reason":

            report.get(
                "failure_reason"
            ),

        **metrics,
    }

    # Include all physical buildings,
    # including the red target.
    # These values use simulation truth
    # only after mission execution.
    if boxes and points:

        min_distance = min(

            distance_to_box(
                point,
                box,
            )

            for point in points

            for box in boxes
        )

        body_radius = 0.35

        result[
            "min_all_building_distance_m"
        ] = min_distance

        result[
            "assumed_body_radius_m"
        ] = body_radius

        result[
            "min_body_clearance_proxy_m"
        ] = (

            min_distance

            - body_radius
        )

        result[
            "collision_distance_proxy"
        ] = (

            min_distance

            <= body_radius
        )

        result[
            "collision_measurement"
        ] = (

            "sampled position versus "
            "box geometry; assumed "
            "0.35 m vehicle envelope"
        )

    else:

        result[
            "collision_distance_proxy"
        ] = None

    targets = [

        box

        for box in boxes

        if box["name"].startswith(
            "red_house"
        )
    ]

    if (

        targets

        and "position" in visual
    ):

        p = visual[
            "position"
        ]

        result[
            "visual_surface_error_m"
        ] = min(

            surface_distance(

                p,

                box,
            )

            for box in targets
        )

        # Distance from hover point
        # to the actual target geometry.
        if "hover_position" in report:

            hover = report[
                "hover_position"
            ]

            result[
                "actual_target_standoff_m"
            ] = min(

                distance_to_box(

                    hover,

                    box,
                )

                for box in targets
            )

    if safe_plan:

        lengths = safe_plan[
            "lengths_m"
        ]

        result[
            "planned_outbound_length_m"
        ] = lengths[
            "outbound"
        ]

        result[
            "planned_return_length_m"
        ] = lengths[
            "return"
        ]

        result[
            "planned_total_length_m"
        ] = sum(
            lengths.values()
        )

    (
        folder
        / "evaluation.json"
    ).write_text(

        json.dumps(

            result,

            ensure_ascii=False,

            indent=2,

            allow_nan=False,
        ),

        encoding="utf-8",
    )

    if points or safe_plan:

        plot_trajectory(

            folder,

            rows,

            safe_plan,

            boxes,

            config,
        )

    return result


def plot_trajectory(

    folder,

    rows,

    safe_plan,

    boxes,

    config,
):

    import matplotlib

    matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    from matplotlib.patches import (
        Rectangle,
    )

    fig, ax = plt.subplots(

        figsize=(10, 8)
    )

    for box in boxes:

        is_target = box[
            "name"
        ].startswith(
            "red_house"
        )

        rectangle = Rectangle(

            (

                box["xmin"],

                box["ymin"],
            ),

            box["xmax"]
            - box["xmin"],

            box["ymax"]
            - box["ymin"],

            facecolor=(
                "firebrick"

                if is_target

                else "gray"
            ),

            alpha=0.55,
        )

        ax.add_patch(
            rectangle
        )

        ax.text(

            (

                box["xmin"]
                + box["xmax"]

            ) / 2,

            (

                box["ymin"]
                + box["ymax"]

            ) / 2,

            box["name"],

            ha="center",

            va="center",

            fontsize=8,
        )

    if config:

        clearance = config[
            "limits"
        ][
            "clearance"
        ]

        for obstacle in config[
            "obstacles"
        ]:

            ax.add_patch(

                Rectangle(

                    (

                        obstacle["xmin"]
                        - clearance,

                        obstacle["ymin"]
                        - clearance,
                    ),

                    obstacle["xmax"]
                    - obstacle["xmin"]
                    + 2 * clearance,

                    obstacle["ymax"]
                    - obstacle["ymin"]
                    + 2 * clearance,

                    fill=False,

                    edgecolor="orange",

                    linestyle="--",
                )
            )

    for key, label in (

        (
            "outbound_waypoints",
            "Planned outbound",
        ),

        (
            "return_waypoints",
            "Planned return",
        ),
    ):

        points = safe_plan.get(
            key,
            [],
        )

        if points:

            ax.plot(

                [p[0] for p in points],

                [p[1] for p in points],

                "o--",

                markersize=3,

                linewidth=1.3,

                label=label,
            )

    for phase in (

        "TAKEOFF",

        "OUTBOUND",

        "HOVER",

        "RETURN",

        "LAND",

        "RECOVERY_LAND",
    ):

        segment = [

            row

            for row in rows

            if (

                row["stage"] == phase

                and row["position_valid"] == "1"
            )
        ]

        if segment:

            ax.plot(

                [

                    float(row["x"])

                    for row in segment
                ],

                [

                    float(row["y"])

                    for row in segment
                ],

                linewidth=2,

                label=phase,
            )

    if "target_surface" in safe_plan:

        x, y, _ = safe_plan[
            "target_surface"
        ]

        ax.scatter(

            [x],

            [y],

            marker="x",

            s=90,

            label="RGB-D target surface",
        )

    if "approach_point" in safe_plan:

        x, y, _ = safe_plan[
            "approach_point"
        ]

        ax.scatter(

            [x],

            [y],

            marker="*",

            s=100,

            label="Approach goal",
        )

    ax.set_xlabel(
        "East x (m)"
    )

    ax.set_ylabel(
        "North y (m)"
    )

    ax.set_title(

        "UAV-VLN trajectory and "

        "known-map obstacles"
    )

    ax.set_aspect(
        "equal",
        adjustable="box",
    )

    ax.autoscale_view()

    ax.grid(
        True,
        alpha=0.25,
    )

    ax.legend(
        fontsize=8
    )

    fig.tight_layout()

    fig.savefig(

        folder
        / "trajectory.png",

        dpi=180,
    )

    plt.close(fig)


def main():

    parser = (
        argparse.ArgumentParser()
    )

    parser.add_argument(

        "--all",

        action="store_true",
    )

    parser.add_argument(

        "--run",

        default="latest",
    )

    args = parser.parse_args()

    folders = sorted(

        folder

        for folder in ROOT.iterdir()

        if folder.is_dir()

        and (

            folder
            / "mission_report.json"
        ).is_file()
    )

    if not folders:

        raise SystemExit(
            "No completed mission results"
        )

    if args.all:

        selected = folders

    elif args.run == "latest":

        selected = [
            folders[-1]
        ]

    else:

        selected = [

            ROOT / args.run
        ]

    results = []

    for folder in selected:

        result = evaluate(
            folder
        )

        results.append(
            result
        )

        print(

            json.dumps(

                result,

                ensure_ascii=False,

                indent=2,
            )
        )

    if args.all:

        keys = sorted({

            key

            for result in results

            for key in result
        })

        with (

            ROOT
            / "summary.csv"

        ).open(

            "w",

            newline="",

            encoding="utf-8",

        ) as file:

            writer = csv.DictWriter(

                file,

                fieldnames=keys,
            )

            writer.writeheader()

            writer.writerows(
                results
            )

        print(

            "Summary:",

            ROOT
            / "summary.csv",
        )


if __name__ == "__main__":

    main()
