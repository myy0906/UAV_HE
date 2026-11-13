#!/usr/bin/env python3

import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path.home() / "px4_ros2_ws"

MAP_FILE = (
    ROOT
    / "src/uav_vln_bringup/config/known_map.json"
)

MODEL_DIR = (
    ROOT
    / "src/uav_vln_bringup/models/vln_scene"
)

MODEL_FILE = MODEL_DIR / "model.sdf"


def add_box(
    model,
    name,
    center,
    size,
    color,
):
    link = ET.SubElement(
        model,
        "link",
        {"name": name},
    )

    pose = ET.SubElement(link, "pose")

    pose.text = (
        f"{center[0]} "
        f"{center[1]} "
        f"{center[2]} 0 0 0"
    )

    # Physical collision geometry.
    collision = ET.SubElement(
        link,
        "collision",
        {"name": f"{name}_collision"},
    )

    geometry = ET.SubElement(
        collision,
        "geometry",
    )

    box = ET.SubElement(geometry, "box")

    ET.SubElement(box, "size").text = (
        f"{size[0]} "
        f"{size[1]} "
        f"{size[2]}"
    )

    # Visual geometry.
    visual = ET.SubElement(
        link,
        "visual",
        {"name": f"{name}_visual"},
    )

    geometry = ET.SubElement(
        visual,
        "geometry",
    )

    box = ET.SubElement(geometry, "box")

    ET.SubElement(box, "size").text = (
        f"{size[0]} "
        f"{size[1]} "
        f"{size[2]}"
    )

    material = ET.SubElement(
        visual,
        "material",
    )

    color_text = " ".join(
        str(value) for value in color
    )

    ET.SubElement(
        material,
        "ambient",
    ).text = color_text

    ET.SubElement(
        material,
        "diffuse",
    ).text = color_text

    ET.SubElement(
        material,
        "specular",
    ).text = "0.1 0.1 0.1 1"


def main():
    with MAP_FILE.open(
        "r",
        encoding="utf-8",
    ) as file:
        config = json.load(file)

    sdf = ET.Element(
        "sdf",
        {"version": "1.9"},
    )

    model = ET.SubElement(
        sdf,
        "model",
        {"name": "vln_scene"},
    )

    ET.SubElement(
        model,
        "static",
    ).text = "true"

    # Known rectangular buildings.
    # Navigation uses the same map configuration.
    for obstacle in config["obstacles"]:

        xmin = float(obstacle["xmin"])
        xmax = float(obstacle["xmax"])
        ymin = float(obstacle["ymin"])
        ymax = float(obstacle["ymax"])
        height = float(obstacle["height"])

        if xmax <= xmin:
            raise ValueError(
                "Invalid obstacle x bounds"
            )

        if ymax <= ymin:
            raise ValueError(
                "Invalid obstacle y bounds"
            )

        if height <= 0:
            raise ValueError(
                "Invalid obstacle height"
            )

        center = (
            (xmin + xmax) / 2.0,
            (ymin + ymax) / 2.0,
            height / 2.0,
        )

        size = (
            xmax - xmin,
            ymax - ymin,
            height,
        )

        add_box(
            model=model,
            name=obstacle["name"],
            center=center,
            size=size,
            color=(0.55, 0.57, 0.60, 1.0),
        )

        print(
            "Generated known obstacle:",
            obstacle["name"],
            "center=", center,
            "size=", size,
        )

    # The target truth belongs to Gazebo only.
    # Never copy these coordinates into known_map.json
    # or into the navigation/mission modules.
    add_box(
        model=model,
        name="red_house_body",
        center=(16.0, 0.0, 1.5),
        size=(4.0, 4.0, 3.0),
        color=(0.85, 0.02, 0.02, 1.0),
    )

    add_box(
        model=model,
        name="red_house_roof",
        center=(16.0, 0.0, 3.25),
        size=(4.4, 4.4, 0.5),
        color=(0.50, 0.02, 0.02, 1.0),
    )

    MODEL_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    ET.indent(sdf, space="    ")

    tree = ET.ElementTree(sdf)

    tree.write(
        MODEL_FILE,
        encoding="utf-8",
        xml_declaration=True,
    )

    print()
    print("Scene generated successfully:")
    print(MODEL_FILE)

    print()
    print(
        "Target truth is present only "
        "in simulation scene generation."
    )


if __name__ == "__main__":
    main()
