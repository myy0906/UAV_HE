#!/usr/bin/env python3

import json
import math
import re

from jsonschema import Draft202012Validator


class ProtocolError(ValueError):
    pass


def number(minimum, maximum):
    return {
        "type": "number",
        "minimum": minimum,
        "maximum": maximum,
    }


def action_schema(action_type, properties):
    fields = {
        "action_type": {
            "const": action_type
        },
        **properties,
    }

    return {
        "type": "object",
        "properties": fields,
        "required": list(fields.keys()),
        "additionalProperties": False,
    }


SCHEMA = {
    "$schema":
        "https://json-schema.org/draft/2020-12/schema",

    "title":
        "UAV-VLN semantic mission protocol",

    "type": "object",

    "required": [
        "frame_id",
        "actions",
    ],

    "additionalProperties": False,

    "properties": {
        "frame_id": {
            "const": "map_enu",
        },

        "actions": {
            "type": "array",
            "minItems": 4,
            "maxItems": 5,

            "items": {
                "oneOf": [

                    action_schema(
                        "takeoff",
                        {
                            "altitude":
                                number(1.0, 4.0),

                            "timeout":
                                number(20.0, 90.0),
                        },
                    ),

                    action_schema(
                        "goto",
                        {
                            "target_semantics": {
                                "const":
                                    "red_house",
                            },

                            "standoff_distance":
                                number(2.0, 6.0),

                            "velocity_limit":
                                number(0.1, 1.0),

                            "timeout":
                                number(15.0, 240.0),
                        },
                    ),

                    action_schema(
                        "hover",
                        {
                            "duration":
                                number(1.0, 30.0),
                        },
                    ),

                    action_schema(
                        "return_home",
                        {
                            "velocity_limit":
                                number(0.1, 1.0),

                            "timeout":
                                number(15.0, 240.0),
                        },
                    ),

                    action_schema(
                        "land",
                        {
                            "timeout":
                                number(20.0, 90.0),
                        },
                    ),
                ]
            },
        },
    },
}


def reject_constant(value):
    raise ProtocolError(
        f"Non-finite JSON constant: {value}"
    )


def unique_object(pairs):
    obj = {}

    for key, value in pairs:

        if key in obj:
            raise ProtocolError(
                f"Duplicate JSON key: {key}"
            )

        obj[key] = value

    return obj


def strict_json_loads(text):
    if not isinstance(text, str):
        raise ProtocolError(
            "JSON input must be a string"
        )

    try:
        return json.loads(
            text,

            parse_constant=reject_constant,

            object_pairs_hook=unique_object,
        )

    except json.JSONDecodeError as exc:
        raise ProtocolError(
            f"Invalid JSON: {exc}"
        ) from exc


def validate_finite(value):

    if isinstance(value, float):

        if not math.isfinite(value):
            raise ProtocolError(
                "Non-finite numerical value"
            )

    elif isinstance(value, list):

        for item in value:
            validate_finite(item)

    elif isinstance(value, dict):

        for item in value.values():
            validate_finite(item)


def validate_mission(plan, config):
    validate_finite(plan)

    errors = list(
        Draft202012Validator(
            SCHEMA
        ).iter_errors(plan)
    )

    if errors:

        error = errors[0]

        path = ".".join(
            str(item)
            for item in error.absolute_path
        )

        raise ProtocolError(
            f"Schema validation failed "
            f"at {path}: {error.message}"
        )

    actions = plan["actions"]

    kinds = [
        action["action_type"]
        for action in actions
    ]

    allowed_sequences = [
        [
            "takeoff",
            "goto",
            "return_home",
            "land",
        ],

        [
            "takeoff",
            "goto",
            "hover",
            "return_home",
            "land",
        ],
    ]

    if kinds not in allowed_sequences:
        raise ProtocolError(
            "Unsupported action sequence"
        )

    fence = config["geofence"]
    limits = config["limits"]

    altitude = float(
        actions[0]["altitude"]
    )

    if not (
        fence["zmin"]
        <= altitude
        <= fence["zmax"]
    ):

        raise ProtocolError(
            "Altitude outside geofence"
        )

    for action in actions:

        kind = action["action_type"]

        if kind in {
            "goto",
            "return_home",
        }:

            speed = float(
                action["velocity_limit"]
            )

            if speed > limits[
                "horizontal_speed_max"
            ]:

                raise ProtocolError(
                    "Requested speed exceeds limit"
                )

            # The retained PX4 executor uses
            # position setpoints, not a per-goal
            # velocity setting.
            # This version uses one fixed
            # execution speed configuration.
            if abs(
                speed
                - limits[
                    "horizontal_speed_max"
                ]
            ) > 1e-6:

                raise ProtocolError(
                    "Only the configured fixed "
                    "navigation speed is supported"
                )

        if kind == "goto":

            distance = float(
                action["standoff_distance"]
            )

            if not (
                limits["standoff_min"]
                <= distance
                <= limits["standoff_max"]
            ):

                raise ProtocolError(
                    "Invalid standoff distance"
                )

        if kind == "hover":

            if action["duration"] > limits[
                "hover_duration_max"
            ]:

                raise ProtocolError(
                    "Hover duration exceeds limit"
                )

    return plan


def make_plan(
    altitude=2.0,
    standoff=3.0,
    hover=3.0,
    speed=1.0,
    goto_timeout=120.0,
    return_timeout=120.0,
):

    actions = [

        {
            "action_type": "takeoff",
            "altitude": altitude,
            "timeout": 45.0,
        },

        {
            "action_type": "goto",
            "target_semantics": "red_house",
            "standoff_distance": standoff,
            "velocity_limit": speed,
            "timeout": goto_timeout,
        },
    ]

    if hover is not None:

        actions.append(
            {
                "action_type": "hover",
                "duration": hover,
            }
        )

    actions.extend(
        [
            {
                "action_type": "return_home",
                "velocity_limit": speed,
                "timeout": return_timeout,
            },

            {
                "action_type": "land",
                "timeout": 45.0,
            },
        ]
    )

    return {
        "frame_id": "map_enu",
        "actions": actions,
    }


NUMBER = r"(-?\d+(?:\.\d+)?)"


def requested_constraints(text):
    """
    Explicit numeric constraints recognized
    in the supported instruction language.
    """

    values = {}

    patterns = {

        "altitude": [
            rf"(?:起飞到|飞到|保持|高度为|高度)"
            rf"\s*{NUMBER}\s*(?:米|m)",

            rf"(?:红房子|红色建筑|红楼)"
            rf"\s*上方\s*{NUMBER}\s*(?:米|m)",

            rf"altitude\s*(?:of|=|:)?"
            rf"\s*{NUMBER}\s*m",
        ],

        "speed": [
            rf"{NUMBER}\s*(?:m/s|米/秒|米每秒)",
        ],

        "standoff": [
            rf"(?:前方|前面|距离|停距)"
            rf"\s*{NUMBER}\s*(?:米|m)",

            rf"standoff\s*(?:=|:)?"
            rf"\s*{NUMBER}\s*m",
        ],

        "hover": [
            rf"悬停\s*{NUMBER}\s*(?:秒|s)",

            rf"hover\s*(?:for)?"
            rf"\s*{NUMBER}\s*s",
        ],

        "goto_timeout": [
            rf"{NUMBER}\s*秒内"
            rf"(?:到达|飞到|飞抵)",

            rf"(?:到达|去程)超时"
            rf"\s*{NUMBER}\s*(?:秒|s)",

            rf"goto timeout\s*=?"
            rf"\s*{NUMBER}\s*s",
        ],
    }

    for key, group in patterns.items():

        found = []

        for pattern in group:

            for match in re.finditer(
                pattern,
                text,
                flags=re.IGNORECASE,
            ):

                found.append(
                    float(match.group(1))
                )

        if found:

            if max(found) - min(found) > 1e-6:

                raise ProtocolError(
                    f"Conflicting constraints: {key}"
                )

            values[key] = found[0]

    return values


def check_instruction_scope(text):
    if not text.strip():

        raise ProtocolError(
            "Empty instruction"
        )

    known = (
        "红房子",
        "红色房子",
        "红色建筑",
        "红楼",
        "红色楼",
        "red house",
        "red building",
    )

    lower = text.lower()

    if not any(
        word in lower
        for word in known
    ):

        raise ProtocolError(
            "Unknown or ambiguous target"
        )

    unsupported = (
        "蓝色",
        "黄色",
        "停车场",
        "树木区域",
        "绕圈",
        "盘旋",
        "穿过建筑",
        "关闭安全",
        "忽略安全",
        "关闭围栏",
        "禁止返航",
        "不要返航",
        "不返航",
        "不要降落",
        "不降落",
        "ignore safety",
        "disable geofence",
        "do not return",
        "blue building",
    )

    if any(
        word in lower
        for word in unsupported
    ):

        raise ProtocolError(
            "Instruction contains unsupported "
            "or prohibited operation"
        )


def verify_constraints(text, plan):

    requested = requested_constraints(text)

    actions = plan["actions"]

    by_type = {
        action["action_type"]: action
        for action in actions
    }

    actual = {

        "altitude":
            by_type["takeoff"]["altitude"],

        "speed":
            by_type["goto"]["velocity_limit"],

        "standoff":
            by_type["goto"]["standoff_distance"],

        "goto_timeout":
            by_type["goto"]["timeout"],

        "hover":
            by_type.get(
                "hover",
                {},
            ).get("duration"),
    }

    for key, value in requested.items():

        result = actual.get(key)

        if (
            result is None
            or abs(
                float(result) - value
            ) > 1e-6
        ):

            raise ProtocolError(
                f"Explicit constraint changed "
                f"or omitted: {key}, "
                f"requested={value}, "
                f"parsed={result}"
            )


def parse_rule(text):

    check_instruction_scope(text)

    values = requested_constraints(text)

    return make_plan(

        altitude=values.get(
            "altitude",
            2.0,
        ),

        standoff=values.get(
            "standoff",
            3.0,
        ),

        hover=values.get(
            "hover",
            3.0,
        ),

        speed=values.get(
            "speed",
            1.0,
        ),

        goto_timeout=values.get(
            "goto_timeout",
            120.0,
        ),
    )
