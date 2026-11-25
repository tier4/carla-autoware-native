"""Sensor layout for run_scenario.py --sensor-layout (no carla import).

A layout is a JSON file describing an ego vehicle and the sensors mounted on it:

  {"ego_blueprint": "vehicle.x.y", "role_name": "ego",
   "base_link":  [x, y, z, roll, pitch, yaw],      # offset of the base_link actor from the ego, CARLA m / deg
   "sensor_kit": [x, y, z, roll, pitch, yaw],      # offset of the sensor_kit actor from base_link
   "sensors": [{"name": "...", "blueprint": "sensor.lidar.rgl", "model": "<lidar_models preset>",
                "attach": "ego|base_link|sensor_kit", "transform": [x, y, z, roll, pitch, yaw],
                "attributes": {"<blueprint attribute>": "<string value>", ...}}, ...]}

All transforms are carla.Transform values (metres, degrees, UE axes). Layout files describe vehicles and are
generated outside the repository; none is committed.
"""
import json
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

Tuple6 = Tuple[float, float, float, float, float, float]
UDP_PORT_KEY = "rgl_udp_dest_port"
MAX_PORT = 65535


@dataclass(frozen=True)
class LayoutSensor:
    """One sensor mounting and blueprint configuration."""
    name: str
    blueprint: str
    model: str
    attach: str
    transform: Tuple6
    attributes: Dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class Layout:
    """Ego and sensor mounting configuration loaded from JSON."""
    ego_blueprint: str
    role_name: str
    base_link: Tuple6
    sensor_kit: Tuple6
    sensors: List[LayoutSensor]


def _need(d, key, where):
    if not isinstance(d, dict) or key not in d:
        raise ValueError(f"layout: missing key {key!r} in {where}")
    return d[key]


def _tuple6(v, where):
    try:
        t = tuple(float(x) for x in v)
    except (TypeError, ValueError):
        raise ValueError(f"layout: {where} is not a list of 6 numbers") from None
    if len(t) != 6:
        raise ValueError(f"layout: {where} needs 6 numbers (x, y, z, roll, pitch, yaw), got {len(t)}")
    return t


def load_layout(path, udp_port_offset):
    """Read a layout file; every sensor's rgl_udp_dest_port gets udp_port_offset added.

    ValueError for a missing key, a malformed transform or a shifted port above 65535 (or not an integer).
    A sensor without rgl_udp_dest_port is not a UDP sender and is left as it is.
    """
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
    except json.JSONDecodeError as e:
        raise ValueError(f"layout: {path} is not valid JSON: {e}") from e
    sensors = []
    for i, s in enumerate(_need(d, "sensors", "the layout")):
        where = f"sensors[{i}]"
        attrs = {str(k): str(v) for k, v in _need(s, "attributes", where).items()}
        if UDP_PORT_KEY in attrs:
            try:
                port = int(attrs[UDP_PORT_KEY]) + int(udp_port_offset)
            except ValueError:
                raise ValueError(f"layout: {where} {UDP_PORT_KEY}={attrs[UDP_PORT_KEY]!r} is not an integer") \
                    from None
            if not 0 < port <= MAX_PORT:
                # Preserve percent-format numeric coercion and invalid-input errors.
                # pylint: disable-next=consider-using-f-string
                raise ValueError("layout: %s %s=%d is outside 1..%d after the offset" % (where, UDP_PORT_KEY, port,
                                                                                         MAX_PORT))
            attrs[UDP_PORT_KEY] = str(port)
        sensors.append(LayoutSensor(
            name=str(_need(s, "name", where)), blueprint=str(_need(s, "blueprint", where)),
            model=str(_need(s, "model", where)), attach=str(_need(s, "attach", where)),
            transform=_tuple6(_need(s, "transform", where), where + ".transform"), attributes=attrs))
    return Layout(ego_blueprint=str(_need(d, "ego_blueprint", "the layout")),
                  role_name=str(_need(d, "role_name", "the layout")),
                  base_link=_tuple6(_need(d, "base_link", "the layout"), "base_link"),
                  sensor_kit=_tuple6(_need(d, "sensor_kit", "the layout"), "sensor_kit"),
                  sensors=sensors)


def udp_ports(layout):
    """The (already shifted) UDP destination ports of the layout's sensors, in sensor order."""
    return [int(s.attributes[UDP_PORT_KEY]) for s in layout.sensors if UDP_PORT_KEY in s.attributes]
