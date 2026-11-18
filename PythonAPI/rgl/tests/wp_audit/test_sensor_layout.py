"""Tests for sensor_layout.py (pure JSON handling, no carla)."""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor_layout import Layout, LayoutSensor, load_layout, udp_ports  # noqa: E402


def _sensor(name, port, transform=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
    return {"name": name, "blueprint": "sensor.lidar.rgl", "model": "HesaiPandar128E4X",
            "attach": "sensor_kit", "transform": list(transform),
            "attributes": {"sensor_tick": "0.1", "rgl_udp_enabled": "true", "rgl_udp_dest_port": str(port)}}


def _layout_dict(ports=(2322, 2377)):
    return {"ego_blueprint": "vehicle.test.car", "role_name": "ego",
            "base_link": [-2.0, 0.0, 0.0, 0.0, 0.0, 0.0], "sensor_kit": [4.0, -1.0, 2.5, 0.0, 0.0, 0.0],
            "sensors": [_sensor("s%d" % i, p, (0.0, 0.0, 0.1 * i, 0.0, 0.0, 90.0)) for i, p in enumerate(ports)]}


def _write(tmp_path, d, name="layout.json"):
    p = tmp_path / name
    p.write_text(json.dumps(d))
    return str(p)


def test_layout_port_offset(tmp_path):
    p = _write(tmp_path, _layout_dict())
    assert udp_ports(load_layout(p, 40000)) == [42322, 42377]
    assert udp_ports(load_layout(p, 0)) == [2322, 2377]


def test_layout_fields(tmp_path):
    lay = load_layout(_write(tmp_path, _layout_dict()), 40000)
    assert isinstance(lay, Layout) and lay.ego_blueprint == "vehicle.test.car" and lay.role_name == "ego"
    assert lay.base_link == (-2.0, 0.0, 0.0, 0.0, 0.0, 0.0) and lay.sensor_kit == (4.0, -1.0, 2.5, 0.0, 0.0, 0.0)
    s = lay.sensors[1]
    assert isinstance(s, LayoutSensor) and s.name == "s1" and s.model == "HesaiPandar128E4X"
    assert s.attach == "sensor_kit" and s.transform == (0.0, 0.0, 0.1, 0.0, 0.0, 90.0)
    assert s.attributes["rgl_udp_dest_port"] == "42377" and s.attributes["sensor_tick"] == "0.1"


def test_layout_missing_transform_raises(tmp_path):
    d = _layout_dict()
    del d["sensors"][0]["transform"]
    with pytest.raises(ValueError):
        load_layout(_write(tmp_path, d), 40000)


def test_layout_missing_top_level_key_and_bad_transform_raise(tmp_path):
    d = _layout_dict()
    del d["sensor_kit"]
    with pytest.raises(ValueError):
        load_layout(_write(tmp_path, d, "a.json"), 40000)
    d = _layout_dict()
    d["sensors"][1]["transform"] = [0.0, 0.0, 0.0]
    with pytest.raises(ValueError):
        load_layout(_write(tmp_path, d, "b.json"), 40000)


def test_layout_port_over_65535_raises(tmp_path):
    p = _write(tmp_path, _layout_dict(ports=(2322, 25600)))
    with pytest.raises(ValueError):
        load_layout(p, 40000)                 # 65600 > 65535
    assert udp_ports(load_layout(p, 39935)) == [42257, 65535]   # exactly 65535 is fine


def test_layout_sensor_without_udp_port_is_left_alone(tmp_path):
    d = _layout_dict()
    del d["sensors"][0]["attributes"]["rgl_udp_dest_port"]
    lay = load_layout(_write(tmp_path, d), 40000)
    assert udp_ports(lay) == [42377]
    assert "rgl_udp_dest_port" not in lay.sensors[0].attributes
