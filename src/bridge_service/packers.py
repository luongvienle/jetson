# packers.py
# Simple packer utilities for mapping ROS2 message fields into binary packets for Yamcs.
import struct
import yaml
import importlib
from typing import Any, Dict, Callable

# Supported basic type packers mapping to struct format (little-endian)
BASIC_PACKERS = {
    'uint8': '<B',
    'int8': '<b',
    'uint16': '<H',
    'int16': '<h',
    'uint32': '<I',
    'int32': '<i',
    'uint64': '<Q',
    'int64': '<q',
    'float': '<f',
    'double': '<d',
}

def load_mapping(path: str) -> Dict:
    """Load YAML mapping file describing topics to bridge.

    Example mapping.yaml:

    topics:
      - name: /fmu/vehicle_imu/out
        type: px4_msgs.msg.VehicleImu
        time_field: timestamp
        pack:
          - field: accel_m_s2[0]
            type: float
            name: imu.accel_x
          - field: accel_m_s2[1]
            type: float
            name: imu.accel_y
          - field: accel_m_s2[2]
            type: float
            name: imu.accel_z

    yamcs:
      protocol: udp
      host: 127.0.0.1
      port: 10015
    """
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def resolve_field_value(msg: Any, field_path: str):
    # support indexed fields like accel_m_s2[0]
    if '[' in field_path and ']' in field_path:
        base, rest = field_path.split('[',1)
        idx = int(rest.split(']')[0])
        val = getattr(msg, base)
        return val[idx]
    else:
        # nested attribute e.g., header.stamp.sec (if present)
        parts = field_path.split('.')
        cur = msg
        for p in parts:
            if hasattr(cur, p):
                cur = getattr(cur, p)
            else:
                raise AttributeError(f"Field {field_path} not found in message {type(msg)}")
        return cur

def pack_message_by_definition(msg: Any, pack_def: Dict) -> bytes:
    """Pack a ROS2 message according to pack_def (list of field specs)."""
    out = bytearray()
    for item in pack_def:
        field = item['field']
        t = item['type']
        if t in BASIC_PACKERS:
            fmt = BASIC_PACKERS[t]
            val = resolve_field_value(msg, field)
            try:
                out += struct.pack(fmt, val)
            except struct.error as e:
                raise
        elif t == 'timestamp':
            # expect integer microseconds or nanoseconds; pack as uint64 (microseconds)
            val = resolve_field_value(msg, field)
            # Normalize: accept either seconds float, or integer micros/nanos
            if isinstance(val, float):
                micros = int(val * 1e6)
            else:
                # integer: assume microseconds or nanoseconds. Heuristic: if >1e12 assume ns -> convert to us
                if val > 1e12:
                    micros = int(val / 1000)
                else:
                    micros = int(val)
            out += struct.pack('<Q', micros)
        else:
            raise ValueError(f"Unsupported packer type: {t}")
    return bytes(out)

def import_msg_class(typename: str):
    """Dynamically import a ROS2 message class like 'px4_msgs.msg.VehicleImu'"""
    module_name, class_name = typename.rsplit('.', 1)
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name)
    return cls
