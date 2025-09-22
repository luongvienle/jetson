mapping.yaml fields:
- name: ROS2 topic name to subscribe
- type: full python import path of message class, e.g., px4_msgs.msg.VehicleImu
- time_field: optional field used for timestamp (used by Yamcs as primary time)
- pack: list of pack definitions; each pack entry:
    - field: attribute path on the msg (supports simple indexing like arr[0])
    - type: 'float','double','int32','uint64','timestamp' etc. See packers.BASIC_PACKERS
    - name: friendly name (informational)
