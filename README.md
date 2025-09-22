# ROS2 → Yamcs Bridge (MacOS) - Template Service

**Purpose:** A Python-based bridge service that subscribes to ROS 2 topics (px4_msgs or others),
packs each sample into a binary packet according to a simple packer scheme, and sends telemetry
packets to Yamcs over UDP or TCP in near-realtime.

**Notes for macOS**
- This project assumes you have ROS 2 (e.g., Humble/Iron) installed on macOS and `rclpy` is available.
  Installing ROS2 on macOS is possible but more involved than on Ubuntu; see ROS 2 docs.
- You will run this bridge on the Recording PC (macOS). The Jetson publishes to DDS; the Recording PC
  must be able to see DDS/ROS2 topics (same ROS_DOMAIN_ID or use discovery server).

## Quick start (development)

1. Create a Python virtualenv (optional):
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```
   **Note:** `rclpy` is typically provided by your ROS2 install; do *not* pip-install rclpy unless you know
   your environment provides a compatible binary. When running the service, source your ROS2 setup:
   ```bash
   source /opt/ros/humble/setup.bash   # or the path for your ROS2 distro on macOS
   ```

2. Edit `bridge/config/mapping.yaml` to list topics you want to bridge and how to pack them.

3. Run the bridge service (example UDP):
   ```bash
   ./run_bridge.sh bridge/config/mapping.yaml --protocol udp --yamcs-host 127.0.0.1 --yamcs-port 10015
   ```

## What the bridge does
- Loads topic->parameter mapping from YAML.
- Dynamically imports ROS2 message classes (e.g., `px4_msgs.msg.VehicleImu`).
- Subscribes to configured topics.
- Runs packers that convert message fields into binary payloads (struct).
- Sends each packed packet to Yamcs over UDP/TCP.

## Project structure
- bridge/bridge_node.py       : main rclpy node (dynamic subscribers)
- bridge/packers.py          : packer functions and mapping loader
- bridge/datalink.py         : UDP/TCP client (asyncio)
- bridge/config/mapping.yaml : example mapping for topics
- run_bridge.sh              : helper run script
- requirements.txt           : Python dependencies (for non-rclpy libs)

## Customization hints
- Define exact MDB/XTCE format required by Yamcs; replace packer logic in `packers.py` to match.
- Use `timestamp` or `timestamp_sample` fields from messages as primary time; mapping supports `time_field`.
- Add support for batching or sequence numbers if Yamcs configuration expects them.

## Troubleshooting
- If discovery fails, ensure `ROS_DOMAIN_ID` is identical on Jetson and Recording PC or use an RTPS discovery server (Fast DDS Discovery Server).
- Ensure network allows multicast or that DDS discovery server is reachable.
- For macOS service integration: create a `launchd` plist or use `brew services` with a wrapper script.

