# bridge_node.py
import sys
import asyncio
import rclpy
from rclpy.node import Node
import argparse
import importlib
from src.bridge_service.bridge_service.packers import load_mapping, import_msg_class, pack_message_by_definition
from src.bridge_service.bridge_service.datalink import UDPClient, TCPClient

class TopicSubscriber:
    def __init__(self, node: Node, topic_conf: dict, send_cb):
        self.node = node
        self.topic_conf = topic_conf
        self.send_cb = send_cb
        self.msg_type = import_msg_class(topic_conf['type'])
        self.sub = node.create_subscription(self.msg_type, topic_conf['name'], self.callback, 10)
        self.pack_def = topic_conf['pack']
        self.time_field = topic_conf.get('time_field', None)

    def callback(self, msg):
        try:
            payload = pack_message_by_definition(msg, self.pack_def)
        except Exception as e:
            self.node.get_logger().error(f"Packing error for topic {self.topic_conf['name']}: {e}")
            return
        # Optionally prefix with metadata (topic id, length). For now send raw payload.
        asyncio.ensure_future(self.send_cb(payload))

class BridgeNode(Node):
    def __init__(self, mapping, protocol, yamcs_host, yamcs_port):
        super().__init__('ros2_yamcs_bridge')
        self.get_logger().info('Bridge node starting...')
        self.mapping = mapping
        self.protocol = protocol
        self.yamcs_host = yamcs_host
        self.yamcs_port = yamcs_port
        self.loop = asyncio.get_event_loop()
        if protocol.lower() == 'udp':
            self.client = UDPClient(yamcs_host, yamcs_port)
        else:
            self.client = TCPClient(yamcs_host, yamcs_port)
        self.subscribers = []
        for t in mapping.get('topics', []):
            sub = TopicSubscriber(self, t, self.async_send)
            self.subscribers.append(sub)
        self.get_logger().info(f"Subscribed {len(self.subscribers)} topics.")

    async def async_send(self, payload: bytes):
        # Here you can add envelope, sequence number or timestamp prefix
        try:
            await self.client.send(payload)
        except Exception as e:
            self.get_logger().error(f"Error sending payload: {e}")

    def destroy_node(self):
        # ensure client closed
        try:
            self.loop.run_until_complete(self.client.close())
        except Exception:
            pass
        super().destroy_node()

def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('mapping', nargs='?', default='bridge_service/mapping.yaml', help='path to mapping.yaml (default: bridge_service/mapping.yaml)')
    parser.add_argument('--protocol', default='udp', choices=['udp','tcp'])
    parser.add_argument('--yamcs-host', default='127.0.0.1')
    parser.add_argument('--yamcs-port', default=10015, type=int)
    args = parser.parse_args(argv[1:] if argv else None)

    mapping = load_mapping(args.mapping)
    # init rclpy
    rclpy.init()
    bridge = BridgeNode(mapping, args.protocol, args.yamcs_host, args.yamcs_port)
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.get_logger().info('Shutting down bridge...')
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
