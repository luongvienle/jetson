import sys
import asyncio
import rclpy
from rclpy.node import Node
from bridge_service.packers import load_mapping, import_msg_class, pack_message_by_definition
from bridge_service.datalink import UDPClient, TCPClient


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
        asyncio.ensure_future(self.send_cb(payload))


class BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_yamcs_bridge')

        # Declare and get ROS parameters
        self.declare_parameter('mapping', 'src/bridge_service/mapping.yaml')
        self.declare_parameter('protocol', 'udp')
        self.declare_parameter('yamcs_host', '127.0.0.1')
        self.declare_parameter('yamcs_port', 10015)

        mapping_path = self.get_parameter('mapping').get_parameter_value().string_value
        protocol = self.get_parameter('protocol').get_parameter_value().string_value
        yamcs_host = self.get_parameter('yamcs_host').get_parameter_value().string_value
        yamcs_port = self.get_parameter('yamcs_port').get_parameter_value().integer_value

        self.get_logger().info('Bridge node starting...')
        self.get_logger().info(f'Loading mapping from: {mapping_path}')

        try:
            self.mapping = load_mapping(mapping_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load mapping file: {e}")
            raise

        self.protocol = protocol
        self.yamcs_host = yamcs_host
        self.yamcs_port = yamcs_port
        self.loop = asyncio.get_event_loop()

        if protocol.lower() == 'udp':
            self.client = UDPClient(yamcs_host, yamcs_port)
        else:
            self.client = TCPClient(yamcs_host, yamcs_port)

        self.subscribers = []
        for t in self.mapping.get('topics', []):
            sub = TopicSubscriber(self, t, self.async_send)
            self.subscribers.append(sub)

        self.get_logger().info(f"Subscribed to {len(self.subscribers)} topics.")

    async def async_send(self, payload: bytes):
        try:
            await self.client.send(payload)
        except Exception as e:
            self.get_logger().error(f"Error sending payload: {e}")

    def destroy_node(self):
        try:
            self.loop.run_until_complete(self.client.close())
        except Exception:
            pass
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    bridge = BridgeNode()
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
