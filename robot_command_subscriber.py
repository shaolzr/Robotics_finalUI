import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

def main():
    rclpy.init()
    node = Node('robot_command_subscriber')

    def callback(msg):
        try:
            data = json.loads(msg.data)
            print(f'[robot_command_subscriber] Received command: id={data.get("id")}, object={data.get("object")}, destination={data.get("destination")}, raw={msg.data}')
        except Exception as e:
            print(f'[robot_command_subscriber] Received invalid message: {msg.data}, error: {e}')

    node.create_subscription(String, 'robot_command', callback, 10)
    print('[robot_command_subscriber] Listening on topic: robot_command')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down robot_command_subscriber')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 