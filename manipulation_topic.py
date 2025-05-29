 #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import datetime
import signal
import sys

class RobotStatusMonitor(Node):
    """Minimal monitor: print status changes only."""

    def __init__(self):
        super().__init__('manipulation_node')   # ← node name

        self.last_status = None
        self.create_subscription(String,
                                 '/robot_status',
                                 self.status_callback,
                                 10)
        self.get_logger().info('✓ Started manipulation_node → listening on /robot_status')

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------
    def status_callback(self, msg: String):
        status = msg.data
        if status == self.last_status:          # ignore repeats
            return

        ts = datetime.datetime.now().strftime('%H:%M:%S')
        print(f'[{ts}] status = {status}')
        self.last_status = status

# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main():
    rclpy.init()
    monitor = RobotStatusMonitor()

    # Handle Ctrl-C gracefully
    def sigint_handler(sig, frame):
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)

    rclpy.spin(monitor)

if __name__ == '__main__':
    main()