import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ArmMonitorNode(Node):

    def __init__(self):
        super().__init__("arm_monitor_node")

        self.state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10,
        )

        self.get_logger().info(
            'Arm Monitor Node started'
        )

    def joint_state_callback(self,msg):

        if len(msg.position) != 6:
            self.get_logger().warning(
                f'Expected 6 joint positions , but got {len(msg.position)}'
            )
            return

        joint_degrees = [
            math.degrees(angle)
            for angle in msg.position
        ]

        self.get_logger().info(
            'current joints | '
            + ' '.join(
                f'J{index + 1}={angle:2f}°'
                for index, angle in enumerate(joint_degrees)
            )
        )


def main(args=None):
    rclpy.init(args=args)

    node = ArmMonitorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()