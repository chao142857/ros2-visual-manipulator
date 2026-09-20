import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class ArmCommandNode(Node):

    def __init__(self):
        super().__init__('arm_command_node')

        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
        ]

        self.command_publisher = self.create_publisher(
            JointState,
            'arm/joint_command',
            10
        )

        self.disable_client = self.create_client(
            Trigger,
            '/arm/disable'
        )

        self.stop_client = self.create_client(
            Trigger,
            '/arm/stop'
        )

        self.get_logger().info(
            'Arm Command Node started'
        )

    def send_joint_command(self, joint_degrees):

        if len(joint_degrees) != 6:
            self.get_logger().warning(
                f'Expected 6 joint angles, but got {len(joint_degrees)}'
            )
            return

        joint_radians = [
            math.radians(angle)
            for angle in joint_degrees
        ]

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joint_radians

        self.command_publisher.publish(msg)

        self.get_logger().info(
            f'Sent joint command | '
            f'deg={joint_degrees} '
            f'rad={joint_radians}'
        )
        return True

    def disable_arm(self):
        if not self.disable_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().warning(
                '/arm/disable service is not available'
            )
            return

        request = Trigger.Request()

        future = self.disable_client.call_async(
            request
        )

        while rclpy.ok() and not future.done():
            time.sleep(0.01)

        try:

            response = future.result()

            if response.success:
                self.get_logger().info(
                    f'Arm disabled: {response.message}'
                )
            else:
                self.get_logger().warning(
                    f'Disable failed: {response.message}'
                )

        except Exception as error:
            self.get_logger().error(
                f'Disable service error: {error}'
            )

    def stop_arm(self):
        if not self.stop_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().warning(
                '/arm/stop service is not available'
            )
            return

        request = Trigger.Request()

        future = self.stop_client.call_async(
            request
        )

        while rclpy.ok() and not future.done():
            time.sleep(0.01)

        try:

            response = future.result()

            if response.success:
                self.get_logger().info(
                    f'Arm stopped: {response.message}'
                )
            else:
                self.get_logger().warning(
                    f'Stop failed: {response.message}'
                )

        except Exception as error:
            self.get_logger().error(
                f'Stop service error: {error}'
            )


def main(args=None):

    rclpy.init(args=args)
    node = ArmCommandNode()

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )

    spin_thread.start()

    try:
        while rclpy.ok():

            user_input = input(
                'Enter 6 joint angles in degrees '
                '(q to quit, stop tp stop, disable to disable):'
            )

            if user_input.strip().lower() == 'q':
                break

            if user_input.strip().lower() == 'stop':
                node.stop_arm()
                continue

            if user_input.strip().lower() == 'disable':
                node.disable_arm()
                continue

            try:
                joint_degrees = [
                    float(value)
                    for value in user_input.split()
                ]

            except ValueError:
                node.get_logger().warning(
                    'Please enter valid numbers only. '
                )
                continue

            node.send_joint_command(
                joint_degrees
            )

    except KeyboardInterrupt:
        pass

    finally:
        rclpy.try_shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
