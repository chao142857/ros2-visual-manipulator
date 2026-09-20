import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class GripperCommandNode(Node):

    def __init__(self):
        super().__init__('gripper_command_node')

        self.publisher = self.create_publisher(
            Int32,
            '/gripper/pulse_us',
            10
        )

        self.get_logger().info(
            'Gripper Command Node started'
        )

    def send_pulse(self, pulse_us):

        msg = Int32()

        msg.data = pulse_us

        self.publisher.publish(
            msg
        )

        self.get_logger().info(
            f'Sent gripper pulse: {pulse_us} us'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GripperCommandNode()

    while rclpy.ok():

        user_input = input(
            'Enter command: '
            'open, close, grab <pulse>, or q to quit:'
        )

        command = user_input.lower().strip()

        if command == 'q':
            break

        if command == 'open':
            pulse_us = 600

        elif command == 'close':
            pulse_us = 1300

        elif command.startswith('grab '):

            try:
                pulse_us = int(
                    command.split()[1]
                )

            except (ValueError, IndexError):
                node.get_logger().warning(
                    'Usage: grab <pulse>'
                )
                continue

        else:

            try:
                pulse_us = int(command)

            except ValueError:
                node.get_logger().warning(
                    'Enter open, close, grab <pulse>, '
                    'or a pulse value.'
                )
                continue

        if pulse_us < 600 or pulse_us > 1700:

            node.get_logger().warning(
                f'Pulse out of range: {pulse_us} us. '
                'Valid range is 600-1700 us.'
            )
            continue

        node.send_pulse(
            pulse_us
        )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
