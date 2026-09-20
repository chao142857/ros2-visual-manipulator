"""Publish a saved fixed-camera extrinsic on tf_static."""

import math

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from camera_perception.camera_extrinsics import CameraExtrinsics


class CameraExtrinsicsPublisherNode(Node):
    """Load validated extrinsics from parameters and broadcast them once."""

    def __init__(self):
        """Initialize the fail-closed runtime extrinsics publisher."""
        super().__init__('camera_extrinsics_publisher_node')

        self.declare_parameter('calibrated', False)
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'camera_link')
        self.declare_parameter('xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('quaternion', [0.0, 0.0, 0.0, 1.0])

        if not self.get_parameter('calibrated').value:
            message = (
                'Camera extrinsics are not marked calibrated; refusing to '
                'publish a fixed transform'
            )
            self.get_logger().fatal(message)
            raise RuntimeError(message)

        original_quaternion = tuple(
            self.get_parameter('quaternion').value
        )
        try:
            self.extrinsics = CameraExtrinsics.from_values(
                self.get_parameter('parent_frame').value,
                self.get_parameter('child_frame').value,
                self.get_parameter('xyz').value,
                original_quaternion,
            )
        except ValueError as error:
            self.get_logger().fatal(f'Invalid camera extrinsics: {error}')
            raise RuntimeError('Invalid camera extrinsics') from error

        quaternion_norm = math.sqrt(
            sum(value * value for value in original_quaternion)
        )
        if abs(quaternion_norm - 1.0) > 1.0e-6:
            self.get_logger().warning(
                'Saved camera quaternion was not unit length and has been '
                f'normalized (norm={quaternion_norm:.9f})'
            )

        self.static_broadcaster = StaticTransformBroadcaster(self)
        transform = self.extrinsics.to_transform_stamped(
            self.get_clock().now().to_msg()
        )
        self.static_broadcaster.sendTransform(transform)

        xyz = self.extrinsics.xyz
        quaternion = self.extrinsics.quaternion
        self.get_logger().info(
            'Published saved camera extrinsic on /tf_static\n'
            f'  parent = {self.extrinsics.parent_frame}\n'
            f'  child  = {self.extrinsics.child_frame}\n'
            f'  xyz = [{xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}] m\n'
            '  quaternion = '
            f'[{quaternion[0]:.8f}, {quaternion[1]:.8f}, '
            f'{quaternion[2]:.8f}, {quaternion[3]:.8f}]'
        )


def main(args=None):
    """Run the saved-extrinsics publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraExtrinsicsPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
