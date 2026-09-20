import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from camera_perception.depth_image import get_depth_at_pixel
from camera_perception.camera_geometry import pixel_to_camera_point

class DepthMonitorNode(Node):

    def __init__(self):
        super().__init__('depth_monitor_node')

        self.declare_parameter(
            'depth_topic',
            '/camera/depth/image_raw'
        )

        self.declare_parameter(
            'depth_info_topic',
            '/camera/depth/camera_info'
        )

        self.depth_topic = (
            self.get_parameter('depth_topic')
            .get_parameter_value()
            .string_value
        )

        self.depth_info_topic = (
            self.get_parameter('depth_info_topic')
            .get_parameter_value()
            .string_value
        )

        self.frame_count = 0

        self.first_depth_frame_received = False
        self.camera_info_received = False
        self.first_3d_point_logged = False

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.depth_info_topic,
            self.camera_info_callback,
            10
        )

        self.get_logger().info(
            'Depth Monitor Node started\n'
            f'  depth_topic = {self.depth_topic}\n'
            f'  info_topic  = {self.depth_info_topic}'
        )


    def camera_info_callback(self, msg):

        k_intrinsics = (
            msg.k[0],
            msg.k[4],
            msg.k[2],
            msg.k[5]
        )

        p_intrinsics = (
            msg.p[0],
            msg.p[5],
            msg.p[2],
            msg.p[6]
        )

        if all(
            math.isfinite(value)
            for value in k_intrinsics
        ):

            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

            intrinsics_source = 'K'

        elif all(
            math.isfinite(value)
            for value in p_intrinsics
        ):

            self.fx = msg.p[0]
            self.fy = msg.p[5]
            self.cx = msg.p[2]
            self.cy = msg.p[6]

            intrinsics_source = 'P'

        else:

            self.get_logger().warning(
                'CameraInfo contains no valid intrinsics'
            )

            return


        if not self.camera_info_received:

            self.camera_info_received = True

            self.get_logger().info(
                'Depth CameraInfo received\n'
                f'  source = {intrinsics_source}\n'
                f'  fx = {self.fx:.3f}\n'
                f'  fy = {self.fy:.3f}\n'
                f'  cx = {self.cx:.3f}\n'
                f'  cy = {self.cy:.3f}\n'
                f'  frame_id = {msg.header.frame_id}'
            )


    def depth_callback(self, msg):

        self.frame_count += 1

        if not self.first_depth_frame_received:

            self.first_depth_frame_received = True

            self.get_logger().info(
                'First depth frame received\n'
                f'  size     = {msg.width} x {msg.height}\n'
                f'  encoding = {msg.encoding}\n'
                f'  step     = {msg.step} bytes\n'
                f'  frame_id = {msg.header.frame_id}\n'
                f'  stamp    = '
                f'{msg.header.stamp.sec}.'
                f'{msg.header.stamp.nanosec:09d}'
            )


        center_u = msg.width // 2
        center_v = msg.height // 2

        depth_m = get_depth_at_pixel(
            msg,
            center_u,
            center_v
        )

        if (
            depth_m is not None
            and self.camera_info_received
            and not self.first_3d_point_logged
        ):

            point = pixel_to_camera_point(
                center_u,
                center_v,
                depth_m,
                self.fx,
                self.fy,
                self.cx,
                self.cy
            )

            if point is not None:

                x, y, z = point

                self.first_3d_point_logged = True

                self.get_logger().info(
                    'Center 3D point\n'
                    f'  pixel = '
                    f'({center_u}, {center_v})\n'
                    f'  depth = {depth_m:.3f} m\n'
                    f'  X = {x:.3f} m\n'
                    f'  Y = {y:.3f} m\n'
                    f'  Z = {z:.3f} m\n'
                    f'  frame_id = '
                    f'{msg.header.frame_id}'
                )


        if self.frame_count % 30 == 0:

            self.get_logger().info(
                f'Depth frames received: '
                f'{self.frame_count}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = DepthMonitorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()