import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    TransformStamped
)
from message_filters import (
    Subscriber,
    ApproximateTimeSynchronizer
)
from camera_perception.apriltag_detector import (
    create_apriltag_detector,
    detect_apriltags
)
from camera_perception.depth_image import (
    get_valid_depth_near_pixel
)
from camera_perception.camera_geometry import (
    intrinsics_from_camera_info,
    pixel_to_camera_point
)
from camera_perception.tag_pose import (
    estimate_tag_pose,
    rotation_matrix_to_quaternion,
    TAG_SIZES
)


class TagLocalizerNode(Node):

    def __init__(self):

        super().__init__(
            'tag_localizer_node'
        )


        self.declare_parameter(
            'rgb_topic',
            '/camera/color/image_raw'
        )

        self.declare_parameter(
            'depth_topic',
            '/camera/depth/image_raw'
        )

        self.declare_parameter(
            'camera_info_topic',
            '/camera/depth/camera_info'
        )

        self.declare_parameter(
            'target_tag_id',
            6
        )

        self.declare_parameter(
            'depth_search_radius',
            2
        )

        self.declare_parameter(
            'color_info_topic',
            '/camera/color/camera_info'
        )

        self.rgb_topic = (
            self.get_parameter(
                'rgb_topic'
            )
            .get_parameter_value()
            .string_value
        )

        self.depth_topic = (
            self.get_parameter(
                'depth_topic'
            )
            .get_parameter_value()
            .string_value
        )

        self.camera_info_topic = (
            self.get_parameter(
                'camera_info_topic'
            )
            .get_parameter_value()
            .string_value
        )

        self.target_tag_id = (
            self.get_parameter(
                'target_tag_id'
            )
            .get_parameter_value()
            .integer_value
        )

        self.depth_search_radius = (
            self.get_parameter(
                'depth_search_radius'
            )
            .get_parameter_value()
            .integer_value
        )

        self.color_info_topic = (
            self.get_parameter(
                'color_info_topic'
            )
            .get_parameter_value()
            .string_value
        )

        self.bridge = CvBridge()

        self.apriltag_detector = (
            create_apriltag_detector()
        )


        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_frame = None

        self.camera_info_received = False

        self.color_camera_matrix = None
        self.color_distortion = None

        self.localized_count = 0


        self.camera_info_subscription = (
            self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.camera_info_callback,
                10
            )
        )

        self.color_info_subscription = (
            self.create_subscription(
                CameraInfo,
                self.color_info_topic,
                self.color_info_callback,
                10
            )
        )

        self.rgb_subscription = Subscriber(
            self,
            Image,
            self.rgb_topic
        )

        self.depth_subscription = Subscriber(
            self,
            Image,
            self.depth_topic
        )


        self.synchronizer = (
            ApproximateTimeSynchronizer(
                [
                    self.rgb_subscription,
                    self.depth_subscription
                ],
                queue_size=10,
                slop=0.05
            )
        )

        self.synchronizer.registerCallback(
            self.synced_callback
        )


        self.point_publisher = (
            self.create_publisher(
                PointStamped,
                '/camera/tag_point',
                10
            )
        )

        self.pose_publisher = (
            self.create_publisher(
                PoseStamped,
                '/camera/tag_pose',
                10
            )
        )

        self.tf_broadcaster = (
            TransformBroadcaster(
                self
            )
        )

        self.get_logger().info(
            'Tag Localizer Node started\n'
            f'  rgb_topic   = '
            f'{self.rgb_topic}\n'
            f'  depth_topic = '
            f'{self.depth_topic}\n'
            f'  target_tag  = '
            f'{self.target_tag_id}'
        )


    def camera_info_callback(
        self,
        msg
    ):

        intrinsics = (
            intrinsics_from_camera_info(
                msg
            )
        )

        if intrinsics is None:

            return


        (
            self.fx,
            self.fy,
            self.cx,
            self.cy,
            source
        ) = intrinsics


        self.camera_frame = (
            msg.header.frame_id
        )


        if not self.camera_info_received:

            self.camera_info_received = True

            self.get_logger().info(
                'Camera intrinsics ready\n'
                f'  source = {source}\n'
                f'  fx = {self.fx:.3f}\n'
                f'  fy = {self.fy:.3f}\n'
                f'  cx = {self.cx:.3f}\n'
                f'  cy = {self.cy:.3f}\n'
                f'  frame = '
                f'{self.camera_frame}'
            )


    def synced_callback(
        self,
        rgb_msg,
        depth_msg
    ):

        if not self.camera_info_received:
            return

        if self.color_camera_matrix is None:
            return


        if (
            rgb_msg.width != depth_msg.width
            or rgb_msg.height != depth_msg.height
        ):

            self.get_logger().warning(
                'RGB and depth image sizes '
                'do not match'
            )

            return


        try:

            frame = (
                self.bridge.imgmsg_to_cv2(
                    rgb_msg,
                    desired_encoding='bgr8'
                )
            )

        except Exception as error:

            self.get_logger().error(
                f'RGB conversion failed: '
                f'{error}'
            )

            return


        detections = detect_apriltags(
            frame,
            self.apriltag_detector
        )


        for detection in detections:

            tag_id = detection['id']


            # 只处理我们认识的 Tag：
            # ID0 / ID2 / ID6
            tag_size_m = TAG_SIZES.get(
                tag_id
            )

            if tag_size_m is None:
                continue


            u, v = detection['center']


            pose = estimate_tag_pose(
                detection['corners'],
                tag_size_m,
                self.color_camera_matrix,
                self.color_distortion
            )

            if pose is None:
                continue


            rvec = pose['rvec']

            tvec = pose['tvec']

            rotation_matrix = pose[
                'rotation_matrix'
            ]


            quaternion = (
                rotation_matrix_to_quaternion(
                    rotation_matrix
                )
            )


            #
            # 1. 画每一个 Tag 的坐标轴
            #

            cv2.drawFrameAxes(
                frame,
                self.color_camera_matrix,
                self.color_distortion,
                rvec,
                tvec,
                tag_size_m * 0.4
            )


            cv2.putText(
                frame,
                f'ID {tag_id}',
                (
                    u + 10,
                    v - 10
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )


            #
            # 2. 广播 camera -> tag TF
            #

            transform_msg = (
                TransformStamped()
            )


            transform_msg.header.stamp = (
                rgb_msg.header.stamp
            )

            transform_msg.header.frame_id = (
                rgb_msg.header.frame_id
            )


            transform_msg.child_frame_id = (
                f'tag_{tag_id}'
            )


            transform_msg.transform.translation.x = (
                float(tvec[0, 0])
            )

            transform_msg.transform.translation.y = (
                float(tvec[1, 0])
            )

            transform_msg.transform.translation.z = (
                float(tvec[2, 0])
            )


            transform_msg.transform.rotation.x = (
                float(quaternion[0])
            )

            transform_msg.transform.rotation.y = (
                float(quaternion[1])
            )

            transform_msg.transform.rotation.z = (
                float(quaternion[2])
            )

            transform_msg.transform.rotation.w = (
                float(quaternion[3])
            )


            self.tf_broadcaster.sendTransform(
                transform_msg
            )


            #
            # 3. /camera/tag_pose
            #
            # 这个 topic 暂时仍然只发布
            # target_tag_id
            #

            if tag_id != self.target_tag_id:
                continue


            pose_msg = PoseStamped()

            pose_msg.header = rgb_msg.header


            pose_msg.pose.position.x = (
                float(tvec[0, 0])
            )

            pose_msg.pose.position.y = (
                float(tvec[1, 0])
            )

            pose_msg.pose.position.z = (
                float(tvec[2, 0])
            )


            pose_msg.pose.orientation.x = (
                float(quaternion[0])
            )

            pose_msg.pose.orientation.y = (
                float(quaternion[1])
            )

            pose_msg.pose.orientation.z = (
                float(quaternion[2])
            )

            pose_msg.pose.orientation.w = (
                float(quaternion[3])
            )


            self.pose_publisher.publish(
                pose_msg
            )


            #
            # 4. target Tag 才继续做 RGB-D 深度定位
            #

            depth_m = (
                get_valid_depth_near_pixel(
                    depth_msg,
                    u,
                    v,
                    radius=(
                        self.depth_search_radius
                    )
                )
            )

            if depth_m is None:
                continue


            point = pixel_to_camera_point(
                u,
                v,
                depth_m,
                self.fx,
                self.fy,
                self.cx,
                self.cy
            )

            if point is None:
                continue


            x, y, z = point


            point_msg = PointStamped()

            point_msg.header = (
                depth_msg.header
            )

            point_msg.point.x = x
            point_msg.point.y = y
            point_msg.point.z = z


            self.point_publisher.publish(
                point_msg
            )


            self.localized_count += 1


            if (
                self.localized_count
                % 30 == 0
            ):

                rgb_time = (
                    rgb_msg.header.stamp.sec
                    +
                    rgb_msg.header.stamp.nanosec
                    * 1e-9
                )

                depth_time = (
                    depth_msg.header.stamp.sec
                    +
                    depth_msg.header.stamp.nanosec
                    * 1e-9
                )

                time_difference_ms = (
                    abs(
                        rgb_time
                        - depth_time
                    )
                    * 1000.0
                )


                self.get_logger().info(
                    f'Tag ID {tag_id}\n'
                    f'  pixel = ({u}, {v})\n'
                    f'\n'
                    f'PnP:\n'
                    f'  X = '
                    f'{tvec[0, 0]:.3f} m\n'
                    f'  Y = '
                    f'{tvec[1, 0]:.3f} m\n'
                    f'  Z = '
                    f'{tvec[2, 0]:.3f} m\n'
                    f'\n'
                    f'Depth:\n'
                    f'  X = {x:.3f} m\n'
                    f'  Y = {y:.3f} m\n'
                    f'  Z = {z:.3f} m\n'
                    f'\n'
                    f'RGB-depth dt = '
                    f'{time_difference_ms:.2f} ms'
                )


        #
        # 注意：imshow 一定在 for 循环外面
        #
        # 这样一张 frame 上会同时画出
        # ID0 / ID2 / ID6
        #

        cv2.imshow(
            'Tag Pose',
            frame
        )

        cv2.waitKey(1)

    def color_info_callback(self, msg):

        if self.color_camera_matrix is not None:
            return

        self.color_camera_matrix = np.array(
            msg.k,
            dtype=np.float64
        ).reshape(3, 3)

        self.color_distortion = np.array(
            msg.d,
            dtype=np.float64
        )

        self.get_logger().info(
            'Color CameraInfo ready\n'
            f'{self.color_camera_matrix}'
        )

def main(args=None):

    rclpy.init(args=args)

    node = TagLocalizerNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        cv2.destroyAllWindows()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()