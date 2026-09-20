import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped

from tf2_ros import (
    Buffer,
    TransformListener,
    StaticTransformBroadcaster
)

from camera_perception.tag_pose import (
    rotation_matrix_to_quaternion
)
from camera_perception.camera_extrinsics import (
    CameraExtrinsics,
    save_camera_extrinsics
)


def transform_to_matrix(transform):

    x = transform.rotation.x
    y = transform.rotation.y
    z = transform.rotation.z
    w = transform.rotation.w

    norm = np.sqrt(
        x * x
        + y * y
        + z * z
        + w * w
    )

    if norm == 0.0:
        return None

    x /= norm
    y /= norm
    z /= norm
    w /= norm


    rotation = np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w)
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w)
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y)
            ]
        ],
        dtype=np.float64
    )


    matrix = np.eye(
        4,
        dtype=np.float64
    )

    matrix[:3, :3] = rotation

    matrix[0, 3] = (
        transform.translation.x
    )

    matrix[1, 3] = (
        transform.translation.y
    )

    matrix[2, 3] = (
        transform.translation.z
    )

    return matrix


def average_transforms(
    transforms
):

    translations = [
        transform[:3, 3]
        for transform in transforms
    ]

    translation = np.mean(
        translations,
        axis=0
    )


    rotation_average = np.mean(
        [
            transform[:3, :3]
            for transform in transforms
        ],
        axis=0
    )


    u, _, vh = np.linalg.svd(
        rotation_average
    )

    rotation = u @ vh


    if np.linalg.det(rotation) < 0.0:

        u[:, -1] *= -1.0

        rotation = u @ vh


    result = np.eye(
        4,
        dtype=np.float64
    )

    result[:3, :3] = rotation
    result[:3, 3] = translation

    return result


class CameraExtrinsicCalibratorNode(Node):

    def __init__(self):

        super().__init__(
            'camera_extrinsic_calibrator_node'
        )


        self.declare_parameter(
            'base_frame',
            'base_link'
        )

        self.declare_parameter(
            'tag_ref_frame',
            'tag0_ref'
        )

        self.declare_parameter(
            'camera_optical_frame',
            'camera_color_optical_frame'
        )

        self.declare_parameter(
            'detected_tag_frame',
            'tag_0'
        )

        self.declare_parameter(
            'camera_root_frame',
            'camera_link'
        )

        self.declare_parameter(
            'sample_count',
            60
        )

        self.declare_parameter(
            'save_extrinsics',
            False
        )

        self.declare_parameter(
            'extrinsics_file',
            ''
        )

        self.declare_parameter(
            'overwrite_existing',
            False
        )


        self.base_frame = (
            self.get_parameter(
                'base_frame'
            )
            .get_parameter_value()
            .string_value
        )

        self.tag_ref_frame = (
            self.get_parameter(
                'tag_ref_frame'
            )
            .get_parameter_value()
            .string_value
        )

        self.camera_optical_frame = (
            self.get_parameter(
                'camera_optical_frame'
            )
            .get_parameter_value()
            .string_value
        )

        self.detected_tag_frame = (
            self.get_parameter(
                'detected_tag_frame'
            )
            .get_parameter_value()
            .string_value
        )

        self.camera_root_frame = (
            self.get_parameter(
                'camera_root_frame'
            )
            .get_parameter_value()
            .string_value
        )

        self.sample_count = (
            self.get_parameter(
                'sample_count'
            )
            .get_parameter_value()
            .integer_value
        )

        self.save_extrinsics = (
            self.get_parameter(
                'save_extrinsics'
            )
            .get_parameter_value()
            .bool_value
        )

        self.extrinsics_file = (
            self.get_parameter(
                'extrinsics_file'
            )
            .get_parameter_value()
            .string_value
        )

        self.overwrite_existing = (
            self.get_parameter(
                'overwrite_existing'
            )
            .get_parameter_value()
            .bool_value
        )


        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.static_broadcaster = (
            StaticTransformBroadcaster(
                self
            )
        )


        self.samples = []

        self.last_tag_stamp = None

        self.calibration_complete = False

        self.waiting_message_logged = False


        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )


        self.get_logger().info(
            'Camera Extrinsic Calibrator started\n'
            f'  base       = {self.base_frame}\n'
            f'  tag ref    = {self.tag_ref_frame}\n'
            f'  camera     = {self.camera_optical_frame}\n'
            f'  detected   = {self.detected_tag_frame}\n'
            f'  camera root= {self.camera_root_frame}\n'
            f'  samples    = {self.sample_count}\n'
            f'  save       = {self.save_extrinsics}\n'
            f'  output file= {self.extrinsics_file or "(print only)"}\n'
            f'  overwrite  = {self.overwrite_existing}'
        )


    def timer_callback(self):

        if self.calibration_complete:
            return


        try:

            base_to_tag = (
                self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.tag_ref_frame,
                    Time()
                )
            )


            camera_to_tag = (
                self.tf_buffer.lookup_transform(
                    self.camera_optical_frame,
                    self.detected_tag_frame,
                    Time()
                )
            )


            optical_to_root = (
                self.tf_buffer.lookup_transform(
                    self.camera_optical_frame,
                    self.camera_root_frame,
                    Time()
                )
            )


        except Exception as error:

            if not self.waiting_message_logged:

                self.waiting_message_logged = True

                self.get_logger().warning(
                    'Waiting for required TFs...\n'
                    f'  {self.base_frame}'
                    f' -> {self.tag_ref_frame}\n'
                    f'  {self.camera_optical_frame}'
                    f' -> {self.detected_tag_frame}\n'
                    f'  {self.camera_optical_frame}'
                    f' -> {self.camera_root_frame}\n'
                    f'  reason: {error}'
                )

            return


        self.waiting_message_logged = False


        tag_stamp = (
            camera_to_tag.header.stamp.sec,
            camera_to_tag.header.stamp.nanosec
        )


        if tag_stamp == self.last_tag_stamp:
            return

        self.last_tag_stamp = tag_stamp


        base_to_tag_matrix = (
            transform_to_matrix(
                base_to_tag.transform
            )
        )

        camera_to_tag_matrix = (
            transform_to_matrix(
                camera_to_tag.transform
            )
        )

        optical_to_root_matrix = (
            transform_to_matrix(
                optical_to_root.transform
            )
        )


        if (
            base_to_tag_matrix is None
            or camera_to_tag_matrix is None
            or optical_to_root_matrix is None
        ):
            return


        #
        # T_base_optical
        #
        # base -> tag
        # multiplied by
        # tag -> optical
        #

        base_to_optical_matrix = (
            base_to_tag_matrix
            @ np.linalg.inv(
                camera_to_tag_matrix
            )
        )


        #
        # camera optical -> camera root
        #
        # therefore:
        #
        # T_base_root
        # =
        # T_base_optical
        # *
        # T_optical_root
        #

        base_to_root_matrix = (
            base_to_optical_matrix
            @ optical_to_root_matrix
        )


        self.samples.append(
            base_to_root_matrix
        )


        current_count = len(
            self.samples
        )


        if (
            current_count == 1
            or current_count % 10 == 0
        ):

            self.get_logger().info(
                f'Calibration samples: '
                f'{current_count}/'
                f'{self.sample_count}'
            )


        if current_count < self.sample_count:
            return


        average_matrix = (
            average_transforms(
                self.samples
            )
        )


        translation = (
            average_matrix[:3, 3]
        )

        rotation_matrix = (
            average_matrix[:3, :3]
        )

        quaternion = (
            rotation_matrix_to_quaternion(
                rotation_matrix
            )
        )


        transform_msg = TransformStamped()

        transform_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        transform_msg.header.frame_id = (
            self.base_frame
        )

        transform_msg.child_frame_id = (
            self.camera_root_frame
        )


        transform_msg.transform.translation.x = (
            float(translation[0])
        )

        transform_msg.transform.translation.y = (
            float(translation[1])
        )

        transform_msg.transform.translation.z = (
            float(translation[2])
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


        self.static_broadcaster.sendTransform(
            transform_msg
        )


        self.calibration_complete = True


        self.get_logger().info(
            'Camera extrinsic calibration complete\n'
            f'  parent = {self.base_frame}\n'
            f'  child  = {self.camera_root_frame}\n'
            f'  xyz = '
            f'[{translation[0]:.4f}, '
            f'{translation[1]:.4f}, '
            f'{translation[2]:.4f}] m\n'
            f'  quaternion = '
            f'[{quaternion[0]:.5f}, '
            f'{quaternion[1]:.5f}, '
            f'{quaternion[2]:.5f}, '
            f'{quaternion[3]:.5f}]'
        )


        if self.save_extrinsics:

            try:

                extrinsics = CameraExtrinsics.from_values(
                    self.base_frame,
                    self.camera_root_frame,
                    translation,
                    quaternion
                )

                saved_path = save_camera_extrinsics(
                    self.extrinsics_file,
                    extrinsics,
                    overwrite=self.overwrite_existing
                )

                self.get_logger().info(
                    'Saved camera extrinsics\n'
                    f'  file = {saved_path}'
                )

            except FileExistsError as error:

                self.get_logger().error(
                    'Camera extrinsics were NOT saved because the output '
                    'already exists. Set overwrite_existing:=true only after '
                    'confirming the camera was recalibrated intentionally. '
                    f'{error}'
                )

            except (OSError, TypeError, ValueError) as error:

                self.get_logger().error(
                    f'Camera extrinsics were NOT saved: {error}'
                )


def main(args=None):

    rclpy.init(args=args)

    node = CameraExtrinsicCalibratorNode()

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
