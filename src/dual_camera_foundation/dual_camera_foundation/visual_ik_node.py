"""Dry-run visual target inverse kinematics for Dummy V2."""

import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from ament_index_python.packages import (
    get_package_share_directory,
)

from urdf_parser_py.urdf import URDF

BASE_LINK = 'base_link'
TIP_LINK = 'tcp'

JOINT_NAMES = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
]

TOOL_APPROACH_AXIS_LOCAL = np.array([0.0, -1.0, 0.0])
ARM_JOINT_NAMES = tuple(f'joint{index}' for index in range(1, 7))


def rpy_to_rotation(roll, pitch, yaw):

    cr = math.cos(roll)
    sr = math.sin(roll)

    cp = math.cos(pitch)
    sp = math.sin(pitch)

    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ]
    )

    ry = np.array(
        [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ]
    )

    rz = np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    return rz @ ry @ rx


def axis_angle_rotation(axis, angle):

    axis = np.asarray(
        axis,
        dtype=np.float64,
    )

    norm = np.linalg.norm(axis)

    if norm < 1e-12:
        raise ValueError(
            'Joint axis has zero length'
        )

    axis = axis / norm

    x, y, z = axis

    skew = np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ]
    )

    identity = np.eye(3)

    return (
        identity
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle))
        * (skew @ skew)
    )


def transform_matrix(
    xyz=None,
    rpy=None,
):

    matrix = np.eye(4)

    if xyz is not None:

        matrix[:3, 3] = np.asarray(
            xyz,
            dtype=np.float64,
        )

    if rpy is not None:

        matrix[:3, :3] = (
            rpy_to_rotation(
                float(rpy[0]),
                float(rpy[1]),
                float(rpy[2]),
            )
        )

    return matrix


def quaternion_to_rotation(
    x,
    y,
    z,
    w,
):

    quaternion = np.array(
        [x, y, z, w],
        dtype=np.float64,
    )

    norm = np.linalg.norm(
        quaternion
    )

    if norm < 1e-12:
        raise ValueError(
            'Quaternion has zero length'
        )

    x, y, z, w = (
        quaternion / norm
    )

    return np.array(
        [
            [
                1.0 - 2.0 * (y*y + z*z),
                2.0 * (x*y - z*w),
                2.0 * (x*z + y*w),
            ],
            [
                2.0 * (x*y + z*w),
                1.0 - 2.0 * (x*x + z*z),
                2.0 * (y*z - x*w),
            ],
            [
                2.0 * (x*z - y*w),
                2.0 * (y*z + x*w),
                1.0 - 2.0 * (x*x + y*y),
            ],
        ]
    )


def rotation_log(rotation):

    value = (
        np.trace(rotation) - 1.0
    ) / 2.0

    value = np.clip(
        value,
        -1.0,
        1.0,
    )

    angle = math.acos(value)

    if angle < 1e-8:

        return np.array(
            [
                rotation[2, 1]
                - rotation[1, 2],

                rotation[0, 2]
                - rotation[2, 0],

                rotation[1, 0]
                - rotation[0, 1],
            ]
        ) / 2.0

    if abs(math.pi - angle) < 1e-4:

        diagonal = (
            np.diag(rotation)
            + 1.0
        ) / 2.0

        axis = np.sqrt(
            np.maximum(
                diagonal,
                0.0,
            )
        )

        if (
            rotation[2, 1]
            - rotation[1, 2]
            < 0.0
        ):
            axis[0] *= -1.0

        if (
            rotation[0, 2]
            - rotation[2, 0]
            < 0.0
        ):
            axis[1] *= -1.0

        if (
            rotation[1, 0]
            - rotation[0, 1]
            < 0.0
        ):
            axis[2] *= -1.0

        norm = np.linalg.norm(axis)

        if norm < 1e-8:
            return np.zeros(3)

        return (
            angle
            * axis
            / norm
        )

    factor = (
        angle
        / (
            2.0
            * math.sin(angle)
        )
    )

    return factor * np.array(
        [
            rotation[2, 1]
            - rotation[1, 2],

            rotation[0, 2]
            - rotation[2, 0],

            rotation[1, 0]
            - rotation[0, 1],
        ]
    )


class RobotKinematics:

    def __init__(
        self,
        urdf_path,
        base_link,
        tip_link,
        passive_joint_positions=None,
    ):

        self.robot = (
            URDF.from_xml_file(
                urdf_path
            )
        )

        self.base_link = base_link
        self.tip_link = tip_link

        self.chain = (
            self.find_chain()
        )

        self.arm_joint_names = ARM_JOINT_NAMES
        self.passive_joint_positions = dict(passive_joint_positions or {})
        self.active_joints = [
            joint
            for joint in self.chain
            if joint.name in self.arm_joint_names
        ]

        if tuple(joint.name for joint in self.active_joints) != (
            self.arm_joint_names
        ):

            raise RuntimeError(
                'Expected the six arm joints joint1 through joint6 in '
                f'the base-to-tip chain, got '
                f'{[joint.name for joint in self.active_joints]}'
            )

        self.lower_limits = []
        self.upper_limits = []
        self.continuous_joint_indices = tuple(
            index
            for index, joint in enumerate(self.active_joints)
            if joint.type == 'continuous'
        )

        for joint in (
            self.active_joints
        ):

            if (
                joint.type
                == 'continuous'
                or joint.limit is None
            ):

                self.lower_limits.append(
                    -math.pi
                )

                self.upper_limits.append(
                    math.pi
                )

            else:

                self.lower_limits.append(
                    float(
                        joint.limit.lower
                    )
                )

                self.upper_limits.append(
                    float(
                        joint.limit.upper
                    )
                )

        self.lower_limits = np.array(
            self.lower_limits
        )

        self.upper_limits = np.array(
            self.upper_limits
        )

        self.free_joint_indices = tuple(range(6))

    def set_passive_joint_position(self, joint_name, position):
        """Set a non-arm joint value used by forward kinematics only."""
        value = float(position)
        if not math.isfinite(value):
            raise ValueError('passive joint position must be finite')
        self.passive_joint_positions[str(joint_name)] = value

    def joints_within_limits(self, joints):
        values = np.asarray(joints, dtype=np.float64)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            return False
        continuous = set(self.continuous_joint_indices)
        return all(
            index in continuous
            or self.lower_limits[index] - 1.0e-9
            <= value
            <= self.upper_limits[index] + 1.0e-9
            for index, value in enumerate(values)
        )

    def find_chain(self):

        queue = deque(
            [
                (
                    self.base_link,
                    [],
                )
            ]
        )

        while queue:

            link_name, path = (
                queue.popleft()
            )

            if (
                link_name
                == self.tip_link
            ):

                return path

            children = (
                self.robot.child_map.get(
                    link_name,
                    [],
                )
            )

            for (
                joint_name,
                child_link,
            ) in children:

                joint = (
                    self.robot.joint_map[
                        joint_name
                    ]
                )

                queue.append(
                    (
                        child_link,
                        path + [joint],
                    )
                )

        raise RuntimeError(
            f'No URDF chain from '
            f'{self.base_link} '
            f'to {self.tip_link}'
        )

    def forward(self, q):

        transform = np.eye(4)

        active_index = 0

        for joint in self.chain:

            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]

            if joint.origin is not None:

                if (
                    joint.origin.xyz
                    is not None
                ):
                    xyz = joint.origin.xyz

                if (
                    joint.origin.rpy
                    is not None
                ):
                    rpy = joint.origin.rpy

            transform = (
                transform
                @ transform_matrix(
                    xyz,
                    rpy,
                )
            )

            if joint.name in self.arm_joint_names and joint.type in (
                'revolute',
                'continuous',
            ):

                rotation = (
                    axis_angle_rotation(
                        joint.axis,
                        q[active_index],
                    )
                )

                joint_transform = (
                    np.eye(4)
                )

                joint_transform[
                    :3,
                    :3,
                ] = rotation

                transform = (
                    transform
                    @ joint_transform
                )

                active_index += 1

            elif (
                joint.type
                == 'prismatic'
            ):

                axis = np.asarray(
                    joint.axis,
                    dtype=np.float64,
                )

                axis = (
                    axis
                    / np.linalg.norm(
                        axis
                    )
                )

                joint_transform = (
                    np.eye(4)
                )

                joint_transform[
                    :3,
                    3,
                ] = (
                    axis
                    * self.passive_joint_positions.get(joint.name, 0.0)
                )

                transform = (
                    transform
                    @ joint_transform
                )


        return transform

    def numerical_jacobian(
        self,
        q,
        epsilon=1e-6,
    ):

        current = self.forward(q)

        current_position = (
            current[:3, 3]
        )

        current_rotation = (
            current[:3, :3]
        )

        jacobian = np.zeros(
            (6, len(q))
        )

        for index in range(
            len(q)
        ):

            moved_q = q.copy()

            moved_q[index] += (
                epsilon
            )

            moved = self.forward(
                moved_q
            )

            moved_position = (
                moved[:3, 3]
            )

            moved_rotation = (
                moved[:3, :3]
            )

            jacobian[
                :3,
                index,
            ] = (
                moved_position
                - current_position
            ) / epsilon

            delta_rotation = (
                moved_rotation
                @ current_rotation.T
            )

            jacobian[
                3:,
                index,
            ] = (
                rotation_log(
                    delta_rotation
                )
                / epsilon
            )

        return jacobian

    @staticmethod
    def approach_direction(rotation):
        """Return the normalized world-frame TCP approach direction."""
        direction = np.asarray(rotation, dtype=np.float64) @ (
            TOOL_APPROACH_AXIS_LOCAL
        )
        norm = np.linalg.norm(direction)
        if not math.isfinite(norm) or norm < 1.0e-12:
            raise ValueError('Approach direction is invalid')
        return direction / norm

    def solve(
        self,
        target_position,
        target_rotation,
        seed,
    ):

        q = np.array(
            seed,
            dtype=np.float64,
        )

        q = np.clip(
            q,
            self.lower_limits,
            self.upper_limits,
        )
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError('IK seed must contain six finite joint values')

        target_position = np.asarray(target_position, dtype=np.float64)
        target_rotation = np.asarray(target_rotation, dtype=np.float64)
        if target_position.shape != (3,) or target_rotation.shape != (3, 3):
            raise ValueError(
                'IK target must contain a 3D position and rotation'
            )
        if not (
            np.all(np.isfinite(target_position))
            and np.all(np.isfinite(target_rotation))
        ):
            raise ValueError('IK target contains nan or infinity')
        target_approach = self.approach_direction(target_rotation)
        damping = 0.03

        # 角度误差按约 20 cm
        # 的等效长度参与优化。
        orientation_weight = 0.20

        for _ in range(150):

            current = self.forward(
                q
            )

            current_position = (
                current[:3, 3]
            )

            current_rotation = (
                current[:3, :3]
            )

            position_error = (
                target_position
                - current_position
            )

            full_rotation_error = (
                rotation_log(
                    target_rotation
                    @ current_rotation.T
                )
            )

            current_approach = self.approach_direction(current_rotation)
            approach_error_rad = math.acos(
                float(np.clip(
                    np.dot(current_approach, target_approach),
                    -1.0,
                    1.0,
                ))
            )
            position_norm = (
                np.linalg.norm(
                    position_error
                )
            )

            full_rotation_norm = (
                np.linalg.norm(
                    full_rotation_error
                )
            )

            if (
                position_norm
                < 0.0015
                and full_rotation_norm
                < math.radians(1.0)
            ):

                return (
                    True,
                    q,
                    position_norm,
                    math.degrees(
                        approach_error_rad
                    ),
                    math.degrees(
                        full_rotation_norm
                    ),
                )

            error = np.concatenate(
                [
                    position_error,
                    orientation_weight
                    * full_rotation_error,
                ]
            )

            jacobian = (
                self.numerical_jacobian(
                    q
                )
            )

            weighted_jacobian = jacobian.copy()

            weighted_jacobian[
                3:,
                :,
            ] *= orientation_weight

            matrix = (
                weighted_jacobian
                @ weighted_jacobian.T
                + damping**2
                * np.eye(6)
            )

            try:

                delta_q = (
                    weighted_jacobian.T
                    @ np.linalg.solve(
                        matrix,
                        error,
                    )
                )

            except (
                np.linalg.LinAlgError
            ):

                return (
                    False,
                    q,
                    position_norm,
                    math.degrees(
                        approach_error_rad
                    ),
                    math.degrees(
                        full_rotation_norm
                    ),
                )

            # 防止一次跳太大。
            delta_q = np.clip(
                delta_q,
                -0.12,
                0.12,
            )

            q[list(self.free_joint_indices)] += delta_q

            q = np.clip(
                q,
                self.lower_limits,
                self.upper_limits,
            )

        final = self.forward(q)

        final_position_error = (
            np.linalg.norm(
                target_position
                - final[:3, 3]
            )
        )

        final_rotation_error = (
            np.linalg.norm(
                rotation_log(
                    target_rotation
                    @ final[:3, :3].T
                )
            )
        )

        final_approach = self.approach_direction(final[:3, :3])
        final_approach_error = math.acos(
            float(np.clip(
                np.dot(final_approach, target_approach),
                -1.0,
                1.0,
            ))
        )

        return (
            False,
            q,
            final_position_error,
            math.degrees(
                final_approach_error
            ),
            math.degrees(
                final_rotation_error
            ),
        )


class VisualIKNode(Node):

    def __init__(self):

        super().__init__(
            'visual_ik_node'
        )

        share_directory = (
            get_package_share_directory(
                'arm_description'
            )
        )

        urdf_path = (
            share_directory
            + '/urdf/dummy-v2.urdf'
        )

        self.kinematics = (
            RobotKinematics(
                urdf_path,
                BASE_LINK,
                TIP_LINK,
            )
        )

        self.latest_joint_state = None

        self.solution_publisher = (
            self.create_publisher(
                JointState,
                '/perception/dry_run/ik_solution',
                10,
            )
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            '/perception/dry_run/pregrasp',
            self.target_callback,
            10,
        )

        self.get_logger().info(
            'Visual IK Node started\n'
            '  input  = '
            '/perception/dry_run/pregrasp\n'
            '  seed   = /joint_states\n'
            '  output = '
            '/perception/dry_run/ik_solution\n'
            '  hardware command = NONE'
        )

    def joint_state_callback(
        self,
        message,
    ):

        joint_map = dict(
            zip(
                message.name,
                message.position,
            )
        )

        if not all(
            name in joint_map
            for name in JOINT_NAMES
        ):
            return

        self.latest_joint_state = (
            np.array(
                [
                    joint_map[name]
                    for name
                    in JOINT_NAMES
                ],
                dtype=np.float64,
            )
        )

    def target_callback(
        self,
        message,
    ):

        if (
            message.header.frame_id
            != BASE_LINK
        ):

            self.get_logger().warning(
                'Target frame must be '
                f'{BASE_LINK}, got '
                f'{message.header.frame_id!r}'
            )

            return

        position = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=np.float64,
        )

        quaternion = (
            message.pose.orientation
        )

        try:

            target_rotation = (
                quaternion_to_rotation(
                    quaternion.x,
                    quaternion.y,
                    quaternion.z,
                    quaternion.w,
                )
            )

        except ValueError as error:

            self.get_logger().warning(
                str(error)
            )

            return

        top_grasp_seed = np.radians(
            [
                0.0,
                -42.9,
                -36.0,
                0.0,
                -13.9,
                0.0,
            ]
        )

        if self.latest_joint_state is None:

            reference_joint_state = (
                top_grasp_seed
            )

            seeds = [
                (
                    'top_grasp',
                    top_grasp_seed,
                )
            ]

        else:

            reference_joint_state = (
                self.latest_joint_state.copy()
            )
            seeds = [
                (
                    'current',
                    self.latest_joint_state,
                ),
                (
                    'top_grasp',
                    top_grasp_seed,
                ),
            ]
        # J1 再给两个不同分支，
        # 避免一个初值卡在局部解。
        seed_positive = (
            top_grasp_seed.copy()
        )

        seed_positive[0] = (
            math.radians(35.0)
        )

        seeds.append(
            (
                'top_grasp_j1_plus',
                seed_positive,
            )
        )

        seed_negative = (
            top_grasp_seed.copy()
        )

        seed_negative[0] = (
            math.radians(-35.0)
        )

        seeds.append(
            (
                'top_grasp_j1_minus',
                seed_negative,
            )
        )

        candidates = []

        for (
            seed_name,
            seed,
        ) in seeds:

            (
                success,
                solution,
                position_error,
                approach_error_deg,
                full_rotation_error_deg,
            ) = self.kinematics.solve(
                position,
                target_rotation,
                seed,
            )

            if not success:
                continue

            distance_from_current = (
                np.linalg.norm(
                    solution
                    - reference_joint_state
                )
            )

            candidates.append(
                (
                    distance_from_current,
                    seed_name,
                    solution,
                    position_error,
                    approach_error_deg,
                    full_rotation_error_deg,
                )
            )

        if not candidates:

            self.get_logger().warning(
                '\n=== REAL VISUAL IK FAILED ===\n'
                f'  target xyz = '
                f'[{position[0]:.3f}, '
                f'{position[1]:.3f}, '
                f'{position[2]:.3f}] m\n'
                '  no seed converged\n'
                '  NO HARDWARE COMMAND SENT'
            )

            return

        candidates.sort(
            key=lambda item: item[0]
        )

        (
            _,
            seed_name,
            solution,
            position_error,
            approach_error_deg,
            full_rotation_error_deg,
        ) = candidates[0]

        output = JointState()

        output.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        output.name = list(
            JOINT_NAMES
        )

        output.position = [
            float(value)
            for value in solution
        ]

        self.solution_publisher.publish(
            output
        )

        solution_deg = [
            math.degrees(value)
            for value in solution
        ]

        self.get_logger().info(
            '\n'
            '==============================\n'
            'REAL VISUAL IK SUCCESS\n'
            '==============================\n'
            f'target xyz m = '
            f'[{position[0]:.3f}, '
            f'{position[1]:.3f}, '
            f'{position[2]:.3f}]\n'
            f'seed = {seed_name}\n'
            f'solution deg = '
            f'{[round(v, 2) for v in solution_deg]}\n'
            f'position error = '
            f'{position_error * 1000.0:.2f} mm\n'
            f'approach direction error = '
            f'{approach_error_deg:.2f} deg\n'
            f'full orientation residual = '
            f'{full_rotation_error_deg:.2f} deg '
            '(tool-axis roll is relaxed)\n'
            'J6 hard lock = 0 rad\n'
            'NO HARDWARE COMMAND SENT'
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = VisualIKNode()

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
