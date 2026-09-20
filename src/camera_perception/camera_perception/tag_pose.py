import cv2
import numpy as np

TAG_SIZES = {
    0: 0.050,
    2: 0.030,
    6: 0.100,
    3: 0.030,
}

def estimate_tag_pose(
    corners,
    tag_size_m,
    camera_matrix,
    distortion
):

    half_size = tag_size_m / 2.0

    object_points = np.array(
        [
            [-half_size,  half_size, 0.0],
            [ half_size,  half_size, 0.0],
            [ half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ],
        dtype=np.float32
    )

    image_points = np.asarray(
        corners,
        dtype=np.float32
    ).reshape(4, 2)

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE
    )

    if not success:
        return None

    rotation_matrix, _ = cv2.Rodrigues(
        rvec
    )

    return {
        'rvec': rvec,
        'tvec': tvec,
        'rotation_matrix': rotation_matrix
    }

def rotation_matrix_to_quaternion(
    rotation_matrix
):

    r = rotation_matrix

    trace = (
        r[0, 0]
        + r[1, 1]
        + r[2, 2]
    )

    if trace > 0.0:

        s = (
            np.sqrt(trace + 1.0)
            * 2.0
        )

        qw = 0.25 * s

        qx = (
            r[2, 1] - r[1, 2]
        ) / s

        qy = (
            r[0, 2] - r[2, 0]
        ) / s

        qz = (
            r[1, 0] - r[0, 1]
        ) / s

    elif (
        r[0, 0] > r[1, 1]
        and r[0, 0] > r[2, 2]
    ):

        s = (
            np.sqrt(
                1.0
                + r[0, 0]
                - r[1, 1]
                - r[2, 2]
            )
            * 2.0
        )

        qw = (
            r[2, 1] - r[1, 2]
        ) / s

        qx = 0.25 * s

        qy = (
            r[0, 1] + r[1, 0]
        ) / s

        qz = (
            r[0, 2] + r[2, 0]
        ) / s

    elif r[1, 1] > r[2, 2]:

        s = (
            np.sqrt(
                1.0
                + r[1, 1]
                - r[0, 0]
                - r[2, 2]
            )
            * 2.0
        )

        qw = (
            r[0, 2] - r[2, 0]
        ) / s

        qx = (
            r[0, 1] + r[1, 0]
        ) / s

        qy = 0.25 * s

        qz = (
            r[1, 2] + r[2, 1]
        ) / s

    else:

        s = (
            np.sqrt(
                1.0
                + r[2, 2]
                - r[0, 0]
                - r[1, 1]
            )
            * 2.0
        )

        qw = (
            r[1, 0] - r[0, 1]
        ) / s

        qx = (
            r[0, 2] + r[2, 0]
        ) / s

        qy = (
            r[1, 2] + r[2, 1]
        ) / s

        qz = 0.25 * s

    quaternion = np.array(
        [
            qx,
            qy,
            qz,
            qw
        ],
        dtype=np.float64
    )

    quaternion /= np.linalg.norm(
        quaternion
    )

    return quaternion