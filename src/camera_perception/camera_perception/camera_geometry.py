import math


def intrinsics_from_camera_info(msg):

    k_intrinsics = (
        msg.k[0],
        msg.k[4],
        msg.k[2],
        msg.k[5]
    )

    if (
        all(
            math.isfinite(value)
            for value in k_intrinsics
        )
        and msg.k[0] != 0.0
        and msg.k[4] != 0.0
    ):
        return (
            msg.k[0],
            msg.k[4],
            msg.k[2],
            msg.k[5],
            'K'
        )


    p_intrinsics = (
        msg.p[0],
        msg.p[5],
        msg.p[2],
        msg.p[6]
    )

    if (
        all(
            math.isfinite(value)
            for value in p_intrinsics
        )
        and msg.p[0] != 0.0
        and msg.p[5] != 0.0
    ):
        return (
            msg.p[0],
            msg.p[5],
            msg.p[2],
            msg.p[6],
            'P'
        )


    return None


def pixel_to_camera_point(
    u,
    v,
    depth_m,
    fx,
    fy,
    cx,
    cy
):

    values = (
        depth_m,
        fx,
        fy,
        cx,
        cy
    )

    if not all(
        math.isfinite(value)
        for value in values
    ):
        return None

    if (
        depth_m <= 0.0
        or fx == 0.0
        or fy == 0.0
    ):
        return None

    x = (
        (u - cx)
        * depth_m
        / fx
    )

    y = (
        (v - cy)
        * depth_m
        / fy
    )

    z = depth_m

    return x, y, z