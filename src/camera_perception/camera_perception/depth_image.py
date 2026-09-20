import math
import struct
import statistics


def get_depth_at_pixel(msg, u, v):

    if (
        u < 0
        or u >= msg.width
        or v < 0
        or v >= msg.height
    ):
        return None

    byte_order = (
        '>'
        if msg.is_bigendian
        else '<'
    )

    if msg.encoding == '16UC1':

        bytes_per_pixel = 2

        offset = (
            v * msg.step
            + u * bytes_per_pixel
        )

        depth_mm = struct.unpack_from(
            byte_order + 'H',
            msg.data,
            offset
        )[0]

        if depth_mm == 0:
            return None

        return depth_mm / 1000.0


    elif msg.encoding == '32FC1':

        bytes_per_pixel = 4

        offset = (
            v * msg.step
            + u * bytes_per_pixel
        )

        depth_m = struct.unpack_from(
            byte_order + 'f',
            msg.data,
            offset
        )[0]

        if (
            not math.isfinite(depth_m)
            or depth_m <= 0.0
        ):
            return None

        return depth_m


    return None

def get_valid_depth_near_pixel(
        msg,
        u,
        v,
        radius=2
):

    valid_depths = []

    for offset_v in range(
        -radius,
        radius + 1
    ):

        for offset_u in range(
            -radius,
            radius + 1
        ):

            depth_m = get_depth_at_pixel(
                msg,
                u + offset_u,
                v + offset_v
            )

            if depth_m is not None:

                valid_depths.append(
                    depth_m
                )


    if not valid_depths:
        return None


    return statistics.median(
        valid_depths
    )