import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from camera_perception.apriltag_detector import create_apriltag_detector
from camera_perception.apriltag_detector import detect_apriltags

class RGBMonitorNode(Node):

    def __init__(self):
        super().__init__('rgb_monitor_node')

        self.declare_parameter(
            'rgb_topic',
            '/camera/color/image_raw'
        )

        self.rgb_topic = (
            self.get_parameter('rgb_topic')
            .get_parameter_value()
            .string_value
        )

        self.bridge = CvBridge()
        self.apriltag_detector = (
            create_apriltag_detector()
        )

        self.frame_count = 0
        self.first_frame_received = False

        self.subscription = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10
        )

        self.get_logger().info(
            f'RGB Monitor Node started | '
            f'topic={self.rgb_topic}'
        )


    def rgb_callback(self, msg):

        self.frame_count += 1

        if not self.first_frame_received:

            self.first_frame_received = True

            self.get_logger().info(
                'First RGB frame received\n'
                f'  size     = {msg.width} x {msg.height}\n'
                f'  encoding = {msg.encoding}\n'
                f'  step     = {msg.step} bytes\n'
                f'  frame_id = {msg.header.frame_id}'                
            )

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            detections = detect_apriltags(
                frame,
                self.apriltag_detector
            )

            for detection in detections:

                marker_id = detection['id']

                corners = detection[
                    'corners'
                ].astype(int)

                center_u, center_v = (
                    detection['center']
                )

                cv2.polylines(
                    frame,
                    [corners],
                    True,
                    (0, 255, 0),
                    2
                )

                cv2.circle(
                    frame,
                    (center_u, center_v),
                    5,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    frame,
                    f'ID {marker_id}',
                    (
                        center_u + 10,
                        center_v - 10
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

        except Exception as error:

            self.get_logger().error(
                f'Failed to convert RGB image: {error}'
            )

            return

        cv2.imshow(
            'Astra RGB',
            frame
        )

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = RGBMonitorNode()

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