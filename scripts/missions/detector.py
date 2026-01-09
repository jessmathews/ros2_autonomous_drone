#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from qrdet import QRDetector


class QRDetectorNode(Node):

    def __init__(self):
        super().__init__('qr_detector_node')

        # QoS: BEST_EFFORT is REQUIRED for camera topics
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=5
        )

        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos
        )

        self.bridge = CvBridge()
        self.detector = QRDetector(model_size='n')

        self.get_logger().info("QR Detector Node started")

    def image_callback(self, msg: Image):
        # Convert ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        proc_img = cv2.resize(frame, (320, 320))
        h, w = proc_img.shape[:2]



        detections = self.detector.detect(proc_img)
        # if detections:
        #     print(detections[0].keys())


        for det in detections:
            pts = [(int(x), int(y)) for x, y in det['polygon_xy']]
            print("Detected",end='\r')

            # Draw polygon bounding box
            for i in range(len(pts)):
                cv2.line(
                    proc_img,
                    pts[i],
                    pts[(i + 1) % len(pts)],
                    (0, 255, 0),
                    2
                )

            # Draw center
            cx = sum(p[0] for p in pts) // 4
            cy = sum(p[1] for p in pts) // 4
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        
        # Crosshair settings
        color = (0, 0, 255)   # Green (BGR)
        thickness = 2
        line_length = min(w, h) // 2  # Length of each line from center
        x, y = w // 2, h // 2


        cv2.line(proc_img,
         (x - line_length, y),
         (x + line_length, y),
         color,
         thickness)
        
        cv2.line(proc_img,
         (x, y - line_length),
         (x, y + line_length),
         color,
         thickness)

        cv2.imshow("QR Detection", proc_img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = QRDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
