#!/usr/bin/env python3
import os.path

from ultralytics import YOLO
import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference


bridge = CvBridge()


class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')


        # Using cv2.imshow() method
        # Displaying the image
        model_file = os.path.join(get_package_share_directory('yolo'),'models','best.pt')
        self.model = YOLO(model_file)
        self.yolov8_inference = Yolov8Inference()
        print("YOLO model loaded")
        self.subscription = self.create_subscription(Image,'/camera/image_raw', self.camera_callback, 10)
        print("Image subscription created")

        self.YOLO_pub = self.create_publisher(Image,"/detection_results",1)
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        # self.subscription

    def camera_callback(self, data):
        global img
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        results = self.model(img)
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()





        res_img = results[0].plot()
        # res_img = cv2.cvtColor(res_img, cv2.COLOR_BGR2HSV)
        # print(data.height,data.width, data.encoding)
        # cv2.imshow('spot_head_camera', img)
        # cv2.waitKey(20)
        # cv2.destroyAllWindows()
        # cv2.imwrite('sample_out_1.png', img)
        self.YOLO_pub.publish(bridge.cv2_to_imgmsg(res_img,'bgr8'))




# class Yolo_subscriber(Node):
#
#     def __init__(self):
#         super().__init__('yolo_subscriber')
#
#         self.subscription = self.create_subscription(
#             Yolov8Inference,
#             '/Yolov8_Inference',
#             self.yolo_callback,
#             10)
#         self.subscription
#
#         self.cnt = 0
#
#         self.img_pub = self.create_publisher(Image, "/inference_result_cv2", 1)
#
#     def yolo_callback(self, data):
#         global img
#         for r in data.yolov8_inference:
#             class_name = r.class_name
#             top = r.top
#             left = r.left
#             bottom = r.bottom
#             right = r.right
#             yolo_subscriber.get_logger().info(f"{self.cnt} {class_name} : {top}, {left}, {bottom}, {right}")
#             cv2.rectangle(img, (top, left), (bottom, right), (255, 255, 0))
#             self.cnt += 1
#
#         self.cnt = 0
#         img_msg = bridge.cv2_to_imgmsg(img)
#         self.img_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()