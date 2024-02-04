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
from yolov8_msgs.msg import Poseresults
from yolov8_msgs.msg import Poseinference


bridge = CvBridge()


class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')


        # Using cv2.imshow() method
        # Displaying the image
        model_file_detection = os.path.join(get_package_share_directory('yolo'),'models','best_detect.onnx')
        self.model_detection = YOLO(model_file_detection,task='detect')
        model_file_pose = os.path.join(get_package_share_directory('yolo'), 'models', 'best_pose.onnx')
        self.model_pose = YOLO(model_file_pose,task='pose')
        self.yolov8_inference = Yolov8Inference()
        self.poseinference = Poseinference()
        print("YOLO model loaded")
        self.subscription = self.create_subscription(Image,'/camera/image_raw', self.camera_callback, 1)
        print("Image subscription created")

        self.YOLO_pub = self.create_publisher(Image,"/detection_results",1)
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.pose_pub = self.create_publisher(Poseinference, "/pose_inference", 1)
        self.model_names=['chip','socket']
        # self.subscription

    def camera_callback(self, data):
        global img
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        results = self.model_detection(img)
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()
        self.poseinference.header.frame_id = 'pose_inference'
        self.poseinference.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model_names[int(c)]
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
        # self.YOLO_pub.publish(bridge.cv2_to_imgmsg(res_img,'bgr8'))
        results_pose = self.model_pose(img)
        for c in range(len(results_pose[0].boxes.cls)):
            self.pose_result = Poseresults()
            self.pose_result.class_name = results_pose[0].names[int(results_pose[0].boxes.cls[c].numpy())]
            self.pose_result.box = results_pose[0].boxes.xyxy[c,:].to('cpu').detach().numpy().copy().astype(int)
            self.pose_result.keypointx = results_pose[0].keypoints.xy[c, :, 0].to('cpu').detach().numpy().copy().astype(int)
            self.pose_result.keypointy = results_pose[0].keypoints.xy[c, :, 1].to('cpu').detach().numpy().copy().astype(int)
            self.poseinference.pose_inference.append(self.pose_result)

        self.pose_pub.publish(self.poseinference)
        self.poseinference.pose_inference.clear()
        # res_pose = results_pose[0].plot()
        # self.YOLO_pub.publish(bridge.cv2_to_imgmsg(res_pose, 'bgr8'))





def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()