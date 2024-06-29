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
from yolov8_msgs.msg import Ws
from sensor_msgs.msg import PointCloud
import numpy as np



bridge = CvBridge()


class chip:
    def __init__(self):
        self.detected = False
        self.x = float("nan")
        self.y = float("nan")
        self.angs = 3*[float("nan")]
        self.key_x = 4*[float("nan")]
        self.key_y = 4*[float("nan")]
    def undetected(self):
        self.detected = False
        self.x = float("nan")
        self.y = float("nan")
        self.angs = 3*[float("nan")]
        self.key_x = 4 * [float("nan")]
        self.key_y = 4 * [float("nan")]
    def comp_angs(self):
        self.angs[0] = (self.x - 960) * 1.089 / 1920
        self.angs[1] = (self.y - 540) * 1.089 / 1920
        self.angs[2] = np.arctan2(self.key_y[3] - self.key_y[0], self.key_x[3] - self.key_x[0])


class socket:
    def __init__(self):
        self.detected = False
        self.x = float("nan")
        self.y = float("nan")
        self.state = "unknown"
        self.angs = 3*[float("nan")]
        self.state = 'unknown'
        self.key_x = 4 * [float("nan")]
        self.key_y = 4 * [float("nan")]


    def undetected(self):
        self.detected = False
        self.x = float("nan")
        self.y = float("nan")
        self.state = "unknown"
        self.angs = 3*[float("nan")]
        self.key_x = 4 * [float("nan")]
        self.key_y = 4 * [float("nan")]

    def comp_angs(self):
        self.angs[0] = (self.x - 960) * 1.089 / 1920
        self.angs[1] = (self.y - 540) * 1.089 / 1920
        self.angs[2] = np.arctan2(self.key_y[3] - self.key_y[0], self.key_x[3] - self.key_x[0])


class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.chip = chip()
        self.socket = socket()
        # Using cv2.imshow() method
        # Displaying the image
        # model_file_detection = os.path.join(get_package_share_directory('yolo'),'models','best_detect.onnx')
        # self.model_detection = YOLO(model_file_detection,task='detect')
        model_file_pose = os.path.join(get_package_share_directory('yolo'), 'models', 'best.onnx')
        self.model_pose = YOLO(model_file_pose,task='pose')
        self.yolov8_inference = Yolov8Inference()
        self.poseinference = Poseinference()
        print("YOLO pose model loaded")
        self.subscription = self.create_subscription(Image,'/camera/image_raw', self.camera_callback, 1)
        print("Image subscription created")
        self.Pointcloud_sub = self.create_subscription(PointCloud, "/spot/pointcloud", self.pointcloud_callback, 1)
        print("Pointcloud  subscription created")
        self.est_res = self.create_publisher(Ws, "/Ws_res", 1)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ws_state = Ws()
        print('All good, node started')
        self.TOF_dist = 100
        global img
        # self.YOLO_pub = self.create_publisher(Image,"/detection_results",1)
        # self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)

        self.model_names=['chip','socket']
        # self.subscription

    def pointcloud_callback(self, data):
        avg = 0
        if len(data.points):
            for i in data.points:
                avg += i.x
            avg /= len(data.points)
        self.TOF_dist = avg - 0.11778

    def camera_callback(self, data):
        global img
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        # Detection
        # results = self.model_detection(img)
        # self.yolov8_inference.header.frame_id = "inference"
        # self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()
        # self.poseinference.header.frame_id = 'pose_inference'
        # self.poseinference.header.stamp = self.get_clock().now().to_msg()
        #
        # for r in results:
        #     boxes = r.boxes
        #     for box in boxes:
        #         self.inference_result = InferenceResult()
        #         b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
        #         c = box.cls
        #         self.inference_result.class_name = self.model_names[int(c)]
        #         self.inference_result.top = int(b[0])
        #         self.inference_result.left = int(b[1])
        #         self.inference_result.bottom = int(b[2])
        #         self.inference_result.right = int(b[3])
        #         self.yolov8_inference.yolov8_inference.append(self.inference_result)
        #
        # self.yolov8_pub.publish(self.yolov8_inference)
        # self.yolov8_inference.yolov8_inference.clear()
        # res_img = results[0].plot()


        # res_img = cv2.cvtColor(res_img, cv2.COLOR_BGR2HSV)
        # print(data.height,data.width, data.encoding)
        # cv2.imshow('spot_head_camera', img)
        # cv2.waitKey(20)
        # cv2.destroyAllWindows()
        # cv2.imwrite('sample_out_1.png', img)
        # self.YOLO_pub.publish(bridge.cv2_to_imgmsg(res_img,'bgr8'))
        results_pose = self.model_pose(img, verbose=False)
        self.chip.undetected()
        self.socket.undetected()

        for c in range(len(results_pose[0].boxes.cls)):
            name = results_pose[0].names[int(results_pose[0].boxes.cls[c].numpy())]
            box = results_pose[0].boxes.xyxy[c,:].to('cpu').detach().numpy().copy().astype(int)
            cx=(box[0]+box[2])/2
            cy=(box[1]+box[3])/2
            key_x = results_pose[0].keypoints.xy[c, :, 0].to('cpu').detach().numpy().copy().astype(int)
            key_y = results_pose[0].keypoints.xy[c, :, 1].to('cpu').detach().numpy().copy().astype(int)
            if name=='chip':
                self.chip.detected = True
                self.chip.x = cx
                self.chip.y = cy
                self.chip.key_x = key_x
                self.chip.key_y = 1080-key_y
                self.chip.comp_angs()
            else:
                self.socket.detected=True
                self.socket.x = cx
                self.socket.y = cy
                self.socket.key_x = key_x
                self.socket.key_y = 1080-key_y
                self.socket.comp_angs()
                if name == 'socket_open':
                    self.socket.state = 'open'
                else:
                    self.socket.state = 'closed'


        # self.pose_pub.publish(self.poseinference)
        # self.poseinference.pose_inference.clear()
        # res_pose = results_pose[0].plot()
        # self.YOLO_pub.publish(bridge.cv2_to_imgmsg(res_pose, 'bgr8'))

    def timer_callback(self):
        # Setting the Chip
        self.ws_state.chip.detected = self.chip.detected
        if self.chip.detected:
            self.ws_state.chip.c_x = self.chip.x
            self.ws_state.chip.c_y = self.chip.y
            for i in range(3):
                self.ws_state.chip.angs[i] = self.chip.angs[i]
            for i in range(4):
                self.ws_state.chip.key_x[i] = self.chip.key_x[i]
                self.ws_state.chip.key_y[i] = self.chip.key_y[i]


        self.ws_state.socket.detected = self.socket.detected
        if self.socket.detected:
            self.ws_state.socket.c_x = self.socket.x
            self.ws_state.socket.c_y = self.socket.y
            for i in range(3):
                self.ws_state.socket.angs[i] = self.socket.angs[i]
            if self.socket.state == 'open':
                self.ws_state.socket.state = True
            else:
                self.ws_state.socket.state = False
            for i in range(4):
                self.ws_state.socket.key_x[i] = self.socket.key_x[i]
                self.ws_state.socket.key_y[i] = self.socket.key_y[i]

        self.ws_state.tof = float(self.TOF_dist)
        self.est_res.publish(self.ws_state)





def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()