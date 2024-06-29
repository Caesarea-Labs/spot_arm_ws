#!/usr/bin/env python3
import os.path


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud

from std_msgs.msg import String
from cv_bridge import CvBridge
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference
from yolov8_msgs.msg import Poseresults
from yolov8_msgs.msg import Poseinference
from yolov8_msgs.msg import Ws
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()
import cv2 as cv




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
    def comp_ang_z(self):
        self.angs[2]=np.arctan2(self.key_y[3]-self.key_y[0],self.key_x[3]-self.key_x[0])

class socket:
    def __init__(self):
        self.detected = False
        self.x = float("nan")
        self.y = float("nan")
        self.state = "unknown"
        self.angs = 3*[float("nan")]
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

    def comp_ang_z(self):
        self.angs[2] = np.arctan2(self.key_y[3]-self.key_y[0],self.key_x[3]-self.key_x[0])

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        #Create Subscription to detection
        self.Detec_sub = self.create_subscription(Yolov8Inference, "/Yolov8_Inference", self.detec_callback, 1)
        #Create Subscription to yolo pose inference
        self.Pose_sub = self.create_subscription(Poseinference, "/pose_inference", self.pose_callback, 1)
        #Create Sunscription to pointcloud sensor
        self.Pointcloud_sub = self.create_subscription(PointCloud, "/spot/pointcloud", self.pointcloud_callback, 1)
        # Add publisher for workspace estimation
        # self.WS_est_pub = self.create_publisher(TBD, "/ws_state", 1)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 1)
        self.est_draw = self.create_publisher(Image, "/Est_results", 1)
        self.est_res = self.create_publisher(Ws, "/Ws_res", 1)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ws_state = Ws()
        print('All good, node started')

        self.model_names=['chip','socket']
        self.chip = chip()
        self.socket = socket()
        self.TOF_dist=100
        global img
        # self.subscription


    def detec_callback(self, data):
        self.chip.undetected()
        self.socket.undetected()
        # This function will compute the angles of the detection w.r.t. camera
        # print( not (data.yolov8_inference))
        # detection Origin at to left y-horizontal x-vertical y[0, 640] x[0, 480]
        # Output: Origin at bottom left x-horizontal y-vertical x[0, 640] y[0, 480]
        for r in data.yolov8_inference:
            if r.class_name == 'chip':
                self.chip.x = (r.top + r.bottom) / 2
                self.chip.y = 480 - (r.left + r.right) / 2
                self.chip.ang_x = (self.chip.y - 240) * 1.089 / 640
                self.chip.ang_y = (self.chip.x - 320) * 1.089 / 640
                self.chip.detected = True

                # print('chip: ' + ',  x=' + str(self.chip.x) + ',  y=' + str(self.chip.y) + ',  ang_x=' + str(self.chip.ang_x)+ ',  ang_y=' + str(self.chip.ang_y))
            elif r.class_name == 'socket':
                self.socket.x = (r.top + r.bottom) / 2
                self.socket.y = 480 - (r.left + r.right) / 2
                self.socket.ang_x = (self.socket.y - 240) * 1.089 / 640
                self.socket.ang_y = (self.socket.x - 320) * 1.089 / 640
                self.socket.detected = True
                # print('socket: '+ ',  x=' + str(self.socket.x) + ',  y=' + str(self.socket.y) + ',  ang_x=' + str(self.socket.ang_x)+ ',  ang_y=' + str(self.socket.ang_y))
            #This function will compute the angles of the detection w.r.t. camera

    def pose_callback(self, data):
        # Detection: Origin at top left x-horizontal y-vertical x[0, 640] y[0, 480]
        # Output: Origin at bottom left x-horizontal y-vertical x[0, 640] y[0, 480]
        # print(data.pose_inference)
        self.chip.pose_detected = False
        self.socket.pose_detected = False
        for r in data.pose_inference:
            if r.class_name == 'chip':
                self.chip.pose_detected = True
                self.chip.key_x = r.keypointx[0:4]
                self.chip.key_y = 480 - r.keypointy[0:4]
                self.chip.comp_ang_z()
            elif r.class_name == 'socket_closed':
                self.socket.pose_detected = True
                self.socket.state = 'close'
                self.socket.key_x=list()
                self.socket.key_y=list()

                for i in [0, 1, 2, 3]:
                    self.socket.key_x.append(r.keypointx[i])
                    self.socket.key_y.append(480-r.keypointy[i])

                self.socket.comp_ang_z()

            elif r.class_name == 'socket_open':
                self.socket.pose_detected = True
                self.socket.state = 'open'
                self.socket.key_out_x = list()
                self.socket.key_out_y = list()
                self.socket.key_in_x = list()
                self.socket.key_in_y = list()
                for i in [0, 1, 2, 3]:
                    self.socket.key_out_x.append(r.keypointx[i])
                    self.socket.key_out_y.append(480 - r.keypointy[i])
                # for i in [4, 5, 6, 7]:
                #     self.socket.key_in_x.append(r.keypointx[i])
                #     self.socket.key_in_y.append(480 - r.keypointy[i])
                self.socket.comp_ang_z()

        # This function will compute the orientation of the objects w.r.t. camera

    def pointcloud_callback(self,data):
        avg = 0
        if len(data.points):
            for i in data.points:
                avg += i.x
            avg /= len(data.points)
        self.TOF_dist = avg - 0.11778
    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        if (self.chip.detected):
            cv.circle(img, (int(self.chip.x), 480-int(self.chip.y)), 5, (0, 0, 255), -1)
        if (self.socket.detected):
            cv.circle(img, (int(self.socket.x), 480-int(self.socket.y)), 5, ( 255, 0, 0), -1)
        if (self.chip.pose_detected):
            i=0
            cv.circle(img, (int(self.chip.key_x[i]), 480-int(self.chip.key_y[i])), 5, (0, 0, 255), -1)
            cv.putText(img, str(self.chip.ang_z),(int(self.chip.key_x[i]), 480 - int(self.chip.key_y[i])), cv.FONT_HERSHEY_SIMPLEX,4, (255, 255, 255), 2, cv.LINE_AA)
        if (self.socket.pose_detected):
            i=0#for i in range(4):
            cv.circle(img, (int(self.socket.key_in_x[i]), 480 - int(self.socket.key_in_y[i])), 5, (255, 0, 0), -1)
            cv.putText(img, str(self.socket.ang_z), (int(self.socket.key_in_x[i]), 480-int(self.socket.key_in_y[i])),cv.FONT_HERSHEY_SIMPLEX, 4, (255, 255, 255), 2, cv.LINE_AA)


        self.est_draw.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
    def timer_callback(self):
        #Setting the Chip
        if self.chip.detected == True:
            self.ws_state.chip.detected = True
            self.ws_state.chip.c_x = self.chip.x
            self.ws_state.chip.c_y = self.chip.y
            self.ws_state.chip.angs[0] = self.chip.ang_x
            self.ws_state.chip.angs[1] = self.chip.ang_y
        else:
            self.ws_state.chip.detected = False
        if self.chip.pose_detected == True:
            self.ws_state.chip.pose_detected = True
            self.ws_state.chip.box_x = self.chip.key_x
            self.ws_state.chip.box_y = self.chip.key_y
            self.ws_state.chip.angs[2] = self.chip.ang_z
        else:
            self.ws_state.chip.pose_detected = False

        if self.socket.detected == True:
            self.ws_state.socket.detected = True
            self.ws_state.socket.c_x = self.socket.x
            self.ws_state.socket.c_y = self.socket.y
            self.ws_state.socket.angs[0] = self.socket.ang_x
            self.ws_state.socket.angs[1] = self.socket.ang_y
        else:
            self.ws_state.socket.detected = False
        if self.socket.pose_detected == True:
            self.ws_state.socket.pose_detected = True
            if self.socket.state == 'open':
                self.ws_state.socket.state = True
            else:
                self.ws_state.socket.state = False
            for i in range(4):
                self.ws_state.socket.box_x[i] = self.socket.key_in_x[i]
                self.ws_state.socket.box_y[i] = self.socket.key_in_y[i]
                self.ws_state.socket.out_box_x[i] = self.socket.key_out_x[i]
                self.ws_state.socket.out_box_y[i] = self.socket.key_out_y[i]
            self.ws_state.socket.angs[2] = self.socket.ang_z
        else:
            self.ws_state.socket.pose_detected = False

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