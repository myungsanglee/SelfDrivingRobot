#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

# ROS
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
# TensorFlow Lite
import tflite_runtime.interpreter as tflite
import numpy as np
import re
# OpenCV 3.4.0
import cv2
# D435i
import pyrealsense2 as rs
# i2c for BH1750
import smbus
# i2c for PCA9685
import board
import busio
import adafruit_pca9685 
# others
import time
import sys

# Define Variable
DEBUG = True
VIDEO_WRITE = False
LIGHT_I2C = True
CORRECT_CLASS = True

class ROS:

    def __init__(self):

        # Pub Raw Image
        self.color_image_pub = rospy.Publisher('nano/camera/color_image', Image, queue_size=1)
        self.depth_image_pub = rospy.Publisher('nano/camera/depth_image', Image, queue_size=1)
        self.bridge = CvBridge()

        # Pub Compressed Image
        self.compressed_color_image_pub = rospy.Publisher('nano/camera/comp/color_image', CompressedImage, queue_size=1)
        self.compressed_depth_image_pub = rospy.Publisher('nano/camera/comp/depth_image', CompressedImage, queue_size=1)
        # We do not use cv_bridge it does not support CompressedImage in python

        # Pub Nano Status
        self.nano_status_pub = rospy.Publisher('nano/event', String, queue_size=1)

        # Sub Pi Status
        rospy.Subscriber('pi/event', String, self.pi_status_callback)

    # ---------------------------------------------------------------------------------------------------

    def pub_raw_color_image(self, image):

        # Create Image msg
        try:
            raw_image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        rospy.loginfo("Publish the Raw Color Image")
        self.color_image_pub.publish(raw_image_msg)

    # ---------------------------------------------------------------------------------------------------

    def pub_raw_depth_image(self, image):

        # Create Image msg
        try:
            raw_image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        rospy.loginfo("Publish the Raw Depth Image")
        self.depth_image_pub.publish(raw_image_msg)

    # ---------------------------------------------------------------------------------------------------

    def pub_compressed_color_image(self, image):

        # Create CompressedImage
        compressed_image_msg = CompressedImage()
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        # Publish Compressed Image
        rospy.loginfo("Publish the Compressed Color Image")
        self.compressed_color_image_pub.publish(compressed_image_msg)

    # ---------------------------------------------------------------------------------------------------

    def pub_compressed_depth_image(self, image):

        # Create CompressedImage
        compressed_image_msg = CompressedImage()
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        # Publish Compressed Image
        rospy.loginfo("Publish the Compressed Depth Image")
        self.compressed_depth_image_pub.publish(compressed_image_msg)

    # ---------------------------------------------------------------------------------------------------

    def pub_nano_status(self, status):

        # Create Status msg
        status_msg = status
        rospy.loginfo("Publish the Nano Status: {}".format(status_msg))
        self.nano_status_pub.publish(status_msg)

    # ---------------------------------------------------------------------------------------------------

    def pi_status_callback(self, status):

        # Sub Pi Status msg
        global pi_msg
        pi_msg = status
        rospy.loginfo("Subscribe the Pi Status: {}".format(pi_msg))

########################################################################################################################
########################################################################################################################

class Realsense:

    def __init__(self):

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        #self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        #self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)

    # ---------------------------------------------------------------------------------------------------

    def get_frames(self):

        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        return color_frame, depth_frame

    # ---------------------------------------------------------------------------------------------------
        
    def stop(self):

        self.pipeline.stop()

########################################################################################################################
########################################################################################################################

class TFLite:

    def __init__(self):

        # Load COCO label map
        self.labelmap_dir = '/home/michael/TensorFlow/TFLite/models/ssd_project/8th/8th_150000/labelmap.txt'
        self.labels = self.load_labels(self.labelmap_dir)

        # Load TFLite model and allocate tensors.
        self.ssd_v2_edgetpu_detect_tflite_dir = '/home/michael/TensorFlow/TFLite/models/ssd_project/8th/8th_150000/detect_edgetpu.tflite'
        self.interpreter = tflite.Interpreter(model_path=self.ssd_v2_edgetpu_detect_tflite_dir, experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])
        self.interpreter.allocate_tensors()

    # ---------------------------------------------------------------------------------------------------

    def load_labels(self, path):
        """Loads the labels file. Supports files with or without index numbers."""
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            labels = {}
            for row_number, content in enumerate(lines):
                pair = re.split(r'[:\s]+', content.strip(), maxsplit=1)
                if len(pair) == 2 and pair[0].strip().isdigit():
                    labels[int(pair[0])] = pair[1].strip()
                else:
                    labels[row_number] = pair[0].strip()
        return labels

    # ---------------------------------------------------------------------------------------------------

    def get_output_tensor(self, interpreter, index):
        """Returns the output tensor at the given index."""
        output_details = interpreter.get_output_details()[index]
        tensor = np.squeeze(interpreter.get_tensor(output_details['index']))
        return tensor

    # ---------------------------------------------------------------------------------------------------

    def detect_objects(self, interpreter, image, threshold):
        """Returns a list of detection results, each a dictionary of object info."""
        input_details = interpreter.get_input_details()
        interpreter.set_tensor(input_details[0]['index'], image)
        interpreter.invoke()

        # Get all output details
        boxes = self.get_output_tensor(interpreter, 0)
        classes = self.get_output_tensor(interpreter, 1)
        scores = self.get_output_tensor(interpreter, 2)
        count = int(self.get_output_tensor(interpreter, 3))

        results = []
        for i in range(count):
            if scores[i] >= threshold:
                result = {
                    'bounding_box': boxes[i],
                    'class_id': classes[i],
                    'score': scores[i]
                }
                results.append(result)
        return results

    # ---------------------------------------------------------------------------------------------------

    def show_inference_image(self, image, depth_frame, results, labels):
        result_size = len(results)
        status_results = []
        box_results = []
        for idx, obj in enumerate(results):
            # print(obj)
            # Prepare image width, height
            im_height, im_width, _ = image.shape
        
            # Prepare score, name
            score = obj['score']
            name = labels[obj['class_id']]
            class_num = obj['class_id']
            #print('class_id_num: {}'.format(obj['class_id']))
        
            # Prepare boundary box
            ymin, xmin, ymax, xmax = obj['bounding_box']
            xmin = int(xmin * im_width)
            xmax = int(xmax * im_width)
            ymin = int(ymin * im_height)
            ymax = int(ymax * im_height)

            # Get Distance
            xcenter, ycenter = self.get_depth_center(xmin, xmax, ymin, ymax)
            zDepth = depth_frame.get_distance(xcenter, ycenter)

            # Distinguish no_right_turn:roundabout & school_zone:tunnel
            # Distinguish no_right_turn:roundabout
            if CORRECT_CLASS:
                if class_num == 3 or class_num == 5:
                    class_num = self.get_correct_class(image, class_num, xmin, xmax, ymin, ymax)
                    name = labels[class_num]

            # Distinguish school_zone:tunnel
                elif class_num == 1 or class_num == 4:
                    class_num = self.get_correct_class(image, class_num, xmin, xmax, ymin, ymax)
                    name = labels[class_num]

            # Append Class and Distance
            box_results.append([xmin, xmax, ymin, ymax])
            status_result = {
                'class': class_num,
                'distance': int(zDepth*100),
                'box': box_results
            }
            status_results.append(status_result)

            # Draw rectangle to desired thickness
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

            # Annotate image with label and confidence score
            cv2.putText(image, '{}: {:.1f}%({:02d}cm)'.format(name, (score*100), int(zDepth*100)), (xmin, ymin - 5), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

            #print('{}: {:02d}cm'.format(name, int(zDepth*100)))
    
        return image, status_results

    # ---------------------------------------------------------------------------------------------------

    def show_inference_depth_image(self, image, depth_frame, results, labels):
        result_size = len(results)
        for idx, obj in enumerate(results):
            # print(obj)
            # Prepare image width, height
            im_height, im_width, _ = image.shape
        
            # Prepare score, name
            score = obj['score']
            name = labels[obj['class_id']]
            # print('idx: {}, name: {}'.format(idx, name))
        
            # Prepare boundary box
            ymin, xmin, ymax, xmax = obj['bounding_box']
            #xmin = 90
            xmin = int((xmin * im_width) * 1)
            xmax = int((xmax * im_width) * 1) 
            ymin = int((ymin * im_height) * 1)
            ymax = int((ymax * im_height) * 1)

            xcenter = int((xmin + xmax) * 0.5)
            ycenter = int((ymin + ymax) * 0.5)

            if xcenter >= 450:
                xmin = int(xmin * 0.82)
                xmax = int(xmax * 0.8)

            elif xcenter >= 350:
                xmin = int(xmin * 0.87)
                xmax = int(xmax * 0.87)

            elif xcenter >= 250:
                xmin = int(xmin * 1)
                xmax = int(xmax * 0.9)

            elif xcenter >= 150:
                xmin = int(xmin * 1.1)
                xmax = int(xmax * 1.1)

            elif xcenter < 150:
                xmin = 100
                xmax = int(xmax * 1.1)

            if ycenter <= 150:
                ymin = 80
                ymax = int(ymax * 1.2)

            elif ycenter <= 250:
                ymin = int(ymin * 1.2)
                ymax = int(ymax * 1)

            elif ycenter > 250:
                ymin = int(ymin * 0.9)
                ymax = int(ymax * 0.9)

            
            xcenter = int((xmin + xmax) * 0.5)
            ycenter = int((ymin + ymax) * 0.5)

            zDepth = depth_frame.get_distance(xcenter, ycenter)

            #print('xcenter = {0:05d}, ycenter = {1:05d}, zDepth = {2:05d}cm'.format(xcenter, ycenter, int(zDepth*100)), end='\r')

            # Draw rectangle to desired thickness
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

            # Draw circle
            cv2.circle(image, (xcenter, ycenter), 10, (0, 255, 0), -1)
        
            # Annotate image with label and confidence score
            cv2.putText(image, '{}: {:.2f}({:02d}cm)'.format(name, score, int(zDepth*100)), (xmin, ymin + 12), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

        return image

    # ---------------------------------------------------------------------------------------------------

    def get_depth_center(self, xmin, xmax, ymin, ymax):

        xcenter = int((xmin + xmax) * 0.5)
        ycenter = int((ymin + ymax) * 0.5)

        if xcenter >= 450:
            xmin = int(xmin * 0.82)
            xmax = int(xmax * 0.8)

        elif xcenter >= 350:
            xmin = int(xmin * 0.87)
            xmax = int(xmax * 0.87)

        elif xcenter >= 250:
            xmin = int(xmin * 1)
            xmax = int(xmax * 0.9)

        elif xcenter >= 150:
            xmin = int(xmin * 1.1)
            xmax = int(xmax * 1.1)

        elif xcenter < 150:
            xmin = 100
            xmax = int(xmax * 1.1)

        if ycenter <= 150:
            ymin = 80
            ymax = int(ymax * 1.2)

        elif ycenter <= 250:
            ymin = int(ymin * 1.2)
            ymax = int(ymax * 1)

        elif ycenter > 250:
            ymin = int(ymin * 0.9)
            ymax = int(ymax * 0.9)

        xcenter = int((xmin + xmax) * 0.5)
        ycenter = int((ymin + ymax) * 0.5)

        return xcenter, ycenter

    # ---------------------------------------------------------------------------------------------------

    def get_correct_class(self, image, class_num, xmin, xmax, ymin, ymax):

        height, width, _ = image.shape
        # 가끔 마이너스 값이 들어온다. 그럼 Error 발생, Error 방지 코드
        if 0 <= xmin <= width and 0 <= xmax <= width and 0 <= ymin <= height and 0 <= ymax <= height:

            # ROI Image
            roi_image = image[ymin:ymax, xmin:xmax]

            # BGR -> HSV
            hsv_image = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)

            # HSV에서 BGR로 가정할 범위를 정의
            lower_red = np.array([175, 200, 100], dtype=np.uint8)
            upper_red = np.array([180, 255, 255], dtype=np.uint8)

            # HSV Mask
            mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
            cv2.imshow('correct_class_mask', mask_red)
            #print(mask_red.shape)

            # Get Correct Class
            if np.any(mask_red > 0):
                if class_num == 3 or class_num == 5:
                    return 5
                elif class_num == 1 or class_num == 4:
                    return 4
            else:
                if class_num == 3 or class_num == 5:
                    return 3
                elif class_num == 1 or class_num == 4:
                    return 1

        else:
             return class_num

    # -------------------------------------------------------------------------------------------

    def inference(self, color_image, depth_image, depth_frame):
        
        # Expand Dimensions for Inference and resize 300*300
        tflite_image = cv2.resize(color_image, dsize=(300, 300), interpolation=cv2.INTER_AREA)
        tflite_image = np.expand_dims(tflite_image, 0)

        # Detection
        threshold = 0.3
        results = self.detect_objects(self.interpreter, tflite_image, threshold)

        # Return result Color image
        result_color_image, status_results = self.show_inference_image(color_image, depth_frame, results, self.labels)
        #result_color_image = cv2.resize(result_color_image, dsize=(640, 480), interpolation=cv2.INTER_AREA)

        # Return result Depth image
        #result_depth_image = self.show_inference_depth_image(depth_image, depth_frame, results, self.labels)
        result_depth_image = depth_image
        #result_depth_image = cv2.resize(result_depth_image, dsize=(640, 480), interpolation=cv2.INTER_AREA)

        return result_color_image, result_depth_image, status_results

########################################################################################################################
########################################################################################################################

class Status:

    def __init__(self):

        # Nano Status Variable
        self.person = 'person'
        #self.avoid_right = 'avoid_right'
        self.no_right_turn = 'no_right_turn'
        self.speed_up = 'speed_up'
        self.school_zone = 'school_zone'
        self.school_zone_off = 'school_zone_off'
        self.roundabout = 'roundabout'
        self.turtlebot = 'turtlebot'
        self.tunnel = 'tunnel'
        self.first_parking = 'first_parking'
        self.second_parking = 'second_parking'
        #self.left_turn = 'left_turn'
        #self.avoid_left = 'avoid_left'
        self.clear = 'clear'
        self.step = 1
        self.person_flag = 0
        self.red_flag = 0
        self.clear_flag = 0
        self.current_status = ''
        self.traffic_light_status = ''

    # ---------------------------------------------------------------------------------------------------

    def get_traffic_light(self, inference_color_image, box_results):

        # ROI Image
        for box in box_results:
            x1 = box[0]
            x2 = box[1]
            y1 = box[2]
            y2 = box[3]

        # Get Image Height, Width
        height, width, _ = inference_color_image.shape
        # 가끔 마이너스 값이 들어온다. 그럼 Error 발생, Error 방지 코드
        if 0 <= x1 <= width and 0 <= x2 <= width and 0 <= y1 <= height and 0 <= y2 <= height:

            roi_image = inference_color_image[y1:y2, x1:x2]

            # Get ROI Image Height
            height, width, _ = roi_image.shape

            # BGR -> HSV
            hsv_image = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)

            # HSV에서 BGR로 가정할 범위를 정의
            lower_red = np.array([170, 200, 100], dtype=np.uint8)
            upper_red = np.array([180, 255, 255], dtype=np.uint8)

            # HSV Mask
            mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
            cv2.imshow('traffic_light_mask', mask_red)
            #print(mask_red.shape)

            # Traffic Light Detection
            if np.any(mask_red > 0):
                print('Red Light')
                self.traffic_light_status = 'red'
            else:
                print('Green Ligth')
                self.traffic_light_status = 'green'

    # -------------------------------------------------------------------------------------------

    def get_nano_status(self, inference_color_image, status_results):

        # Flag Variable
        flag_1 = False
        flag_2 = False
        global tunnel_flag
        global pi_msg

        # Check Step and Check Object Detection
        for obj in status_results:
            class_num = obj['class']
            zDepth = obj['distance']

            # Always Check Detection Person
            if class_num == 0.0 and (20 < zDepth < 30):
                self.current_status = self.person
                flag_1 = True

            # Divide Check Detection by Step
            # Step 1: Detection Traffic Light
            if self.step == 1 and class_num == 9.0 and (40 < zDepth < 65):
                flag_2 = True
                box_results = obj['box']
                self.get_traffic_light(inference_color_image, box_results)
                if self.traffic_light_status == 'red':
                    self.current_status = self.traffic_light_status
                elif self.traffic_light_status == 'green':
                    self.current_status = self.traffic_light_status

            # Step 2: Detection No_Right_Turn
            elif self.step == 2 and class_num == 5.0 and (35 < zDepth < 45):
                flag_2 = True
                self.current_status = self.no_right_turn

            # Step 3: Detection Speed_Up
            elif self.step == 3 and class_num == 6.0 and (50 < zDepth < 65):
                flag_2 = True
                self.current_status = self.speed_up

            # Step 4: Detection School_Zone
            elif self.step == 4 and class_num == 1.0 and (40 < zDepth < 60):
                flag_2 = True
                self.current_status = self.school_zone

            # Step 5: Detection School_Zone_Off
            elif self.step == 5 and class_num == 2.0 and (45 < zDepth < 60):
                flag_2 = True
                self.current_status = self.school_zone_off

            # Step 6: Detection Roundabout
            elif self.step == 6 and class_num == 3.0 and (25 < zDepth < 35):
                flag_2 = True
                self.current_status = self.roundabout

            # Step 7: Detection Turtlebot
            elif self.step == 7 and class_num == 8.0 and (50 < zDepth < 55) and pi_msg != False: 
                flag_2 = True
                pi_msg = False
                self.current_status = self.turtlebot

            # Step 8: Detection Tunnel
            elif self.step == 8 and class_num == 4.0 and (40 < zDepth < 50):
                flag_2 = True
                tunnel_flag = True
                self.current_status = self.tunnel

            # Step 9: Detection First_Parking
            elif self.step == 9 and class_num == 7.0 and (30 < zDepth < 80):
                flag_2 = True
                self.current_status = self.first_parking

            # Step 10: Detection Second_Parking
            elif self.step == 10 and class_num == 7.0 and (40 < zDepth < 60) and pi_msg != False:
                flag_2 = True
                pi_msg = False
                self.current_status = self.second_parking

            # Step 11: Detection Traffic_Light
            elif self.step == 11 and class_num == 9.0 and (30 < zDepth < 60) and pi_msg != False:
                flag_2 = True
                pi_msg = False
                box_results = obj['box']
                self.get_traffic_light(inference_color_image, box_results)
                if self.traffic_light_status == 'red':
                    self.current_status = self.traffic_light_status
                elif self.traffic_light_status == 'green':
                    self.current_status = 'red'

                else:
                    self.current_status = 'red'

            # Step 12: Detection Turtlebot
            #elif self.step == 12 and class_num == 8.0 and (40 < zDepth < 50):
            #    flag_2 = True
            #    self.current_status = self.avoid_left

        # 1st check: Person
        if flag_1 == True:
            # For Publish only Once
            if self.person_flag == 0:
                self.person_flag = 1
                self.red_flag = 0
                self.clear_flag = 0
                return self.current_status
            else:
                return False

        # 2nd check: Other Object
        elif flag_1 == False and flag_2 == True:
            # Divide Check Detection by Traffic Light 'RED'
            if self.current_status == 'red':
                # For Publish only Once
                if self.red_flag == 0:
                    self.person_flag = 0
                    self.red_flag = 1
                    self.clear_flag = 0
                    return self.current_status
                else:
                    return False

            # if not current_status != 'red', publish msg once and go next step
            else:
                # Next Step
                self.step += 1
                # Flag Reset
                self.person_flag = 0
                self.red_flag = 0
                self.clear_flag = 0
                return self.current_status

        # 3rd check: Clear or Detection Nothing
        else:
            self.current_status = self.clear
            # For Publish only Once
            if self.clear_flag == 0:
                self.person_flag = 0
                self.red_flag = 0
                self.clear_flag = 1
                return self.current_status
            else:
                return False

########################################################################################################################
########################################################################################################################

class Light:

    def __init__(self):

        # Variable for BH1750
        self.I2C_CH = 1
        # BH1750 주소
        # i2cdetect -y -r 1(I2C_CH)
        # 위의 코드로 확인
        self.BH1750_DEV_ADDR = 0x23
        #측정 모드들 https://blog.naver.com/chandong83/221602405432 참고
        self.CONT_H_RES_MODE = 0x10

        # Variable for PCA9685
        # On the Jetson Nano
        # Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
        # Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
        # Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ... 
        self.i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
        self.pca = adafruit_pca9685.PCA9685(self.i2c_bus0)
        self.pca.frequency = 100
        self.led_channel = self.pca.channels[0]

    # ---------------------------------------------------------------------------------------------------

    def get_illuminance(self):

        i2c = smbus.SMBus(self.I2C_CH)
        luxBytes = i2c.read_i2c_block_data(self.BH1750_DEV_ADDR, self.CONT_H_RES_MODE, 2)
        lux = int.from_bytes(luxBytes, byteorder='big')
        i2c.close()
        return lux

    # ---------------------------------------------------------------------------------------------------

    def headlight(self, lux):

        global tunnel_flag

        # Headlight Turn ON or OFF
        if lux < 80:
            tunnel_flag = False
            self.led_channel.duty_cycle = 0x2fff
        elif tunnel_flag == True:
            self.led_channel.duty_cycle = 0x2fff
        else:
            self.led_channel.duty_cycle = 0

########################################################################################################################
########################################################################################################################

def main():

    try:
        ros = ROS()
        tf = TFLite()
        camera = Realsense()
        status = Status()

        if LIGHT_I2C:
            light = Light()

        # Global Variable Initialize for Tunnel Flag
        global tunnel_flag
        tunnel_flag = False

        # Global Variable Initialize for Pi Message
        global pi_msg
        pi_msg = False

        # Variable Initialize for Captured Image
        test = 0

        rospy.init_node('image_pub', anonymous=True)

        if VIDEO_WRITE:
            #cap_color = cv2.VideoCapture('/home/michael/Bit_Project/Video/2nd_dataset_video/color_video_test_02.avi')
            #cap_depth = cv2.VideoCapture('/home/michael/Bit_Project/Video/2nd_dataset_video/color_video_test_02.avi')

            frame_size = (640, 480)
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            fps = 20
            color_filename = '/home/michael/Bit_Project/Video/color_video.avi'
            depth_filename = '/home/michael/Bit_Project/Video/depth_video.avi'

            out_color = cv2.VideoWriter(color_filename, fourcc, fps, frame_size)
            #out_depth = cv2.VideoWriter(depth_filename, fourcc, fps, frame_size)



        while not rospy.is_shutdown():

            start_time = time.time()

            # Get the Image from Realsense Camera
            color_frame, depth_frame = camera.get_frames()
            if not depth_frame or not color_frame:
            #if not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)

            # Show Color Image
            #cv2.imshow('Color Image', color_image)

            # Show depth Image
            #cv2.imshow('Depth Image', depth_colormap)

            #if VIDEO_WRITE:
                # Get Video Frames
                #retval_color, color_image = cap_color.read()
                #retval_depth, depth_colormap = cap_depth.read()
                #if not retval_color or not retval_depth:
                #if not retval_color:
                #    break

            #color_image = cv2.imread('/home/michael/test_image/image_03.jpg')

            # Get Inference Image
            start_time_inference = time.time()
            inference_color_image, inference_depth_image, status_results = tf.inference(color_image, depth_colormap, depth_frame)
            end_time_inference = time.time()
            inference_fps = int(1/(end_time_inference - start_time_inference))

            # Get Nano Status
            nano_status = status.get_nano_status(inference_color_image, status_results)

            # Show Inference Image
            cv2.imshow('Inference Color Image', inference_color_image)
            #cv2.imshow('Inference Depth Image', inference_depth_image)

            if VIDEO_WRITE:
                # Video Write
                out_color.write(inference_color_image)
                #out_depth.write(inference_depth_image)
                #out_color.write(color_image)
                #out_depth.write(depth_colormap)

            # Publish the Image
            start_time_publish = time.time()
            #ros.pub_raw_color_image(color_image)
            #ros.pub_raw_depth_image(depth_colormap)
            ros.pub_raw_color_image(inference_color_image)
            #ros.pub_raw_depth_image(inference_depth_image)
            #ros.pub_compressed_color_image(inference_color_image)
            #ros.pub_compressed_depth_image(inference_depth_image)
            end_time_publish = time.time()
            publish_fps = int(1/(end_time_publish - start_time_publish))

            # Publish the Nano Status
            if nano_status != False:
                ros.pub_nano_status(nano_status)

            # Get Illuminance
            if LIGHT_I2C:
                lux = light.get_illuminance()
                light.headlight(lux)
                if DEBUG:
                    print('Lux: {}'.format(lux))

            end_time = time.time()

            if DEBUG:
                print('Tunnel_Flag: {}'.format(tunnel_flag))
                print('Pi_Msg: {}'.format(pi_msg))
                print('Step: {}, Status: {}'.format(status.step, status.current_status))
                print('Total_FPS = {0:03d}, Inference_FPS = {1:03d}, Publish_FPS = {2:03d}'.format(int(1/(end_time - start_time)), inference_fps, publish_fps))

            if LIGHT_I2C:
                key = cv2.waitKey(120) # 120ms for Illuminance sensor
            else:
                key = cv2.waitKey(1)

            if key == ord('a'):
                nano_status = 'person'
                ros.pub_nano_status(nano_status)
            elif key == ord('b'):
                nano_status = 'red'
                ros.pub_nano_status(nano_status)
            elif key == ord('c'):
                nano_status = 'green'
                ros.pub_nano_status(nano_status)
            elif key == ord('d'):
                nano_status = 'no_right_turn'
                ros.pub_nano_status(nano_status)
            elif key == ord('e'):
                nano_status = 'speed_up'
                ros.pub_nano_status(nano_status)
            elif key == ord('f'):
                nano_status = 'school_zone'
                ros.pub_nano_status(nano_status)
            elif key == ord('g'):
                nano_status = 'school_zone_off'
                ros.pub_nano_status(nano_status)
            elif key == ord('h'):
                nano_status = 'roundabout'
                ros.pub_nano_status(nano_status)
            elif key == ord('i'):
                nano_status = 'tunnel'
                ros.pub_nano_status(nano_status)
            elif key == ord('j'):
                nano_status = 'first_parking'
                ros.pub_nano_status(nano_status)
            elif key == ord('k'):
                nano_status = 'second_parking'
                ros.pub_nano_status(nano_status)
            elif key == ord('l'):
                nano_status = 'turtlebot'
                ros.pub_nano_status(nano_status)
            elif key == ord('o'):
                status.step += 1
            elif key == ord('p'):
                status.step -= 1
            elif key == ord('q'):
                nano_status = 'clear'
                ros.pub_nano_status(nano_status)
            elif key == ord('z'):
                test += 1
                cv2.imwrite('/home/michael/Bit_Project/Captured_Image/test_{:03d}.jpg'.format(test), inference_color_image)
                if DEBUG:
                    print('Captured Image--------------------------------')

    except KeyboardInterrupt as e:
        print(e)
        pass

    except rospy.ROSInterruptException as e:
        print(e)
        pass

    except Exception as e:
        print('Exception Arise')
        print(e)
        pass

    finally:
        print('Finally Arise')
        camera.stop()
        cv2.destroyAllWindows()

        if VIDEO_WRITE:
            out_color.release()
            #out_depth.release()
            #cap_color.release()
            #cap_depth.release()

########################################################################################################################
########################################################################################################################

if __name__ == '__main__':
    main()
