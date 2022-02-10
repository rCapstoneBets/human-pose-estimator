
# Copyright 2021 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Main script to run pose classification and pose estimation."""
import argparse
import logging
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Bool, UInt8
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

from human_pose_estimation.ml.classifier import Classifier
from human_pose_estimation.ml.movenet import Movenet
from human_pose_estimation.ml.movenet_multipose import MoveNetMultiPose
from human_pose_estimation.ml.posenet import Posenet
import human_pose_estimation.utils as utils

class HumanPoseNode(Node):

    def __init__(self):
        super().__init__("human_body_pose")
        # create data subscriptions for camera
        self.image_sub = self.create_subscription(Image, "/robot/signaling/raw", self.new_frame, qos_profile_system_default)
        self.debug_sub = self.create_subscription(UInt8, "/robot/debug", self.debug_cb, qos_profile_system_default)

        # create publishers for processed data
        self.image_pub = self.create_publisher(Image, "/robot/signaling/overlay", qos_profile_system_default)
        self.trigger_pub = self.create_publisher(Bool, "/robot/signaling/signal", qos_profile_system_default)
        self.joint_pub = self.create_publisher(JointState, "/robot/signaling/joints", qos_profile_system_default)

        self.debug = 0

        # declare params
        # declare the configuration data
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_name', 'movenet_thunder.tflite'),
                ('tracker_type', 'bounding_box'),
                ('classif_name', 'bounding_box'),
                ('classif_det_thresh', 10.0),
                ('min_human_frame_pct', 10.0),
                ('max_human_frame_pct', 90.0),
                ('min_arm_angle', 60.0),
                ('max_arm_angle', 125.0)
            ])

        self.config = {}
        self.params = [
            'model_name',
            'min_human_frame_pct',
            'max_human_frame_pct',
            'min_arm_angle',
            'max_arm_angle',
            'tracker_type',
            'classif_det_thresh'
        ]

        for param in self.get_parameters(self.params):
            self.config[param.name] = param.value

        self.estimation_model = self.config['model_name']
        self.tracker_type = self.config['tracker_type']

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # load model now
        # Notify users that tracker is only enabled for MoveNet MultiPose model.
        if self.tracker_type and ('movenet_multipose' not in self.estimation_model):
            self.get_logger().warning(
                'No tracker will be used as tracker can only be enabled for '
                'MoveNet MultiPose model.')

        # Initialize the pose estimator selected.
        if 'movenet_lightning' in  self.estimation_model or 'movenet_thunder' in self.estimation_model :
            self.pose_detector = Movenet(self.estimation_model )
        elif 'posenet' in self.estimation_model :
            self.pose_detector = Posenet(self.estimation_model )
        elif 'movenet_multipose' in self.estimation_model :
            self.pose_detector = MoveNetMultiPose(self.estimation_model , self.tracker_type)
        else:
            self.get_logger().fatal('ERROR: Model {} is not supported.'.format(self.estimation_model ))
            sys.exit('ERROR: Model is not supported.')

        self.get_logger().info("Startup complete, model is loaded")
        


    def new_frame(self, msg: Image):
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)
        if(self.debug > 3): self.get_logger().info("got frame")
        
        #cv2.imshow(self.estimation_model, current_frame)

        # have the NN process the image
        processed_frame = self.run(current_frame)

        # convert the frame back to an image
        processed_image = self.br.cv2_to_imgmsg(processed_frame, "bgr8")

        #publish the overlaid image
        self.image_pub.publish(processed_image)

    def debug_cb(self, msg: UInt8):
        self.debug = msg.data

    def run(self, image):

        if self.estimation_model == 'movenet_multipose':
            # Run pose estimation using a MultiPose model.
            list_persons = self.pose_detector.detect(image)
        else:
            # Run pose estimation using a SinglePose model, and wrap the result in an
            # array.
            list_persons = [self.pose_detector.detect(image)]

        # Draw keypoints and edges on input image
        image = utils.visualize(image, list_persons)

        return image
        


def main(args=None):
    rclpy.init(args=args)

    node = HumanPoseNode()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
