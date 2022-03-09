#!/usr/bin/env python3

import rospy

from ai2thor.controller import Controller
# from ai2thor.server import Event
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np

class Ai2Thor:
    def __init__(self, scene_number = 1, crouch = True):
        self.controller = Controller(agentMode="default",
                                    visibilityDistance = 1,
                                    scene = ("FloorPlan{}".format(scene_number)),

                                    # step sizes
                                    gridSize = 0.001,
                                    snapToGrid = True,
                                    rotateStepDegrees = 90,

                                    # image modalities
                                    renderDepthImage = True,
                                    renderInstanceSegmentation = False,

                                    # camera properties
                                    width = 600,
                                    height = 600,
                                    fieldOfView = 90)
        if crouch:
            self.controller.step(action="Crouch")

        self.br = CvBridge()
        
        self.pub_camera = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size = 10)
        self.pub_depth = rospy.Publisher('/camera/depth/image_raw', Image, queue_size = 10)
        self.pub_cam_info = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size = 10)
        self.pub_depth_cam_info = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size = 10)
        
        self.sub_speed = rospy.Subscriber("/cmd_vel", Twist, self.update_pose)

        self.time = rospy.get_time()

        rospy.loginfo("Ai2Thor Ros Node Started")

    def update_pose(self, msg):
        dt = rospy.get_time() - self.time
        # print(msg)
        # vx = 
        event = self.controller.step("MoveAhead", moveMagnitude = msg.linear.x * dt)
        print("MoveAhead Successful:", event.metadata["lastActionSuccess"])
        self.controller.step("RotateLeft", degrees = msg.angular.z * dt * 10)

        self.time = rospy.get_time()
        
    def get_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera"

        return header

    def cvt_img_to_msg(self, image, header, encoding):
        img_msg = self.br.cv2_to_imgmsg(image)
        img_msg.header = header

        if encoding == "rgb8":
            img_msg.encoding = encoding

        return img_msg

    def publish_images(self, header):
        event = self.pub_camera.publish(self.cvt_img_to_msg(self.controller.last_event.frame, header, "rgb8"))
        print(event)
        self.pub_depth.publish(self.cvt_img_to_msg(self.controller.last_event.depth_frame, header, "mono16"))
        # print(type(self.controller.last_event.depth_frame))
    

    def publish_camera_info(self, header, width = 600, height = 600, fov = 90):
        cam_info = CameraInfo()
        cam_info.header = header

        focal_length = 0.5 * width * np.tan(np.deg2rad(fov/2))
        fx, fy, cx, cy = (focal_length, focal_length, width / 2, height / 2)

        cam_info.width = width
        cam_info.height = height
        cam_info.distortion_model = "plumb_bob"
        cam_info.K = np.float32([fx, 0, cx, 0, fy, cy, 0, 0, 1])
        cam_info.D = np.float32([0, 0, 0, 0, 0])
        cam_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

        self.pub_cam_info.publish(cam_info)
        self.pub_depth_cam_info.publish(cam_info)
        print(self.sub_speed.get_num_connections())
        
    def publish_state(self, freq):
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            # rospy.loginfo("Hello")
            header = self.get_header()
            self.publish_images(header)
            self.publish_camera_info(header)
            rate.sleep()
    
    
if __name__ == '__main__':
    rospy.init_node("ai2thor_rosnode", anonymous = False)
    rosnode = Ai2Thor(302)
    rosnode.publish_state(freq = 20)
    rospy.spin()