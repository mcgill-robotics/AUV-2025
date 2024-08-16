#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
import quaternion
from tf_transformations import euler_from_quaternion
from pyproj import Transformer, CRS
from pyproj.aoi import AreaOfInterest
from pyproj.database import query_utm_crs_info
from time import strftime
import time
import os
import json

DEG_PER_RAD = 180 / np.pi

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')

        self.gps = None
        self.depth = None
        self.seen_pose = False
        self.seen_image = False
        self.image = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.title = strftime("%d_%m_%Y_%H:%M:%S")
        self.bridge = CvBridge()
        self.camera_info_seen = False
        
        # Declare parameters
        self.declare_parameter('latitude_offset', 0.0)
        self.declare_parameter('longitude_offset', 0.0)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('output_dir', '/tmp')

        # Retrieve parameters
        self.latitude_offset = self.get_parameter('latitude_offset').get_parameter_value().double_value
        self.longitude_offset = self.get_parameter('longitude_offset').get_parameter_value().double_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value
        self.data_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        
        self.output_dir = f"{self.data_dir}/{self.title}"
        os.makedirs(self.output_dir, exist_ok=True)

        # Setup CRS transformation
        utm_crs_list = query_utm_crs_info(
            datum_name="WGS 84",
            area_of_interest=AreaOfInterest(
                west_lon_degree=self.longitude_offset,
                south_lat_degree=self.latitude_offset,
                east_lon_degree=self.longitude_offset,
                north_lat_degree=self.latitude_offset,
            ),
        )
        utm_crs = CRS.from_epsg(utm_crs_list[0].code)
        self.forwards = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True)
        self.backwards = Transformer.from_crs(utm_crs, "EPSG:4326", always_xy=True)
        self.east_offset, self.north_offset = self.forwards.transform(self.latitude_offset, self.longitude_offset)

        # Setup subscriptions
        self.pose_sub = self.create_subscription(Pose, '/state/pose', self.pose_callback, 10)
        self.image_sub = self.create_subscription(Image, '/vision/down_cam/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/vision/down_cam/camera_info', self.camera_info_callback, 10)

        # Initialize text file
        self.output_txt = open(self.output_dir + "/geo.txt", "w")
        self.output_txt.write("EPSG:4326\n")

    def camera_info_callback(self, msg):
        if self.camera_info_seen:
            return
        self.camera_info_seen = True

        # Extract camera parameters
        width = msg.width
        height = msg.height
        distortion_model = msg.distortion_model
        focal_x = msg.K[0]
        focal_y = msg.K[4]
        c_x = msg.K[2]
        c_y = msg.K[5]
        k1 = msg.D[0]
        k2 = msg.D[1]
        p1 = msg.D[2]
        p2 = msg.D[3]
        k3 = msg.D[4]

        # Format camera parameters
        camera_params = {
            f"{width} {height} {distortion_model} {focal_x:.8f}": {
                "projection_type": distortion_model,
                "width": width,
                "height": height,
                "focal_x": focal_x,
                "focal_y": focal_y,
                "c_x": (c_x - (width / 2)) / (width / 2),
                "c_y": (c_y - (height / 2)) / (height / 2),
                "k1": k1,
                "k2": k2,
                "p1": p1,
                "p2": p2,
                "k3": k3,
            }
        }

        with open(self.output_dir + "/cameras.json", "w") as camera_txt:
            camera_txt.write(json.dumps(camera_params))

    def pose_callback(self, msg):
        new_north, new_east = self.north_offset + msg.position.x, self.east_offset - msg.position.y
        self.gps = self.backwards.transform(new_east, new_north)
        self.depth = msg.position.z

        quaternion_nwu = np.quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )
        quaternion_ned = (
            np.quaternion(0, 1, 0, 0) * quaternion_nwu * np.quaternion(0, 1, 0, 0)
        )
        self.yaw, self.pitch, self.roll = euler_from_quaternion(
            [quaternion_ned.x, quaternion_ned.y, quaternion_ned.z, quaternion_ned.w], "szyx"
        )
        self.yaw *= DEG_PER_RAD
        self.pitch *= DEG_PER_RAD
        self.roll *= DEG_PER_RAD
        self.seen_pose = True

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.seen_image = True

    def save_data(self):
        if self.seen_pose and self.seen_image:
            millis = str(int(round(time.time() * 1000)))[0:3]
            the_time = strftime(f"%H:%M:%S.{millis}")
            self.title = the_time + ".jpg"
            self.output_txt.write(
                f"{self.title} {self.gps[0]:10.13f} {self.gps[1]:10.13f} {self.depth} {self.yaw} {self.pitch} {self.roll}\n"
            )
            cv2.imwrite(self.output_dir + "/" + self.title, self.image)

    def shutdown(self):
        self.get_logger().info("Shutting down")
        self.output_txt.close()

def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    try:
        while input("Press Enter to capture, 'x' to finish: ") != "x":
            node.save_data()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
