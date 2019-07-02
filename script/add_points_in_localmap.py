#!/usr/bin/env python
import rospy
import rospkg
import csv
import numpy
import time

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 
from lcm_to_ros.msg import hyundai_mission

import tf2_ros
import tf2_py as tf2
from tf2_geometry_msgs.tf2_geometry_msgs import transform_to_kdl
import PyKDL

class CSVRepublisher:
    def __init__(self):
        rospy.init_node("csv_to_map", anonymous=True)

        rospack = rospkg.RosPack()
        #left_geofence_path = rospack.get_path("csv_to_map") + "/csv_map/geofence_first_lane.csv"
        left_geofence_path = rospack.get_path("csv_to_map") + "/csv_map/geofence_left_lane.csv"
        right_geofence_path = rospack.get_path("csv_to_map") + "/csv_map/geofence_right_shift.csv"

        self.left_wall_csv_data = self.read_csv(left_geofence_path)
        self.right_wall_csv_data = self.read_csv(right_geofence_path)
        self.entire_csv_data = self.left_wall_csv_data
        [self.entire_csv_data.append(element) for element in self.right_wall_csv_data]
        #self.entire_csv_data = self.right_wall_csv_data

        self.odom_header = Header()
        self.odom_header.frame_id = "/odom"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.mission_number = 0

        self.map_pub = rospy.Publisher("/csv_override_map", OccupancyGrid, queue_size=1)
        self.pose_array_pub = rospy.Publisher("/csv_pose_array", PoseArray, queue_size=1)
        rospy.Subscriber("/local_map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/lcm_to_ros/LCM2ROS_mission", hyundai_mission, self.mission_num_callback)

    def read_csv(self, file_path):
        csv_data = []
        with open(file_path, "r") as f:
            rdr = csv.reader(f, delimiter='\t')
            for line in rdr:
                line.remove('')
                float_line = [float(i) for i in line]
                csv_data.append(float_line)
        return csv_data

    def mission_num_callback(self, hyundai_data):
        self.mission_number = hyundai_data.mission_number
        
    
    def map_callback(self, map_data):
        out_map = map_data
        out_map.header.frame_id = "/base_footprint"
        out_map.header.stamp = rospy.Time.now()
        out_data = list(out_map.data)

        if self.mission_number == 6:
            time1 = time.time()
            self.tf_csv_data = self.csv_transform(self.entire_csv_data)
            time2 = time.time()

            pose_array = PoseArray()
            time3 = time.time()
            for point in self.tf_csv_data:    
                index = int((point[1] - map_data.info.origin.position.y)/map_data.info.resolution) * int(map_data.info.width) + int((point[0] - map_data.info.origin.position.x)/map_data.info.resolution)
                #rospy.loginfo("%d", index)
                if index < len(out_data) and index >= 0:
                    out_data[index] = 100

            out_map.data = tuple(out_data)
            time4 = time.time()

            self.map_pub.publish(out_map)
            time5 = time.time()
            #rospy.loginfo("%f %f %f %f", time2 - time1, time3 - time2, time4 - time3, time5 - time4)
        else:
            self.map_pub.publish(out_map)
    
    def csv_transform(self, csv_data):
        #self.tf_listener.waitForTransform("/base_footprint", "/odom", rospy.Time(0), rospy.Duration(4.0))
        trans = self.tf_buffer.lookup_transform('base_footprint', 'odom',
                                           rospy.Time(0),
                                           rospy.Duration(4.0))
        t_kdl = transform_to_kdl(trans) 
        r = [t_kdl * PyKDL.Vector(p[1], p[0], 0.0) for p in csv_data]
        return r

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    repub = CSVRepublisher()
    repub.run()
