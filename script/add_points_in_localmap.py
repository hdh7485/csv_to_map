#!/usr/bin/env python

import rospy
import csv
import numpy
from tf import TransformListener
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 

class CSVRepublisher:
    def __init__(self):
        rospy.init_node("csv_to_map", anonymous=True)

        self.map_pub = rospy.Publisher("/csv_override_map", OccupancyGrid, queue_size=1)
        self.pose_array_pub = rospy.Publisher("/csv_pose_array", PoseArray, queue_size=1)

        rospy.Subscriber("/local_map", OccupancyGrid, self.map_callback)

        self.left_wall_csv_data = self.read_csv("../csv_map/geofence_left_total.csv")
        self.right_wall_csv_data = self.read_csv("../csv_map/geofence_right_total.csv")
        self.entire_csv_data = self.left_wall_csv_data
        [self.entire_csv_data.append(element) for element in self.right_wall_csv_data]

        self.map_header = Header()
        self.tf_listener = TransformListener()

    def read_csv(self, file_path):
        csv_data = []
        with open(file_path, "r") as f:
            rdr = csv.reader(f, delimiter='\t')
            for line in rdr:
                line.remove('')
                float_line = [float(i) for i in line]
                csv_data.append(float_line)
        return csv_data
    
    def map_callback(self, map_data):
        self.map_header = map_data.header;
        self.tf_csv_data = self.csv_transform(self.entire_csv_data)
        out_map = map_data
        out_map.header.frame_id = "/base_footprint"
        out_data = list(out_map.data)

        pose_array = PoseArray()
        pose_array.header = out_map.header

        for point in self.tf_csv_data:    
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            pose_array.poses.append(pose)

            index = int(point[1]/map_data.info.resolution) * int(map_data.info.width) + int(point[0]/map_data.info.resolution)
            if index < len(out_data):
                out_data[index] = 100
        out_map.data = tuple(out_data)
        self.map_pub.publish(out_map)
        self.pose_array_pub.publish(pose_array)
    
    def csv_transform(self, csv_data):
        odom_header = self.map_header
        odom_header.frame_id = "/odom"
        odom_header.stamp = rospy.Time.now()
        self.tf_listener.waitForTransform("/base_footprint", "/odom", rospy.Time.now(), rospy.Duration(4.0))
        mat44 = self.tf_listener.asMatrix('/base_footprint', odom_header)
        def xf(p):
            xyz = list(numpy.dot(mat44, numpy.array([p[1], p[0], 0.0, 1.0])))[:3]
            return xyz
        r = [xf(p) for p in csv_data]
        return r

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.spin()
            r.sleep()

if __name__ == "__main__":
    repub = CSVRepublisher()
    repub.run()