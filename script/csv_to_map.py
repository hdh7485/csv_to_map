#!/usr/bin/env python

import rospy
import rospkg
import csv
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

def read_csv(file_path):
    csv_data = []
    with open(file_path, "r") as f:
        rdr = csv.reader(f, delimiter='\t')
        for line in rdr:
            line.remove('')
            float_line = [float(i) for i in line]
            csv_data.append(float_line)
    return csv_data

def csv_to_gridmap(csv_data):
    header = Header()
    header.seq = "0"
    #header.frame_id = "/csv_map"
    header.frame_id = "/odom"
    header.stamp = rospy.Time.now()

    occupancy_grid = OccupancyGrid()
    occupancy_grid.header = header
    occupancy_grid.info.resolution = 1.0  #resolution is 0.1m
    #occupancy_grid.info.resolution = 0.1  #resolution is 0.1m
    occupancy_grid.info.width = 200 / occupancy_grid.info.resolution  
    occupancy_grid.info.height = 400 / occupancy_grid.info.resolution  
    occupancy_grid.info.origin.position.x = 0;
    occupancy_grid.info.origin.position.y = 0;
    occupancy_grid.info.origin.position.z = 0;
    occupancy_grid.info.origin.orientation.x = 0;
    occupancy_grid.info.origin.orientation.y = 0;
    occupancy_grid.info.origin.orientation.z = 0;
    occupancy_grid.info.origin.orientation.w = 1;

    grid_list = [0] * int(occupancy_grid.info.width * occupancy_grid.info.height)
    for pose in csv_data:
        index = int(pose[0]/occupancy_grid.info.resolution) * int(occupancy_grid.info.width) + int(pose[1]/occupancy_grid.info.resolution) 
        #print(index)
        grid_list[index] = 100
    occupancy_grid.data = grid_list

    return occupancy_grid

if __name__ == "__main__":
    rospy.init_node("csv_to_map", anonymous=True)
    map_pub = rospy.Publisher("/csv_map", OccupancyGrid, queue_size=1)

    rospack = rospkg.RosPack()
    left_geofence_path = rospack.get_path("csv_to_map") + "/csv_map/geofence_left_total.csv"
    right_geofence_path = rospack.get_path("csv_to_map") + "/csv_map/geofence_right_total.csv"

    self.left_wall_csv_data = self.read_csv(left_geofence_path)
    self.right_wall_csv_data = self.read_csv(right_geofence_path)

    csv_data = left_wall_csv_data
    [csv_data.append(element) for element in right_wall_csv_data]
    csv_map = csv_to_gridmap(csv_data)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo("publish csv map")
        map_pub.publish(csv_map) 
        rate.sleep()
