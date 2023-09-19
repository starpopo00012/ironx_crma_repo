#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def get_pose_from_csv(position, csv_file):
    with open(csv_file, 'r') as file:
        csv_reader = csv.reader(file)
        location_name = position
        for row in csv_reader:
            if len(row) < 5:
                rospy.logwarn("Skipping invalid row: {}".format(row))
                continue

            if row[0] == location_name:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = 'map'  # Set the frame ID as needed
                pose.pose.position.x = float(row[1])
                pose.pose.position.y = float(row[2])
                pose.pose.orientation.z = float(row[4])
                pose.pose.orientation.w = float(row[5])
                return pose

    return None

def send_pose(response):
    csv_file = '//home//por//catkin_ws//src//tesr_ros_ironx_driver_pkg//tesr_ros_ironx_driver_pkg//csv//pose_data.csv'  # Replace with the path to your CSV file

    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    #location_name = input("Enter the location name: ")  # Getlocation
    #response = input("Enter the location name: ")  # Python 3
    goal_pose = get_pose_from_csv(response.data, csv_file)
    if goal_pose:
        rospy.loginfo("Publishing goal for location: {}".format(type(response)))
        goal_publisher.publish(goal_pose)
    else:
        rospy.logwarn("Location '{}' not found in the CSV file.".format(type(response)))

def main():
    rospy.init_node('location_to_goal_node')
    rospy.Subscriber("position_call", String, send_pose)
    rospy.spin() # Wait for the publisher to initialize

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
