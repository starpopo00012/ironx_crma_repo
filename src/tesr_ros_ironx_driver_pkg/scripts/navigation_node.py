#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import PoseStamped

def get_pose_from_csv(location_name, csv_file):
    with open(csv_file, 'r') as file:
        csv_reader = csv.reader(file)
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
                pose.pose.position.z = float(row[3])
                pose.pose.orientation.z = float(row[4])
                pose.pose.orientation.w = float(row[5])
                return pose

    return None

def main():
    rospy.init_node('location_to_goal_node')
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Wait for the publisher to initialize

    csv_file = 'pose_data.csv'  # Replace with the path to your CSV file

    while not rospy.is_shutdown():
        location_name = input("Enter the location name: ")  # Python 2
        # location_name = input("Enter the location name: ")  # Python 3

        goal_pose = get_pose_from_csv(location_name, csv_file)
        if goal_pose:
            rospy.loginfo("Publishing goal for location: {}".format(location_name))
            goal_publisher.publish(goal_pose)
        else:
            rospy.logwarn("Location '{}' not found in the CSV file.".format(location_name))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
