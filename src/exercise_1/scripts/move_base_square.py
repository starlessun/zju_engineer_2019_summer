#!/usr/bin/env python
#encoding:utf-8

""" move_base_square.py - Version 1.1 2013-12-20

    Command a robot to move in a square using move_base actions..

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.htmlPoint

"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
import json


class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        # How big is the square we want the robot to navigate?
        square_size = rospy.get_param("~square_size", 1.0) # meters

        # Create a list to hold the target quaternions (orientations)
        norminal_angle = quaternion_from_euler(0, 0, 0, axes='sxyz')
        norminal_quaternion = Quaternion(*norminal_angle)

        with open('mapdata.json') as data_file:
            mapdata = json.load(data_file)

        with open('obsdata.json') as data_file:
            obsdata = json.load(data_file)

        # Create a list to hold the waypoint poses
        waypoints = list()
        obstaclepoints = list()

        for i in range(1, mapdata[0]['size'] + 1):
            waypoints.append( Pose(Point(mapdata[i]['x'], mapdata[i]['y'], 0.0), norminal_quaternion))

        mapResolution = 0.5 ## unit:m

        mapPointsX = mapdata[0]['x_size']
        obstaclepoints = list()
        f = open("obsData.txt")
        line = f.readline()
        while line:
              x_index = int(line) / mapPointsX
              y_index = int(line) % mapPointsX
              obstaclepoints.append( Pose(Point( ( x_index*mapResolution), (y_index*mapResolution), 0.0), norminal_quaternion))
              line = f.readline()
        f.close()

        # Initialize the visualization markers for RViz
        self.init_waypoint_markers()
        self.init_obstacle_markers()

        # Set a visualization marker at each waypoint
        for waypoint in waypoints:
            p = Point()
            p = waypoint.position
            self.waypoint_markers.points.append(p)
        # Set a visualization marker at each waypoint
        for obstaclepoint in obstaclepoints:
            p = Point()
            p = obstaclepoint.position
            self.obstacle_markers.points.append(p)

        # Cycle through the four waypoints
        while not rospy.is_shutdown():
            # Update the marker display
            self.waypoint_marker_pub.publish(self.waypoint_markers)
            self.obstacle_marker_pub.publish(self.obstacle_markers)

    def init_waypoint_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}

        # Define a marker publisher.
        self.waypoint_marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)

        # Initialize the marker points list.
        self.waypoint_markers = Marker()
        self.waypoint_markers.ns = marker_ns
        self.waypoint_markers.id = marker_id
        self.waypoint_markers.type = Marker.CUBE_LIST
        self.waypoint_markers.action = Marker.ADD
        self.waypoint_markers.lifetime = rospy.Duration(marker_lifetime)
        self.waypoint_markers.scale.x = marker_scale
        self.waypoint_markers.scale.y = marker_scale
        self.waypoint_markers.color.r = marker_color['r']
        self.waypoint_markers.color.g = marker_color['g']
        self.waypoint_markers.color.b = marker_color['b']
        self.waypoint_markers.color.a = marker_color['a']

        self.waypoint_markers.header.frame_id = 'map'
        self.waypoint_markers.header.stamp = rospy.Time.now()
        self.waypoint_markers.points = list()

    def init_obstacle_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'obstaclepoints'
        marker_id = 0
        marker_color = {'r': 0.0, 'g': 0.7, 'b': 0.0, 'a': 1.0}

        # Define a marker publisher.
        self.obstacle_marker_pub = rospy.Publisher('obstacle_markers', Marker, queue_size=5)

        # Initialize the marker points list.
        self.obstacle_markers = Marker()
        self.obstacle_markers.ns = marker_ns
        self.obstacle_markers.id = marker_id
        self.obstacle_markers.type = Marker.CUBE_LIST
        self.obstacle_markers.action = Marker.ADD
        self.obstacle_markers.lifetime = rospy.Duration(marker_lifetime)
        self.obstacle_markers.scale.x = marker_scale
        self.obstacle_markers.scale.y = marker_scale
        self.obstacle_markers.color.r = marker_color['r']
        self.obstacle_markers.color.g = marker_color['g']
        self.obstacle_markers.color.b = marker_color['b']
        self.obstacle_markers.color.a = marker_color['a']

        self.obstacle_markers.header.frame_id = 'map'
        self.obstacle_markers.header.stamp = rospy.Time.now()
        self.obstacle_markers.points = list()


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
