#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point


from hdt_nri_description.msg import PathPts
from math import cos, sin, pi, sqrt, atan2
import robot_calc_functions as r
import numpy as np
from scipy.optimize import fsolve
import math

from random import random

server = None
counter = 0
dead_zone = 7

def processFeedback( feedback ):

    global counter
    counter += 1

    #print t_new
    msg = PathPts()

    if(counter == dead_zone):
        global xg
        xg = np.round((feedback.pose.position.x)*1000)/1000

        global yg
        yg = np.round((feedback.pose.position.y)*1000)/1000

        global zg
        zg = np.round((feedback.pose.position.z)*1000)/1000

        rospy.loginfo( str(xg) + "," + str(yg) + "," + str(zg))
        [msg.x_goal, msg.y_goal, msg.z_goal] = [xg,yg,zg]

        #rospy.loginfo(msg)
        pub.publish(msg)

        counter = 0
    
    server.applyChanges()

def makeBox( msg ):
    marker = Marker()
    scale_value = 0.25

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * scale_value
    marker.scale.y = msg.scale * scale_value
    marker.scale.z = msg.scale * scale_value
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def saveMarker( int_marker ):
    server.insert(int_marker, processFeedback)

def makeMarker(interaction_mode, position, show_6dof = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"
    int_marker.pose.position = position
    int_marker.scale = 0.1

    int_marker.name = "bob"
    int_marker.description = "Here's Bob"

    # insert a box
    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 1
    # control.orientation.y = 0
    # control.orientation.z = 0
    # control.name = "rotate_x"
    # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    # int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 0
    # control.orientation.y = 1
    # control.orientation.z = 0
    # control.name = "rotate_z"
    # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    # int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)


    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 0
    # control.orientation.y = 0
    # control.orientation.z = 1
    # control.name = "rotate_y"
    # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    # int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

if __name__=="__main__":
    rospy.init_node("feedback_pt")
    server = InteractiveMarkerServer("feedback_pt")

    global pub
    pub = rospy.Publisher('path_chatter', PathPts, queue_size=1)
    #rate = rospy.Rate(5) #5hz
    #rate.sleep() #to be used after publishing to maintain regular intervals

    position = Point( 0, 0, 0)
    makeMarker(InteractiveMarkerControl.MOVE_3D, position, False)
    #MOVE_3D

    server.applyChanges()

    rospy.spin()