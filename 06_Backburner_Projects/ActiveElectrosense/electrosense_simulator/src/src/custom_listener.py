#!/usr/bin/env python
import rospy
from hdt_nri_description.msg import Jangles
import tf
from sensor_msgs.msg import JointState
from math import pi

def callback(data):
    rospy.loginfo("%f, %f, %f" % (data.joint1, data.joint2, data.joint3))
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    #rate = rospy.Rate(20)
    joint_state = JointState()

    joint_state.header.stamp = rospy.Time.now()
    t = rospy.get_time()
    joint_state.name = ['gantryX','gantryY','gantryZ','gantryYaw','joint1','joint2','joint3','thumb_base','thumb_prox','index_prox','ring_prox']

    #i = random.uniform(0, 1)
    jangs = [data.gantry_x, data.gantry_y, data.gantry_z, data.gantry_yaw, data.joint1, data.joint2, data.joint3, data.thumb_base, data.thumb_joint, data.index_joint, data.ring_joint]
    #Fingers are zero to pi/2, arm is -pi/2 to pi/2

    print jangs

    joint_state.position = jangs #[0,0,0,0, jangs[0],jangs[1],jangs[2], 0, 0, 0, 0] # update joint_states with time
    joint_pub.publish(joint_state)

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber('jangles_chatter', Jangles, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()