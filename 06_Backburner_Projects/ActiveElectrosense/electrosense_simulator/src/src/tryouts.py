#!/usr/bin/env python

import rospy
import numpy as np
from scipy import signal
import math
import matplotlib.pyplot as plt

def draw_sqw():
	t = np.linspace(0, 1, 500, endpoint=False)
	plt.plot(t, signal.square(2 * np.pi * t))
	plt.ylim(-2, 2)
	plt.xlim(-2, 2)
	plt.show()

if __name__ == '__main__':
	rospy.init_node('test_node', anonymous=True)
	global now_first
	now_first = rospy.Time.now().secs
	draw_sqw()
	#rospy.spin()