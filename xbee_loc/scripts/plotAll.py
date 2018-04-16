#!/usr/bin/env python

import numpy
import rospy
from xbee_loc.msg import Beacons
from matplotlib import pyplot as plt

# ROS messages
rssi_msg = Beacons()

# ROS topics to be subscribed
rssi_topic = 'xbee_data'

hold = True
def rssi_callback(msg):
	global rssi_msg
	global hold
	rssi_msg = msg
	hold = False

def listener():
	rospy.init_node('plotter',anonymous=True)
	rospy.Subscriber(rssi_topic, Beacons, rssi_callback)

	plt.ion()
	plt.show()
	k = 0

	fig, ax = plt.subplots()

	ax.scatter(k,rssi_msg.rssi[0], color='r', marker='^', label='TX1 RSSI')
	ax.scatter(k,rssi_msg.rssi[1], color='b', label='TX2 RSSI')
	ax.scatter(k,rssi_msg.rssi[2], color='g', marker='s', label='TX3 RSSI')
	ax.legend(loc='upper center', ncol=3, bbox_to_anchor=(0.5,1.1))

	while not rospy.is_shutdown():
		while hold:
			pass
		k = k+1
		ax.scatter(k,rssi_msg.rssi[0], color='r', marker='^', label='TX1 RSSI')
		ax.scatter(k,rssi_msg.rssi[1], color='b', label='TX2 RSSI')
		ax.scatter(k,rssi_msg.rssi[2], color='g', marker='s', label='TX3 RSSI')
		plt.draw()
		plt.pause(0.0001)

if __name__ == '__main__':
	listener()
