#!/usr/bin/env python

import os
import sys
import rospy
import signal
from sensor_msgs.msg import Imu
from xbee_loc.msg import Beacons

# ROS messages
rssi_msg = Beacons()
imu_msg = Imu()

# ROS topics to be subscribed
rssi_topic = 'xbee_data'
imu_topic = 'imu/data'

# Filenames
rssi_filename = './rssi_data.txt'
imu_filename = './imu_data.txt'

rssi_file = open(rssi_filename, 'a')
imu_file = open(imu_filename, 'a')

def handle_ctrl_c(signal, frame):
	rssi_file.close()
	imu_file.close()
	sys.exit(130)

def rssi_callback(msg):
	rssi_file.write("\n" + str(msg.rssi[0]) + ",")
	rssi_file.write(str(msg.rssi[1]) + ",")
	rssi_file.write(str(msg.rssi[2]))

def imu_callback(msg):
	imu_file.write("\n" + str(msg.orientation.x) + ",")
	imu_file.write(str(msg.orientation.y) + ",")
	imu_file.write(str(msg.orientation.z) + ",")
	imu_file.write(str(msg.orientation.w) + ",")

	imu_file.write(str(msg.angular_velocity.x) + ",")
	imu_file.write(str(msg.angular_velocity.y) + ",")
	imu_file.write(str(msg.angular_velocity.z) + ",")

	imu_file.write(str(msg.linear_acceleration.x) + ",")
	imu_file.write(str(msg.linear_acceleration.y) + ",")
	imu_file.write(str(msg.linear_acceleration.z))
	


def parser():
	rssi_data = rssi_msg.rssi
	imu_orientation = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w] 
	imu_angular = [imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z] 
	imu_linear = [imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z] 
	
	with open(rssi_file,'a') as file:
		file.write(str(rssi_msg.rssi))

'''	
	with open(imu_file, 'a') as file:
		file.write(str(imu_orientation[0]) + ",")
		file.write(str(imu_orientation[1]) + ",")
		file.write(str(imu_orientation[2]) + ",")
		file.write(str(imu_orientation[3]) + ",")

		file.write(str(imu_angular[0]) + ",")
		file.write(str(imu_angular[1]) + ",")
		file.write(str(imu_angular[2]) + ",")
		
		file.write(str(imu_linear[0]) + ",")
		file.write(str(imu_linear[1]) + ",")
		file.write(str(imu_linear[2]) + "\n")
'''

def listener():
	rospy.init_node('parser',anonymous=True)
	rospy.Subscriber(rssi_topic, Beacons, rssi_callback)
	rospy.Subscriber(imu_topic, Imu, imu_callback)

	rospy.loginfo("Subscribed to topics {} and {}".format(rssi_topic, imu_topic))
	rospy.loginfo("Parsing data to files {} and {}".format(rssi_filename, imu_filename))

	rospy.spin()

if __name__ == '__main__':
	signal.signal(signal.SIGINT, handle_ctrl_c)
	listener()