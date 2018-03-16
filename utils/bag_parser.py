#!/usr/bin/env python
'''
	Parse data from RTK log (post-processed) and RosBag
'''

import os
import rosbag
from sensor_msgs.msg import Imu, NavSatFix
from xbee_loc.msg import Beacons
import sys

# List of topics I want to merge (all must have valid timestamps)
topics = ['/RTK/fix', '/RosAria/odom', '/imu/data', '/xbee_data']

if __name__ == '__main__':
	# Check for arguments
	if len(sys.argv) < 3:
		print 'Usage: python bag_parser.py <main_bag> <combined_bag>\n'
		sys.exit(0)

	# Lists for data
	rtk_data = []
	odom_data = []
	imu_data = []
	xbee_data = []
	gps_data = []

	# Get data from rosbag
	with rosbag.Bag(sys.argv[2], 'r') as bag:
		for topic, msg, t in bag.read_messages(topics=topics[0]):
			rtk_data.append(msg)
		for topic, msg, t in bag.read_messages(topics=topics[1]):
			odom_data.append(msg)
		for topic, msg, t in bag.read_messages(topics=topics[2]):
			imu_data.append(msg)
		for topic, msg, t in bag.read_messages(topics=topics[3]):
			xbee_data.append(msg)
			
	with rosbag.Bag(sys.argv[1], 'r') as bag:
		for topic, msg, t in bag.read_messages(topics=['/gps/fix']):
			gps_data.append(msg)

	# Create log directories if they don't exist
	rtk_dir = os.getcwd() + '/parsed/RTK'
	if not os.path.exists(rtk_dir):
		os.makedirs(rtk_dir)
	gps_dir = os.getcwd() + '/parsed/gps'
	if not os.path.exists(gps_dir):
		os.makedirs(gps_dir)
	odom_dir = os.getcwd() + '/parsed/odom'
	if not os.path.exists(odom_dir):
		os.makedirs(odom_dir)
	imu_dir = os.getcwd() + '/parsed/imu'
	if not os.path.exists(imu_dir):
		os.makedirs(imu_dir)
	xbee_dir = os.getcwd() + '/parsed/xbee'
	if not os.path.exists(xbee_dir):
		os.makedirs(xbee_dir)

	# Record data for RTK (lat, lon, height, time)
	rtk_filename = rtk_dir + '/RTK_log.txt'
	with open(rtk_filename, 'w') as file:
		for data in rtk_data:
			file.write('%.15f,%.15f,%.15f,%f\n' % (data.latitude, data.longitude, data.altitude, data.header.stamp.to_sec()))

	# Record data for GPS (lat, lon, height, time)
	gps_filename = gps_dir + '/gps_log.txt'
	with open(gps_filename, 'w') as file:
		for data in gps_data:
			file.write('%.15f,%.15f,%.15f,%f\n' % (data.latitude, data.longitude, data.altitude, data.header.stamp.to_sec()))

	# Record data for odom (position[x;y;z], orientation[x;y;z;w], time)
	odom_filename = odom_dir + '/odom_log.txt'
	with open(odom_filename, 'w') as file:
		for data in odom_data:
			file.write('%f,%f,%f,%f,%f,%f,%f,%f\n' % (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, 
													  data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w, 
													  data.header.stamp.to_sec()))

	# Record data for imu (orientation[x;y;z;w], angular_velocity[x;y;z], linear_acceleration[x;y;z], time)
	imu_filename = imu_dir + '/imu_log.txt'
	with open(imu_filename, 'w') as file:
		for data in imu_data:
			file.write('%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n' % (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w, 
															   data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z, 
															   data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z,
															   data.header.stamp.to_sec()))

	# Record data for xbee (rssi[tx1;tx2;tx3], time)
	xbee_filename = xbee_dir + '/xbee_log.txt'
	with open(xbee_filename, 'w') as file:
		for data in xbee_data:
			file.write('%.f,%.f,%.f,%f\n' % (data.rssi[0], data.rssi[1], data.rssi[2], 
										  data.header.stamp.to_sec()))