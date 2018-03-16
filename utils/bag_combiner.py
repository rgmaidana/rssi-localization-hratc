#!/usr/bin/env python

import rosbag
import sys

# List of topics I want to merge (all must have valid timestamps)
topics = ['/RTK/fix', '/RosAria/odom', '/imu/data', '/xbee_data']

if __name__ == "__main__":

	# Check for arguments
	if len(sys.argv) < 3:
		print "Usage: python bag_combiner.py <main bag> <RTK bag>\n"
		sys.exit(0)

	# Combine the two bags into new.bag
	with rosbag.Bag('new.bag', 'w') as newbag:
		# Record topics from the main bag into the new bag
		for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages(topics=topics[1:len(topics)]):
			newbag.write(topic, msg, msg.header.stamp)
		# Record topics from the RTK bag into the new bag
		for topic, msg, t in rosbag.Bag(sys.argv[2]).read_messages(topics=topics[0]):
			newbag.write(topic, msg, msg.header.stamp)