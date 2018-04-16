#!/usr/bin/env python

'''
	XBEE_NODE
	Author: Renan Guedes Maidana
	Description: ROS implementation of the XBee receiver node.
	It sends a remote DB command to the end-devices connected to the ZigBee network,
	one at a time, whose addresses are specified in the XBee address YAML table.
	Once all RSSI values have been obtained, the node publishes them in a custom message
	type on the "/xbee_data" topic.
'''

import os
import sys
import time 						# For timed delays
import yaml 						# For getting XBee addresses in the YAML table
import rospy
import rospkg 						# Handles directory information on ROS packages
import signal 						# Handles keyboard interrupts
import argparse 					# Smartly parses command-line arguments
from xbee import XBee, ZigBee 		# Custom library to handle API frame XBee communications
from serialDevice import Device 	# Custom library that abstracts a serial port as a device
from xbee_loc.msg import Beacons 	# Custom message type specific to this ROS package

# ROS Parameters
pub_topic = 'xbee_data' # Topic to be published
pub_freq = 100 			# Maximum update rate of the topic

# Beacon messages
rssi_msg = Beacons()
rssi_msg.header.frame_id = 'Beacons'
rssi_msg.header.seq = 0

# Argument parser
parser = argparse.ArgumentParser()

# Regular arguments
parser.add_argument("-b", "--baud",
                    default=9600,
                    help='defines baud rate of serial port')
parser.add_argument("-o", "--output",
                    default='serialOutput.txt',
                    help='filename for text file with serial port output')
parser.add_argument("-V", "--vid",
                    default='0403',
                    help='vendor ID of device')

# Parse known arguments
args, unknown = parser.parse_known_args()

# Create XBee explorer device object
expl = Device(name='XBee Explorer', vid=args.vid, baud=args.baud, timeout=1)

# Create a mock xbee device to get the hardware version
expl.connect()
mock_xbee = XBee(expl.ser, escaped=True)
mock_xbee.send('at', frame_id='\x01', command="DD")
response = mock_xbee.wait_read_frame()
device_descr = response['parameter'].encode('hex')[0:4]
mock_xbee.halt()
expl.disconnect()

# Get addresses for the XBees from the YAML address table
addr_path = rospkg.RosPack().get_path('xbee_loc') + '/param/addresses.yaml'
addr = yaml.load(file(addr_path, 'r'))
rx_addr = addr[device_descr]['RX']['SH'] + addr[device_descr]['RX']['SL']
tx1_addr = addr[device_descr]['TX1']['SH'] + addr[device_descr]['TX1']['SL'] 	# Beacon 1
tx2_addr = addr[device_descr]['TX2']['SH'] + addr[device_descr]['TX2']['SL']	# Beacon 2
tx3_addr = addr[device_descr]['TX3']['SH'] + addr[device_descr]['TX3']['SL']	# Beacon 3

# Handle keyboard interrupt
def handle_ctrl_c(signal, frame):
	xbee.halt()			# Stop communication with the XBee
	expl.disconnect()	# Disconnect serial port
	sys.exit(130)		# Exit

# Initial values for RSSI (120 is the maximum allowed in the XBee, an absurd amount)
rssi_data = [120, 120, 120]

# Callback function called when a message is received in the XBee
def rxCallback(data):
	global rssi_data
	
	# Get address from sender
	addr = ':'.join("{:02x}".format(ord(c)) for c in data['source_addr_long']).split(':')
	addr = ''.join(addr)
	
	# Get RSSI from sender
	rssi = ord(data['parameter'])
	
	# Update data accordingly
	if addr == tx1_addr:			# If it came from beacon 1
		rssi_data[0] = int(rssi)
	elif addr == tx2_addr:			# If it came from beacon 2
		rssi_data[1] = int(rssi)
	elif addr == tx3_addr:			# If it came from beacon 3
		rssi_data[2] = int(rssi)

# Function to request RSSI from a transmitter
def request_DB(addr, t):
	# Wait some time
	time.sleep(t)

	# Send a message to target XBee to update DB values
	if device_descr == '0001':		# If XBee is Series 1
		xbee.send('tx_long_addr',
			  	  frame_id=b'\x01',
			  	  dest_addr=bytes(bytearray.fromhex(addr)),
			  	  data='A')
	else:
		xbee.send('tx',
			  	  frame_id=b'\x01',
			  	  dest_addr_long=bytes(bytearray.fromhex(addr)),
				  data='A')

	# Wait some more time
	time.sleep(t)
		
	# Request DB information and get response
	xbee.send('remote_at',
			  frame_id=b'\x01',
			  dest_addr_long=bytes(bytearray.fromhex(addr)),
			  command='DB')


# Main
if __name__ == '__main__':
	# Handle CTRL+C interruption
	signal.signal(signal.SIGINT, handle_ctrl_c)

	# Create publisher for RSSI data and initiate node
	pub = rospy.Publisher('xbee_data', Beacons, queue_size=10)
	rospy.init_node(pub_topic, anonymous=True)
	rate = rospy.Rate(pub_freq)		# Topic update rate

	# Connect to the XBee via Serial
	rospy.loginfo("Connecting...")
	connected, _ = expl.connect()
	if connected:
		rospy.loginfo("Connected to device at %s" % expl.port)
	else:
		rospy.loginfo("Could not connect to device")
		exit()

	# Initialize xbee library with the explorer instance
	if device_descr == '0001':		# If XBee is Series 1
		xbee = XBee(expl.ser, callback=rxCallback, escaped=True)
	else:
		xbee = ZigBee(expl.ser, callback=rxCallback, escaped=True)

	# Run until node is killed
	rospy.loginfo("Publishing at topic \'%s\' at a rate of %d Hz", pub_topic, pub_freq)
	while not rospy.is_shutdown():
		try:
			# Request RSSI from the three beacons
			request_DB(tx1_addr, 0.01)
			request_DB(tx2_addr, 0.01)
			request_DB(tx3_addr, 0.01)

			# Get current time for message timestamp
			rssi_msg.header.stamp = rospy.Time.from_sec(time.time())
			# Put the three RSSI in the custom message
			rssi_msg.rssi = rssi_data	

			pub.publish(rssi_msg)	# Publish the message
			rate.sleep()			# Sleep for 1/pub_freq seconds
		except KeyboardInterrupt:
			break

	# At this point, the node was shutdown normally
	xbee.halt()			# Stop communication with the XBee
	expl.disconnect()	# Disconnect serial port