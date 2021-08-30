#!/usr/bin/env python3
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from cheeyengsung_lab1.msg import scan_range

class Safety(object):
	def __init__(self):
		self.rate = rospy.Rate(10)
		self.acker_pub = rospy.Publisher("/brake", AckermannDriveStamped, queue_size = 1)
		self.bool_pub = rospy.Publisher("/brake_bool", Bool, queue_size = 1)
		self.scan_sub = message_filters.Subscriber("/scan", LaserScan)
		self.odom_sub = message_filters.Subscriber("/odom", Odometry)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.scan_sub, self.odom_sub],1,0.1)
		self.ts.registerCallback(self.callback)
				
	def callback(self, scan_msg, odom_msg):
		try:
			self.odom(odom_msg)
			rest_error = 0.01
			if self.speed > rest_error or self.speed < -rest_error:
				print(self.speed)
				self.scan(scan_msg)
				ttcs = self.distance/self.velocity
				min_ttc = min(ttcs)
				thres_ttc = 0.65
				if min_ttc < thres_ttc:
					brake_bool = Bool()
					brake_bool.data = True
					brake = AckermannDriveStamped()
					brake.drive.speed = 0
					self.bool_pub.publish(brake_bool)
					self.acker_pub.publish(brake)
		except rospy.ROSException as e:
#			rospy.logerr(e)
			pass

	def odom(self, odom_msg):
		twist = odom_msg.twist.twist
		self.speed = twist.linear.x
		
	def scan(self, scan_msg):
		self.filter_scan(scan_msg)
		self.proj_velocity()

	def filter_scan(self, scan_msg):	
		angle_range = enumerate(scan_msg.ranges)
		filter_data = [[count*scan_msg.angle_increment, val] for count, val in angle_range if not np.isinf(val) and not np.isnan(val)]
		filter_data = np.array(filter_data)
		self.angles = filter_data[:,0]
		self.distance = filter_data[:,1]
		self.filter_angle = np.array([])
		for angle in self.angles:
			if np.pi < angle < 2*np.pi:
				angle -= 2*np.pi
			self.filter_angle = np.append(self.filter_angle, angle)

	def proj_velocity(self):
		self.velocity = np.array([])
		for angle in self.filter_angle:
			velocity = -self.speed*np.cos(angle)
			if velocity < 0:
				velocity = 0
			self.velocity = np.append(self.velocity, velocity)

def main():
	rospy.init_node('chee0134_safety', anonymous=True)
	sn = Safety()
	rospy.spin()
			
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSException as e:
#		rospy.logerr(e)
		pass
