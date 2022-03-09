#!/usr/bin/env python3

import math,time
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import pyads

ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]
ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]
ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]




class iAmechROS():
	def __init__(self):
		rospy.init_node('iAmech', log_level=rospy.DEBUG)
		
		# Connection
		PLC_AMS_ID = '192.168.100.100.1.1'
		PLC_IP = '192.168.100.100'
		self.connectToPLC(PLC_IP)
		
		print("Initialize connection to PLC")
		self.plc = pyads.Connection(PLC_AMS_ID, 801, PLC_IP)
		self.plc.open()
		print("Connection built!")
		
		# read voltage
		l_volt = self.plc.read_by_name(".SLAM_L[18]", pyads.PLCTYPE_DINT)
		r_volt = self.plc.read_by_name(".SLAM_R[18]", pyads.PLCTYPE_DINT)
		print("Current voltage is {}v {}v".format(l_volt, r_volt))
		if l_volt < 42 or r_volt < 42:
			print("AMR NEEDS CHARGING")
		
		# Unlock motor
		self.plc.write_by_name(".bSLAM_ServeON", 1, pyads.PLCTYPE_BOOL)
		x = self.plc.read_by_name(".bSLAM_ServeON",pyads.PLCTYPE_BOOL)
		print("set to %s" %x)
		time.sleep(1)
		
		# Cleanup when terminating the node
		rospy.on_shutdown(self.shutdown)
		
		
		# Subscribe to topics	
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
		# Odom
		self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
		self.odom_broadcaster = tf.TransformBroadcaster()
		
		self.rate = float(rospy.get_param("~base_controller_rate", 10))
		
		self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
		# self.stopped = False
		# not used
		# wheel_radius = 85  
		length = 589  
		self.theta_dot = 0
		self.d_th = 0
		self.t_last = 0
		self.x_dot= 0
		self.y_dot= 0
		self.x = 0
		self.y = 0
		self.theta = 0
		self.left_wheel = 0
		self.right_wheel = 0
		
		now = rospy.Time.now()
		self.then = now
		
		now = rospy.Time.now()    
		self.then = now # time for determining dx/dy
		self.t_delta = rospy.Duration(1.0 / self.rate)
		self.t_next = now + self.t_delta
		self.last_cmd_vel = now
		
		while not rospy.is_shutdown():
			self.poll()
				
		rospy.spin()
		
	def connectToPLC(self, plc_ip):
		pyads.open_port()
		pyads.set_local_address('1.2.3.4.1.1')
		pyads.close_port()

		pyads.open_port()
		pyads.add_route('192.168.100.100.1.1', '192.168.100.100')
		pyads.close_port()

		SENDER_AMS = '1.2.3.4.1.1'
		PLC_IP = '192.168.100.100'
		PLC_USERNAME = 'Administrator'
		PLC_PASSWORD = '1'
		ROUTE_NAME = 'RouteToMyPC'
		HOSTNAME = '192.168.100.191'
		# PLC_AMS_ID = '192.168.100.100.1.1'

		pyads.add_route_to_plc(SENDER_AMS, HOSTNAME, plc_ip, PLC_USERNAME, PLC_PASSWORD, route_name=ROUTE_NAME)
		
		
		
	def cmdVelCallback(self, req):
		self.last_cmd_vel = rospy.Time.now()
		
		L = 630 # mm
		R = 85 # mm
		
		x = req.linear.x	# m/s
		th = req.angular.z # rad/s

		v = int(x * 1000) # mm/s
		
		vr = 20*(2*v + 0.2*th*L) / (R) 
		vl = 20*(2*v - 0.2*th*L) / (R) 
		
		
		self.left_wheel = int(vl)
		self.right_wheel = int(vr)
		print(f'Wheels vel {self.left_wheel} {self.right_wheel}')
	
	def poll(self):
		now = rospy.Time.now()
		if now < self.t_next:
			return
		#l_vv = self.plc.read_by_name(".SLAM_L[2]", pyads.PLCTYPE_DINT)
		#r_vv = self.plc.read_by_name(".SLAM_R[2]", pyads.PLCTYPE_DINT)
		l_vv = self.plc.read_by_name(".SLAM_L[15]", pyads.PLCTYPE_DINT)
		r_vv = self.plc.read_by_name(".SLAM_R[15]", pyads.PLCTYPE_DINT)
		l_v = l_vv/1000
		r_v = r_vv/1000
		#t_now_ros = rospy.Time.now()
		delta_t = now - self.then
		self.then = now
		delta_t = delta_t.to_sec()
		"""
		if 1==1: #l_v != 0 and r_v != 0:
			
		        if delta_t > 2:
			        delta_t = 0
			
			#if abs(l_v) <= 48:
			#    l_v = l_v + 0.5 * l_v * delta_t
			#if abs(r_v) <= 48:
			#    r_v = r_v + 0.5 * r_v * delta_t
				  
			# print("TIME")
			# print(delta_t)
			# print("----------------")
		
		if abs(l_vv) < 3 and abs(r_vv) < 3 :
			l_v = 0
			r_v = 0
		"""
		R = 0.085 # m
		L = 0.630 # m
		self.x_dot = (l_v + r_v) * cos(self.theta) / 2
		self.y_dot = (l_v + r_v) * sin(self.theta) / 2
		
		self.theta_dot = (1 / L) * (r_v - l_v) # 589=>630
			
		# print(f'theta_dot {self.theta_dot}')
		self.x += self.x_dot * delta_t
		self.y += self.y_dot * delta_t
		self.theta += (self.theta_dot * (delta_t))
			
		self.theta = self.theta % (math.pi * 2)
		# print(f'theta {self.theta}')
		# print(l_v)
		# print(r_v)			print(quaternion.w)
			
		'''
		print("====== x, dx, y, dy, dt, dth, lv, rv======")
		print(self.x)
		print(self.x_dot)
		print(self.y)
		print(self.y_dot)
		print(delta_t)
		print(self.theta_dot)
		print(l_v)
		print(r_v)
		print(f'theta {self.theta}')
		'''
		x_2 = self.x 
		y_2 = self.y 
		quaternion = Quaternion()
		quaternion.x = 0.0
		quaternion.y = 0.0
		quaternion.z = sin(self.theta/2.0)
		quaternion.w = cos(self.theta/2.0)
		# print(f'quaternion z {quaternion.z}')
		# print(f'quaternion w {quaternion.w}')
		self.odom_broadcaster.sendTransform((x_2, y_2, 0.0), (quaternion.x, quaternion.y, quaternion.z, quaternion.w), now, "base_link", "odom")
			
			
		odom = Odometry()
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_link"
		odom.header.stamp = now
		odom.pose.pose.position.x = x_2 
		odom.pose.pose.position.y = y_2 
		odom.pose.pose.position.z = 0
		odom.pose.pose.orientation = quaternion
		odom.twist.twist.linear.x = self.x_dot
		odom.twist.twist.linear.y =  self.y_dot
		odom.twist.twist.angular.z = self.theta_dot
			
		self.odom_pub.publish(odom)
		#time.sleep(0.1)
		"""
		else:
			x_2 = self.x /100
			y_2 = self.y /100
			t_now = time.time()
			quaternion = Quaternion()
			quaternion.x = 0.0
			quaternion.y = 0.0
			quaternion.z = sin(self.theta / 2.0)
			quaternion.w = cos(self.theta / 2.0)
			self.odom_broadcaster.sendTransform((x_2 , y_2,0.0), (quaternion.x, quaternion.y, quaternion.z, quaternion.w), t_now_ros, "base_link", "odom")

			odom = Odometry()
			odom.header.frame_id = "odom"
			odom.child_frame_id = "base_link"
			odom.header.stamp = t_now_ros
			odom.pose.pose.position.x = x_2
			odom.pose.pose.position.y = y_2 
			odom.pose.pose.position.z = 0
			odom.pose.pose.orientation = quaternion
			odom.twist.twist.linear.x = self.x_dot
			odom.twist.twist.linear.y = self.y_dot
			odom.twist.twist.angular.z = self.theta_dot
			self.odom_pub.publish(odom)
			self.t_last = t_now
		"""
		if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
			self.left_wheel = 0
			self.right_wheel = 0
				
	#change here to inverse the way by add -1
		self.drive(self.left_wheel, self.right_wheel) 
		
	
	def drive(self, left, right):
		self.plc.write_by_name(".SLAM_L[2]", left, pyads.PLCTYPE_DINT)
		self.plc.write_by_name(".SLAM_R[2]", right, pyads.PLCTYPE_DINT)
			
	def stop_drive(self):
		self.plc.write_by_name(".SLAM_L[2]", 0, pyads.PLCTYPE_INT)
		self.plc.write_by_name(".SLAM_R[2]", 0, pyads.PLCTYPE_INT)
	
	def send_odom(self):
		print("test")
	
	def shutdown(self):
		try:
			rospy.loginfo("Stopping robot...")
			# self.plc.write_by_name(".bSLAM_ServeON", 0, pyads.PLCTYPE_BOOL)
			self.plc.close()
			self.cmd_vel_pub.Publish(Twist())
			rospy.sleep(2)
		except:
			pass
		rospy.loginfo("Shutting down iAmech node...")



if __name__ == '__main__':
	new_iAmech = iAmechROS()


