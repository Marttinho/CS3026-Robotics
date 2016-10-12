#!/usr/bin/python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

LEFT=1
RIGHT=0

class Wallfollower:
	#marker=Marker()
	def __init__(self):
		rospy.init_node('wallfollower')
		self.safeDistance=2
		self.following=LEFT
		#followingwallonLEFTorRIGHT
		self.listener=tf.TransformListener()
		self.publisher=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		self.subscriber=rospy.Subscriber('/noisy_base_scan',LaserScan,self.scanReceived)
		#self.m=Marker.Markers()
#usedfordrawinginrviz
	'''def add(self,x,y,r,g,b,frame):
		mr=Marker()
		mr.header.frame_id=frame
		mr.ns="basic"
		mr.id=self.i
		mr.type=mr.SPHERE
		mr.action=mr.ADD
		mr.pose.position.x=x
		mr.pose.position.y=y
		mr.pose.orientation.w=1
		mr.scale.x=1.5
		mr.scale.y=1.5
		mr.scale.z=1.5
		mr.color.r=r
		mr.color.g=g
		mr.color.b=b
		mr.color.a=1.0'''
	def scanReceived(self,LaserScan):
		a=LaserScan.angle_min
		i=LaserScan.angle_increment
#initialiseanglesforscanningtoleftorrightoftherobot
#dependingonwallfollowingdirection.
		if (self.following==RIGHT):
			start=-999
			end=0
		else:
			start=0
			end=999
#setrangestoarbitrarynumbersinitially
		rmin=9999
		amin=999
		for r in LaserScan.ranges:#findclosestpoint
			if(a<=end and a>=start and r<rmin):
				amin=a
				rmin=r
			a+=i
#computepointinlaserframeofreference
#andtransformtorobotframe
		x=rmin*math.cos(amin)
		y=rmin*math.sin(amin)
		ps=PointStamped(header=Header(stamp=rospy.Time(0),frame_id="/base_laser_link"),point=Point(x,y,0))
		p=self.listener.transformPoint("/base_link",ps)
		x=p.point.x
		y=p.point.y
		mag=math.sqrt(x*x+y*y)
#usefulforlater,magnitudeofvector
#tangentvector
		tx=0
		ty=0
		if(self.following==RIGHT):
			tx=-y
			ty=x
		else:
			tx=y
			ty=-x
		tx/=mag
		ty/=mag
		#where we should be going
		dx=x+tx-self.safeDistance*x/mag
		dy=y+ty-self.safeDistance*y/mag
		#self.m.add(self,dx,dy,0,0,1.0,"base_link")

		theta=0

		if (dx>0 and rmin<LaserScan.range_max):
			theta=math.atan(dy/dx)
		elif (self.following==RIGHT):
			theta=-1
		else:
			theta=1

		twist=Twist()
		if (dx>0):
			twist.linear.x=1
		twist.angular.z=theta
		self.publisher.publish(twist)
		#self.m.add(0,0,1.0,0,0,"base_link")
		#self.m.draw()

robot=Wallfollower()
rospy.spin()


