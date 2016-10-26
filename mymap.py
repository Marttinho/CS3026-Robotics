#!/usr/bin/python

import rospy
import roslib
import tf
import markers
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid

class occupancy_grid:
	def __init__(self):
		rospy.init_node('occupancy_grid')
		rospy.loginfo('starting class')
		#rospy.subscriber('/map_server',)
		self.publisher=rospy.Publisher('/my_map',OccupancyGrid,queue_size=100)
		self.grid()

	def grid(self):
		print 'in grid function'
		rospy.loginfo('waiting for service')
		rospy.wait_for_service('/static_map')
		try:
			grid_composition=rospy.ServiceProxy('/static_map',GetMap)
			#self.publisher.publish(grid().map)
			response=self.grid_creation(grid_composition)
		except rospy.ServiceException, e:
			print "Service failed"

	def grid_creation(self, grid_composition):
		print "in grid creation function"
		#print grid_composition().map
		while not rospy.is_shutdown():
			#print "publishing"
			self.publisher.publish(grid_composition().map)
			#print "publishing"
			
			



if __name__ == '__main__':
	try:
		my_map = occupancy_grid()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass