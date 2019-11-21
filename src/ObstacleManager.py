#!/usr/bin/env python
import cv2
import math
import numpy
import Utils
import rospy
from nav_msgs.srv import GetMap
import matplotlib.pyplot as plt


class ObstacleManager(object):

	# Eliminating all obstacles with the power of GIT
	def __init__(self, mapMsg, car_width, car_length, collision_delta):
		self.map_info = mapMsg.info
		self.mapImageGS = numpy.array(mapMsg.data, dtype=numpy.uint8).reshape(
			(mapMsg.info.height, mapMsg.info.width, 1))

		# Retrieve the map dimensions
		height, width, channels = self.mapImageGS.shape
		self.mapHeight = height #1236
		self.mapWidth = width #2792
		self.mapChannels = channels

		# Binarize the Image
		self.mapImageBW = 255 * numpy.ones_like(self.mapImageGS, dtype=numpy.uint8)
		self.mapImageBW[self.mapImageGS == 0] = 0

		# Obtain the car length and width in pixels
		self.robotWidth = int(car_width / self.map_info.resolution + 0.5)
		self.robotLength = int(car_length / self.map_info.resolution + 0.5)
		self.collision_delta = collision_delta

	# Check if the passed config is in collision
	# config: The configuration to check (in meters and radians)
	# Returns False if in collision, True if not in collision
	def get_state_validity(self, config):


		# Convert the configuration to map-coordinates -> mapConfig is in pixel-space
		mapConfig = Utils.world_to_map(config, self.map_info)

		x1 = int (numpy.ceil(mapConfig[0] + self.robotWidth/2))
		x2 = int (numpy.ceil(mapConfig[0] - self.robotWidth/2))
		y1 = int (numpy.ceil(mapConfig[1] + self.robotLength))
		y2 = int (mapConfig[1])

		#print mapConfig
		#print x1,x2,y1,y2

		#print self.mapImageBW[mapConfig[1]][mapConfig[0]][0]

		# if mapConfig[1] >= self.mapHeight or mapConfig[0] >= self.mapWidth:
		# 	print "Out of bounds"
		# 	return True
		# elif self.mapImageBW[mapConfig[1]][mapConfig[0]][0] == 0:
		# 	#print "Conflict"
		# 	return False
		# else:
		# 	return True

		corners = [[x1,y1], [x1,y2], [x2, y1], [x2,y2]]

		for point in corners:

			if point[1] >= self.mapHeight or point[0] >= self.mapWidth:
				#print self.mapWidth, self.mapHeight
				#print "Out of bounds!"
				return False
			elif self.mapImageBW[point[1]][point[0]][0] == 255:
				#print "Collision"
				return False
			else:
				#print "Continue!"
				continue

		return True

		# ---------------------------------------------------------
		# YOUR CODE HERE
		#
		# Return true or false based on whether the robot's configuration is in collision
		# Use a square to represent the robot, return true only when all points within
		# the square are collision free
		#
		# Also return false if the robot is out of bounds of the map
		#
		# Although our configuration includes rotation, assume that the
		# square representing the robot is always aligned with the coordinate axes of the
		# map for simplicity
		# ----------------------------------------------------------

	# Discretize the path into N configurations, where N = path_length / self.collision_delta
	#
	# input: an edge represented by the start and end configurations
	#
	# return three variables:
	# list_x - a list of x values of all intermediate points in the path
	# list_y - a list of y values of all intermediate points in the path
	# edgeLength - The euclidean distance between config1 and config2
	def discretize_edge(self, config1, config2):
		list_x, list_y = [], []
		edgeLength = 0
		# -----------------------------------------------------------
		# YOUR CODE HERE
		# -----------------------------------------------------------

		distancex = config1[0] - config2[0]
		distancey = config1[1] - config2[1]
		distance = numpy.sqrt(numpy.abs(distancex)**2 + numpy.abs(distancey)**2)
		N = int (distance / self.collision_delta)

		counter = 0
		for i in range(N):
			length = self.collision_delta*counter
			delta_x = (length/distance)*(distancex)
			delta_y = (length/distance)*(distancey)

			list_x.append(config2[0] + delta_x)
			list_y.append(config2[1] + delta_y)
			counter += 1

		edgeLength = distance

		return list_x, list_y, edgeLength


	# Check if there is an unobstructed edge between the passed configs
	# config1, config2: The configurations to check (in meters and radians)
	# Returns false if obstructed edge, True otherwise
	def get_edge_validity(self, config1, config2):

		list_x, list_y, edge_length = self.discretize_edge(config1, config2)
		#print len(list_x)
		for i in range(len(list_x)):
			test = self.get_state_validity([list_x[i], list_y[i]])
			if not test:
				#print "Obstructed"
				return False

		# -----------------------------------------------------------
		# YOUR CODE HERE
		#
		# Check if endpoints are obstructed, if either is, return false
		# Find path between two configs by connecting them with a straight line
		# Discretize the path with the discretized_edge function above
		# Check if all configurations along path are obstructed
		# -----------------------------------------------------------
		return True

# Write Your Test Code For Debugging
#if __name__ == '__main__':
#	return
	# Write test code here!

if __name__ == '__main__':

	print "Initializing node"
	rospy.init_node("obstacle_manager", anonymous=True)  # Initialize the node
	print "Initialized"

	map_service_name = "static_map"
	print "Getting map"

	rospy.wait_for_service(map_service_name)
	map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
	map_info = map_msg.info

	print map_info

	print "Got map"

	car_width = 0.25
	car_length = 0.33
	collision_delta = .4

	om = ObstacleManager(map_msg, car_width, car_length, collision_delta)

	# Give time to get setup
	rospy.sleep(1.0)

	lower = numpy.array([map_info.origin.position.x, map_info.origin.position.y])
	upper = numpy.array([map_info.origin.position.x + map_info.resolution * map_info.width,
						 map_info.origin.position.y + map_info.resolution * map_info.height])

	print "Map Info"
	print lower, upper

	lowermap = Utils.world_to_map(lower, map_info)
	uppermap = Utils.world_to_map(upper, map_info)

	print "World to Map"
	print lowermap, uppermap

	###### TEST FOR GET_STATE VALIDITY ######

	x = int (upper[0]) #55
	y = int (upper[1]) #24

	print x,y

	resultx = []
	resulty = []
	badx = []
	bady = []

	for i in range(0,x*100,10):
		for j in range(0,y*100,10):
			config = [i/100.0,j/100.0]
			mapConfig = Utils.world_to_map(config, om.map_info)
			#if om.get_state_validity([i/100.0,j/100.0]):
			if not om.mapImageBW[mapConfig[1]][mapConfig[0]][0] == 0:
				resultx.append(i)
				resulty.append(j)
			else:
				badx.append(i)
				bady.append(j)
				#print("i: ", i)


	rospy.sleep(1.0)

	#
	plt.xlabel('x')
	plt.ylabel('y')
	plt.scatter(resultx, resulty, c='w')

	print "Plot dimensions"
	print len(resultx), len(resulty)
	#plt.scatter(badx, bady, c='k')
	#plt.show()

	###### END OF TEST FOR GET_STATE_VALIDITY ######

	###### TEST FOR DISCRETIZE EDGE ######

	config1 = [[6,5], [20,18], [58,35]]
	config2 = [[20,20], [25,19], [72,35]]

	#plt.scatter(config1[0], config1[1], c='g')
	#plt.scatter(config2[0], config2[1], c='r')

	##### END OF TEST FOR DISCRETIZE EDGE #######

	## Something is funny with the world coordinates in this test. I'm getting "out of bounds"
	## for the line on the far right, which leads me to believe that the others are misaligned
	# somehow. That could explain why they are reporting collisions when we don't see anything


	for i in range(len(config1)):
	#for i in range(1):
		discx, discy, length = om.discretize_edge(config1[i], config2[i])
		print(len(discx))
		coordx = []
		coordy = []
		for j in range(len(discx)):
			coord = Utils.world_to_map([discx[j],discy[j]], map_info)
			coordx.append(coord[0])
			coordy.append(coord[1])

		print(len(coordx))

		#plt.scatter(coordx, coordy, c='g')

		if om.get_edge_validity(config1[i], config2[i]):
			plt.scatter(coordx, coordy, c='g')
		else:
			plt.scatter(coordx, coordy, c='r')

	plt.show()
