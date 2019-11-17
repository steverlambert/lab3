import cv2
import math
import numpy
import Utils

class ObstacleManager(object):

	# Eliminating all obstacles with the power of GIT
	def __init__(self, mapMsg, car_width, car_length, collision_delta):
		self.map_info = mapMsg.info
		self.mapImageGS = numpy.array(mapMsg.data, dtype=numpy.uint8).reshape(
			(mapMsg.info.height, mapMsg.info.width, 1))

		# Retrieve the map dimensions
		height, width, channels = self.mapImageGS.shape
		self.mapHeight = height
		self.mapWidth = width
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

		x1 = int (numpy.ceil(config[0] + self.robotWidth/2))
		x2 = int (numpy.ceil(config[0] - self.robotWidth/2))
		y1 = int (numpy.ceil(config[1] + self.robotLength))
		y2 = int (config[1])

		corners = [[x1,y1], [x1,y2], [x2, y1], [x2,y2]]

		for point in corners:
			if self.mapImageBW[point[1]][point[0]] == 0:
				return False
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
		for i in range(len(list_x)):
			if not self.get_state_validity([list_x[i], list_y[i]]):
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
