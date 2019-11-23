import math
import numpy
from matplotlib import pyplot as plt
import cv2
import Utils
import time
import random


class HaltonPlanner(object):

    # planningEnv: Should be a HaltonEnvironment
    def __init__(self, planningEnv):
        self.planningEnv = planningEnv

    # Generate a plan
    # Assumes that the source and target were inserted just prior to calling this
    # Returns the generated plan
    def plan(self):
        self.sid = self.planningEnv.graph.number_of_nodes() - 2  # Get source id
        self.tid = self.planningEnv.graph.number_of_nodes() - 1  # Get target id

        self.closed = {}  # The closed list
        self.parent = {self.sid: None}  # A dictionary mapping children to their parents
        self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)}  # The open list
        self.gValues = {self.sid: 0}  # A mapping from node to shortest found path length to that node
        self.planIndices = []
        self.cost = 0

        # ------------------------------------------------------------
        # YOUR CODE HERE
        #
        # Implement A*
        # Functions that you will probably use
        # - self.get_solution()
        # - self.planningEnv.get_successors()
        # - self.planningEnv.get_distance()
        # - self.planningEnv.get_heuristic()
        # Note that each node in the graph has both an associated id and configuration
        # You should be searching over ids, not configurations. get_successors() will return
        #   the ids of nodes that can be reached. Once you have a path plan
        #   of node ids, get_solution() will compute the actual path in SE(2) based off of
        #   the node ids that you have found.
        # -------------------------------------------------------------

        # Continue searching while open set is not empty
        while self.open: #current_node != self.tid:

            # Set current_node to lowest cost in open set
            min_key = min(self.open.keys(), key = (lambda k: self.open[k]))
            current_node  = min_key

            # If current node is goal, we're done!
            if current_node == self.tid:
                break

            # Else remove the current node from open and continue
            del self.open[current_node]

            # Get the frontier, set of neighbors in current node
            # Circle the wagons... 
            frontier = self.planningEnv.get_successors(current_node)

            for neighbor in frontier:

                if neighbor not in self.closed:

                    # Check first for a valid path to neighbor
                    n_config = self.planningEnv.get_config(neighbor)
                    cur_n_config = self.planningEnv.get_config(current_node)

                    if self.planningEnv.manager.get_edge_validity(n_config, cur_n_config):

                        # Calculate distance from start to neighbor thorugh current
                        d = self.planningEnv.get_distance(current_node, neighbor) #start_to_neigh
                        g = self.gValues[current_node] + d

                        # If distance is less than existing gValue, record it!
                        if (neighbor not in self.gValues) or (g < self.gValues[neighbor]):

                            self.gValues[neighbor] = g
                            self.parent[neighbor] = current_node

                            h = self.planningEnv.get_heuristic(neighbor, self.tid)

                            f = numpy.float64(g + h) # Total cost f = g + h

                            self.open[neighbor] = f

            # Close current node
            self.closed[current_node] = f

        solution = self.get_solution(self.tid)

        return solution

    # Try to improve the current plan by repeatedly checking if there is a shorter path between random pairs of points in the path
    def post_process(self, plan, timeout):

        t1 = time.time()
        elapsed = 0
        while elapsed < timeout:  # Keep going until out of time
        #while elapsed ==0:
            # ---------------------------------------------------------
            # YOUR CODE HERE

            i = 0
            j = 0
            index_i = 0
            index_j = 0

            while (index_j == index_i):
                index_i = numpy.random.randint(0, len(plan))
                index_j = numpy.random.randint(0, len(plan))
                i = plan[index_i]
                j = plan[index_j]

            if index_i > index_j:
                temp = j
                j = i
                i = temp

                temp_index = index_j
                index_j = index_i
                index_i = temp_index


            if self.planningEnv.manager.get_edge_validity(i, j):

                plan = numpy.delete(plan, slice(index_i+1,index_j), 0)

                # Pick random id i
                # Pick random id j
                # Redraw if i == j
                # Switch i and j if i > j

                # if we can find path between i and j (Hint: look inside ObstacleManager.py for a suitable function)
                # Get the path
                # Reformat the plan such that the new path is inserted and the old section of the path is removed between i and j
                # Be sure to CAREFULLY inspect the data formats of both the original plan and the plan returned
                # to ensure that you edit the path correctly

                elapsed = time.time() - t1

        # Loop through plan and discretize edges
        tmp_idx = 0
        # Copy original
        plan2 = plan
        for idx in range(len(plan)-1):
            i = plan[idx]
            j = plan[idx+1]

            list_x, list_y, edge_length = self.planningEnv.manager.discretize_edge(i, j)

            betterplan = numpy.zeros((len(list_x),3))
            betterplan[:,0] = list_x
            betterplan[:,1] = list_y

            plan2 = numpy.insert(plan2, tmp_idx, betterplan, axis=0)

            tmp_idx += len(list_x)

        return plan2

    # Backtrack across parents in order to recover path
    # vid: The id of the last node in the graph
    def get_solution(self, vid):

        # Get all the node ids
        planID = []
        while vid is not None:
            planID.append(vid)
            vid = self.parent[vid]

        plan = []
        planID.reverse()
        for i in range(len(planID) - 1):
            startConfig = self.planningEnv.get_config(planID[i])
            goalConfig = self.planningEnv.get_config(planID[i + 1])
            px, py, clen = self.planningEnv.manager.discretize_edge(startConfig, goalConfig)
            plan.append([list(a) for a in zip(px, py)])
            self.planIndices.append(len(plan))
            self.cost += clen

        flatPlan = [item for sublist in plan for item in sublist]
        return flatPlan

    # Visualize the plan
    def simulate(self, plan):
        # Get the map
        envMap = 255 * (self.planningEnv.manager.mapImageBW + 1)  # Hacky way to get correct coloring
        envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)

        for i in range(numpy.shape(plan)[0] - 1):  # Draw lines between each configuration in the plan
            startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
            goalPixel = Utils.world_to_map(plan[i + 1], self.planningEnv.manager.map_info)
            cv2.line(envMap, (startPixel[0], startPixel[1]), (goalPixel[0], goalPixel[1]), (255, 0, 0), 5)

        # Generate window
        cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
        cv2.imshow('Simulation', envMap)

        # Terminate and exit elegantly
        cv2.waitKey(20000)
        cv2.destroyAllWindows()
