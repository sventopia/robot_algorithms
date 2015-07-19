import numpy as np
import scipy.io
import random
import math
import Queue

#import cv2

from matplotlib import pyplot as plt
from matplotlib import animation

class astar:
	def __init__(self, worldMap, matlabMap = True, startAndGoalCoords = ((0,0),(4,4)), randomCoords = False):
		#Convert the matlab map to a numpy array
		if matlabMap == True:
			self.worldMap = np.asanyarray(scipy.io.loadmat(worldMap)['grid'])

			#change the walls to -1 so that the first path is recordable
			self.worldMap[self.worldMap == 1] = 255 #for better imaging

		#But if it is an numpy ndarray
		elif type(worldMap) is np.ndarray:
			self.worldMap = worldMap

		#Generate random start and target coordinates in the map
		if randomCoords == True:
			self.startAndGoalCoords = self.getRandomLocations()
		else:
			self.startAndGoalCoords = startAndGoalCoords

		self.__getAStarPath()

	def getRandomLocations(self):
		#
		def findEmptyLoc():
			while True:
				loc = (random.randint(0,self.worldMap.shape[0]-1), 
									random.randint(0,self.worldMap.shape[1]-1))
				if self.worldMap[loc] == 0:
					break
			return loc

		while True:
			startLoc = findEmptyLoc()
			goalLoc = findEmptyLoc()
			if startLoc != goalLoc:
				break

		return (startLoc, goalLoc)

	def __getAStarPath(self):

		self.success = True #set to true if no path found

		st = self.startAndGoalCoords[0]
		gl = self.startAndGoalCoords[1]

		self.path = [st]
		path = self.path

		m = self.worldMap

		#build heuristic from target
		h = np.zeros(m.shape)
		for y, r in enumerate(h):
			for x, v in enumerate(r): 
				h[y,x] = math.fabs(y - gl[0]) + math.fabs(x - gl[1])

		def expand():
			#get the current a last steps from path
			y, x = path[-1]
			yLimit = m.shape[0]
			xLimit = m.shape[1]

			options = []

			if len(path) > 1:
				last = path[-2]
			else:
				last = path[-1]

			if y+1 < yLimit and m[y+1,x] == 0:# and (y+1,x) != last:
				options.append((y+1,x))
			if x+1 < xLimit and m[y,x+1] == 0:# and (y,x+1) != last:
				options.append((y,x+1))
			if y-1 >= 0 and m[y-1,x] == 0:# and (y-1,x) != last:
				options.append((y-1,x))
			if x-1 >= 0 and m[y,x-1] == 0:# and (y,x-1) != last:
				options.append((y,x-1))

			return options

		#build a path until the goal is reached or failure
		while True:
			opts = expand()

			#the g-value is the path index
			g = len(path) - 1

			if opts == []:
				self.success = False
				break

			#find the f-values of all expansions
			f = np.array([])
			for i, v in enumerate(opts):
				f = np.append(f, g + h[v])

			#find the lowest and add it to the path
			nextStep = opts[np.argmin(f)]
			path.append(nextStep)

			#if goal reached?
			if gl == nextStep:
				break

		return

	def astar(start, goal):
		frontier = Queue.Queue()
		frontier.put(start)
		visited = {}
		visited[start] = True

		while not frontier.empty():
		   current = frontier.get()
		   for next in graph.neighbors(current):
		      if next not in visited:
		         frontier.put(next)
		         visited[next] = True

		# closedset := the empty set    // The set of nodes already evaluated.
	 #    openset := {start}    // The set of tentative nodes to be evaluated, initially containing the start node
	 #    came_from := the empty map    // The map of navigated nodes.
	 
	 #    g_score := map with default value of Infinity
	 #    g_score[start] := 0    // Cost from start along best known path.
	 #    // Estimated total cost from start to goal through y.
	 #    f_score = map with default value of Infinity
	 #    f_score[start] := g_score[start] + heuristic_cost_estimate(start, goal)
	     
	 #    while openset is not empty
	 #        current := the node in openset having the lowest f_score[] value
	 #        if current = goal
	 #            return reconstruct_path(came_from, goal)
	         
	 #        remove current from openset
	 #        add current to closedset
	 #        for each neighbor in neighbor_nodes(current)
	 #            if neighbor in closedset
	 #                continue
	 #            tentative_g_score := g_score[current] + dist_between(current,neighbor)
	 
	 #            if neighbor not in openset or tentative_g_score < g_score[neighbor] 
	 #                came_from[neighbor] := current
	 #                g_score[neighbor] := tentative_g_score
	 #                f_score[neighbor] := g_score[neighbor] + heuristic_cost_estimate(neighbor, goal)
	 #                if neighbor not in openset
	 #                    add neighbor to openset
	 
	 #    return failure

if __name__ == '__main__':

	#import map from MatLab and get limits
	astarPath = astar(worldMap = 'staticMap_254.mat', matlabMap = True, startAndGoalCoords = ((20, 20),(975, 263)))
	print astarPath.success
	print astarPath.path

	#ceate an image of the path
	#for i, v in enumerate(astartPath.path):
	#	astarPath.worldMap[v] = 128

	#cv2.imwrite('path.jpg', astarPath.worldMap)






