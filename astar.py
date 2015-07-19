# Author Sven Koppany

# This implements the a-star search algorithm


import numpy as np
import scipy.io
import random

from matplotlib import pyplot as plt
from matplotlib import animation

class astar:
	def __init__(self, worldMap, matlabMap = False, startAndGoalCoords = ((0,0),(4,4)), randomCoords = False):
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

		self.path = self.getAStarPath()

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

	def getAStarPath(self):

		world = self.worldMap #world map
		start = self.startAndGoalCoords[0] #starting location
		goal = self.startAndGoalCoords[1] #goal location

		closed = [] #closed cells
		frontier = {} #open cells with g-values
		frontier_f = {} # dict of f-values
		cameFrom = {}

		moveCost = 1

		def heuristicManh(cell): #the heuristic using Manhattan movement
			return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1]) * moveCost

		def fValue(cell):
			return frontier[cell] + heuristicManh(cell)

		def expandManh(cell): #expand with Manhattan movement
			limitY, limitX = world.shape #get the map limits
			cY, cX = cell #get the node coords
			options = []

			if cY - 1 >= 0 and world[(cY - 1, cX)] == 0:
				options.append((cY - 1, cX))
			if cY + 1 < limitY and world[(cY + 1, cX)] == 0:
				options.append((cY + 1, cX))
			if cX - 1 >= 0 and world[(cY, cX - 1)] == 0:
				options.append((cY, cX - 1))
			if cX + 1 < limitX and world[(cY, cX + 1)] == 0:
				options.append((cY, cX + 1))

			return options

		def buildPath(cell):
			current = cell
			path = [cell]

			#Find the path from current(goal) back to start
			while current != start:
				current = cameFrom[current]
				path.append(current)

			return path

		#Put the starting cell into frontier, 
		#	g-value = 0, f-value = h + g
		frontier[start] = 0
		frontier_f[fValue(start)] = [start] #cal f-value and store it

		while bool(frontier): #while frontier is not empty

			#set current to the minimum f-value
			current = min(frontier_f.items(), key=lambda x: x[0])

			#save the lowest f-value and the current cell coords
			lowF = current[0]
			cell = current[1][0]

			#if the goal has been reached, find the shortest path
			if cell == goal:
				return buildPath(cell)

			#iterate through the neighbors
			neighbors = expandManh(cell)
			for i, v in enumerate(neighbors):

				#skip closed cells
				if v in closed:
					continue

				#next g-value
				gScore = frontier[cell] + moveCost

				#if the neighbor is not in the open frontier
				if v not in frontier.keys():

					#record its parent
					cameFrom[v] = cell

					#record the g-val and f-val
					frontier[v] = gScore
					fScore = fValue(v)
					if fScore not in frontier_f.keys():
						frontier_f[fScore] = [v]
					else:
						frontier_f[fScore].append(v)

			#move current cell to closed
			closed.append(cell)
			del frontier[cell]
			frontier_f[lowF].remove(cell)
			if frontier_f[lowF] == []:
				del frontier_f[lowF]

		return False

if __name__ == '__main__':

	#import map from MatLab 
	# sometimes this times out
	#astarPath = astar(worldMap = 'staticMap_254.mat', matlabMap = True, startAndGoalCoords = ((20, 20),(500, 500)))

	#simple example
	world = np.array([[0,0,0,0,1,0],
						[0,0,0,0,1,0],
						[0,0,0,0,1,0],
						[0,1,1,1,1,0],
						[0,0,0,1,0,0],
						[0,0,0,0,0,0]])
	astarPath = astar(worldMap = world, matlabMap = False, startAndGoalCoords = ((3,5),(1,2)))
	print astarPath.path


