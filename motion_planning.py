#!/usr/bin/env python3

# EE 106 Lab 6

# ~ import numpy as np
# ~ import matplotlib.pyplot as plt

def neighbors(current):
	# define the list of 4 neighbors
	neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)] # List of allowed directions robot can move
	return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]


# Euclidean distance implementation
def heuristic_distance(candidate, goal):
	dsum = 0
	for i in range(len(candidate)):
		dsum += (goal[i] - candidate[i])**2
	
	# Not sure how necessary the int casting is. Have to run this through autograder
	return int(dsum**0.5)


def get_path_from_A_star(start, goal, obstacles):
	# input  start: integer 2-tuple of the current grid, e.g., (0, 0)
	#        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
	#        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
	# output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
	#   note that the path should contain the goal but not the start
	#   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)]
	
	# Label each node we visit with a unique number identifier for indexing parents
	# count = 0 # Update index every time we visit a new, unique node (check to see if it is in open_path or closed)
	parents = {start:None} # parents[] contains point tuples
    
	open_nodes = {start:0}
	closed_nodes = {}
    
	past_cost = {start:0}
	est_tot_cost = {start:0}
    
	while len(open_nodes) > 0:
		# Ensuring open_nodes is "sorted" based on Est. Total Cost
		min_key = (list(open_nodes.keys()))[0]
		for key in open_nodes:
			if open_nodes[key] < open_nodes[min_key]:
				min_key = key
		
		current = min_key
		est_tot_cost = open_nodes[current]
		
		open_nodes.pop(current)
		closed_nodes[current] = est_tot_cost
		
		if current == goal:
			path = []
			while parents[current] != None:
				path.append(current)
				current = parents[current]
				est_tot_cost = closed_nodes[current]
				
			# No need to pop() from path, loop terminates before adding start to path
			path.reverse()
			return path
			
		# Find neighbors
		tmp = neighbors(current)
		
		# Find out which neighbors are NOT in closed_nodes or obstacles
		neighbors_list = []
		for nbr in tmp:
			if nbr not in closed_nodes and nbr not in obstacles:
				neighbors_list.append(nbr)
		
		for nbr in neighbors_list:
			# We are using heuristic_distance also as cost fn for local neighbors
			tent_past_cost = past_cost[current] + heuristic_distance(current, nbr)
			
			if nbr not in past_cost or tent_past_cost < past_cost[nbr]:
				past_cost[nbr] = tent_past_cost
				parents[nbr] = current
				
				# See Lecture 12 slide 26 and slide 32
				open_nodes[nbr] = past_cost[current] + heuristic_distance(nbr, goal)
    
	return -1


# ~ if __name__ == '__main__':
	# ~ start = (0, 0) # this is a tuple data structure in Python initialized with 2 integers
	# ~ goal = (-5, -2)
	# ~ obstacles = [(-2, 1), (-2, 0), (-2, -1), (-2, -2), (-4, -2), (-4, -3)]
	# ~ path = get_path_from_A_star(start, goal, obstacles)
	
	# ~ x = []
	# ~ y = []
	# ~ for point in path:
		# ~ x.append(point[0])
		# ~ y.append(point[1])
		
	# ~ for obstacle in obstacles:
		# ~ x.append(obstacle[0])
		# ~ y.append(obstacle[1])
	
	# ~ x = np.array(x)
	# ~ y = np.array(y)
	
	# ~ plt.scatter(x, y)
	# ~ plt.title("Path")
	# ~ plt.title("Obstacles")
	# ~ plt.title("Path and Obstacles")
	# ~ plt.title("Path without Obstacles")
	# ~ plt.xlabel("x")
	# ~ plt.ylabel("y")
	# ~ plt.show()
	
	# ~ print(path)
