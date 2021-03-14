import pygame, sys
import math

# Set fields for the display window.
DIMENSION = 500
WINDOW = pygame.display.set_mode((DIMENSION, DIMENSION))
pygame.display.set_caption("A* Path Finding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (128, 128, 128)
ORANGE = (255, 165, 0)
PURPLE = (128, 0, 128)
AQUA = (0, 128, 128)

# Create node class.
class Node(object):
	def __init__(self, row, col, node_dim, total_rows):
		self.row = row
		self.col = col
		self.node_dim = node_dim
		self.total_rows = total_rows
		self.x = col * node_dim
		self.y = row * node_dim
		self.color = WHITE
		self.neighbors = []

	# returns position of node (self.row, self.col) == (y, x)
	def get_node(self):
		return self.row, self.col

	# these methods returns node state.
	def checked(self):
		return self.color == RED

	def not_checked(self):
		return self.color == GREEN

	def barrier(self):
		return self.color == BLACK

	def start_node(self):
	    return self.color == AQUA

	def end_node(self):
		return self.color == ORANGE
    
    # reset node to original/blank state.
	def reset(self):
		self.color = WHITE

	# these methods changes the the node state.
	def change_to_closed(self):
		self.color = RED

	def change_to_open(self):
		self.color = GREEN

	def change_to_barrier(self):
		self.color = BLACK

	def change_to_start_node(self):
	    self.color = AQUA

	def change_to_end_node(self):
		self.color = ORANGE	

	def change_to_shortest_path(self):
		self.color = PURPLE

    # method to draw the node.
	def draw_node(self, window):
		pygame.draw.rect(window, self.color, (self.x, self.y, self.node_dim, self.node_dim))

    # method to keep track of neighboring nodes
	def track_neighbors(self, grid):

		# check North.
		if self.row > 0 and not grid[self.row - 1][self.col].barrier():
			self.neighbors.append(grid[self.row - 1][self.col])

		# check North-East.
		if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].barrier():
			self.neighbors.append(grid[self.row - 1][self.col + 1])

		# check East.
		if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].barrier():
			self.neighbors.append(grid[self.row][self.col + 1])

		# check South-East.
		if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].barrier():
			self.neighbors.append(grid[self.row + 1][self.col + 1])

		# check South.
		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].barrier():
			self.neighbors.append(grid[self.row + 1][self.col])

		# check South-West.
		if self.row < self.total_rows - 1 and  self.col > 0 and not grid[self.row + 1][self.col - 1].barrier():
			self.neighbors.append(grid[self.row + 1][self.col - 1])

		# check West.
		if self.col > 0 and not grid[self.row][self.col - 1].barrier():
			self.neighbors.append(grid[self.row][self.col - 1])

		# check North-West.
		if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].barrier():
			self.neighbors.append(grid[self.row - 1][self.col - 1])		

# define Heuristic Function: predicted disance to end node from current node.
def h(n1, n2):
	return abs(n2[0] - n1[0]) + abs(n2[1] - n1[1])

# define path finding algorithm
def path_finding_algorithm(draw, grid, start, end):

	# algorithm on:
	on = True
	
	# holds nodes that needs the neighbors to be visited. 
	open_nodes = {start: 0} # {node: f}

	# hold nodes that all the neighbors have been visited. 
	closed_nodes = {start: [0, 0, None]} # {node: [f, g, prior_node]}

	# start the algorithm
	while on and open_nodes:

		# update display on window.
		draw()

		# set event when need to exit.
		for event in pygame.event.get():
			# check to see if the game has been quit, then quit.
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()

		# current node that needs to be analyzed with the smallest f-value.
		keys_node = list(open_nodes.keys())
		values_f = list(open_nodes.values())

		f_value = min(values_f)
		node = keys_node[values_f.index(f_value)]

		open_nodes.pop(node)

		# change node color that has been taken out of the open_nodes.
		if node != start and node != end:
			node.change_to_closed()
		
		# end of algorithm condition.
		if node == end:
			on = False

			### return shortest path.
			# end loop when node gets to the start position.
			while node != start:

				# update display on window.
				draw()

				# mark shortest path node with purple.
				if node != start and node != end:
					node.change_to_shortest_path()

				node = closed_nodes[node][2]
		
			continue

		# record neigbors that can be visited.
		node.track_neighbors(grid)

		# analyze node and consider updating into closed_nodes and open_nodes.
		for current in node.neighbors:

			# find g and f.
			g = closed_nodes[node][1] + 1	# distance from starting node.
			f = g + h(current.get_node(), end.get_node()) # predicted total distance to end node from starting node.

			# case where node has not been visited yet.
			if current not in closed_nodes:
				closed_nodes[current] = [f, g, node]
				open_nodes[current] = f
				
				# this changes the node color that is put into open_nodes.
				if current != start and current != end:
					current.change_to_open()

			# Node has been already visited in the past, compare g values.
			else:
				# if g is smaller, update closed_nodes.
				if g < closed_nodes[current][1]:
					closed_nodes[current] = [f, g, node] 
					open_nodes[current] = f

	return False

# create grid with all the nodes.
def create_grid(rows, width):
	grid = []
	gap_space = width // rows

	for i in range(rows):
		grid.append([])

		for j in range(rows):
			node = Node(i, j, gap_space, rows)
			grid[i].append(node)

	return grid

# draw gridlines on the grid window.
def draw_gridlines(window, rows, width):
	gap_space = width // rows

	# horizontal lines.
	for i in range(rows):
		pygame.draw.line(window, GREY, (0, i * gap_space), (width, i * gap_space))

	# vertical line
	for j in range(rows):
		pygame.draw.line(window, GREY, (j * gap_space, 0), (j * gap_space, width))

# define main draw function to draw everything.
def draw(window, rows, width, grid):
	
	# set default window.
	window.fill(WHITE)

	# draw nodes.
	for each_row in grid:
		for each_node in each_row:
			each_node.draw_node(window)

    # draw the grid lines.
	draw_gridlines(window, rows, width)

    # update what has been drawn on to display.
	pygame.display.update()

# find clicked position.
def clicked_position(position, rows, width):

	gap_space = width // rows

	x, y = position 

	row = y // gap_space
	col = x // gap_space

	return row, col

# main function to put everything together.
def main(window, width):
	ROWS = 50
	grid = create_grid(ROWS, width)

    # set initial conditions
	start = end = None
	run = True
    
    # begin game.
	while run:

		# draw inital window.
		draw(window, ROWS, width, grid)

		# check each event.
		for event in pygame.event.get():

			# check to see if the game has been quit, then quit.
			if event.type == pygame.QUIT:
				run = False

			# set up nodes before running the path finding algorithm 
			if pygame.mouse.get_pressed()[0]:	# left click
				position = pygame.mouse.get_pos()
				row, col = clicked_position(position, ROWS, width)
				node = grid[row][col]

				# identify starting node.
				if not start and node != end:
					start = node
					start.change_to_start_node()

				# identify end node.
				elif not end and node != start:
					end = node
					end.change_to_end_node()

				# identify barriers.
				elif node != start and node != end:
					node.change_to_barrier()

			# reset the nodes to WHITE when right clicked.
			elif pygame.mouse.get_pressed()[2]:	# right click
				position = pygame.mouse.get_pos()
				row, col = clicked_position(position, ROWS, width)
				node = grid[row][col]
				node.reset()

				if node == start:
					start = None

				elif node == end:
					end = None

			# set pressing space bar to initiate the algorithm.
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_SPACE and start and end:
					path_finding_algorithm(lambda: draw(window, ROWS, width, grid), grid, start, end)

				if event.key == pygame.K_c:
					start = None
					end = None
					grid = create_grid(ROWS, width)

	pygame.quit()


main(WINDOW, DIMENSION)