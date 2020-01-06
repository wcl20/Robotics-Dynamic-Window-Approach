import numpy as np
import cv2

class Environment:

	def __init__(self, magnification):
		# Set magnification factor of the display
		self.magnification = magnification
		# Set width and height of the environment
		self.width = 1.0
		self.height = 1.0
		# Create image of the environment for display
		self.image = np.zeros([int(self.magnification * self.height), int(self.magnification * self.width), 3], dtype=np.uint8)
		# Define start position
		self.start = np.array([0.05, 0.05])
		# Define goal position
		self.goal = np.array([0.95, 0.95])
		# Create obstacles
		self._create_obstacles()

	def _distance(self, p1, p2):
		return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

	def _create_obstacles(self):
		self.obstacles = []
		number_of_obstacles = 60
		obstacle_radius = 0.03
		while len(self.obstacles) < number_of_obstacles:
			# Generate random position
			x = np.random.rand() * self.width
			y = np.random.rand() * self.height
			valid_position = True
			# Check obstacle does not collide with other obstacles
			if valid_position:
				for obstacle in self.obstacles:
					if self._distance((x, y), (obstacle.x, obstacle.y)) < 0.1:
						valid_position = False
						break
			# Check obstacle does not collide with start position
			if valid_position:
				if self._distance((x, y), self.start) < obstacle_radius + 0.02:
					valid_position = False
			# Check obstacle does not collide with goal
			if valid_position:
				if self._distance((x,y), self.goal) < obstacle_radius + 0.02:
					valid_position = False
			# Add obstacle if position is valid
			if valid_position:
				self.obstacles.append(Obstacle(x, y, obstacle_radius))

	def show(self, agent):
		# Create black background
		self.image.fill(0)
		# Draw agent
		x, y = agent.position
		center = (int(x * self.magnification), int((self.height - y) * self.magnification))
		radius = int(0.01 * self.magnification)
		cv2.circle(self.image, center, radius, (0, 0, 255), cv2.FILLED)
		# Draw arrow to show agent orientation
		arrow_length = 0.03
		arrow_x = int((x + arrow_length * np.cos(agent.theta)) * self.magnification)
		arrow_y = int((self.height - y - arrow_length * np.sin(agent.theta)) * self.magnification)
		cv2.arrowedLine(self.image, center, (arrow_x, arrow_y), (255, 255, 255))
		# Draw goal
		center = (int(self.goal[0] * self.magnification), int((self.height - self.goal[1]) * self.magnification))
		radius = int(agent.radius * self.magnification)
		cv2.circle(self.image, center, radius, (0, 255, 0), cv2.FILLED)
		# Draw obstacles
		for obstacle in self.obstacles:
			x = int(obstacle.x * self.magnification)
			y = int((self.height - obstacle.y) * self.magnification)
			radius = int(obstacle.radius * self.magnification)
			cv2.circle(self.image, (x, y), radius, (100, 100, 100), cv2.FILLED)
		# Show image
		cv2.imshow("Environment", self.image)
		# Give time for image to be rendered on screen
		cv2.waitKey(1)

class Obstacle:

	def __init__(self, x, y, radius):
		self.x = x
		self.y = y
		self.radius = radius






