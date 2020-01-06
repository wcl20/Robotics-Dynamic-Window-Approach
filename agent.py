import numpy as np
import sys

class Agent:

	def __init__(self, environment):
		self.environment = environment
		self.position = environment.start 	# Robot position

		# Robot settings
		self.radius = 0.01					# Robot radius
		self.theta = 0.0					# Robot orientation
		self.vmax = 0.02					# Maximum velocity
		self.vl = 0.0						# Left velocity
		self.vr = 0.0						# Right velocity
		self.acc = 0.05						# Accerlation
		
		# Dynamic Window Approach 
		self.benefit = 12					# Benefit weighting
		self.cost = 16						# Cost weighting
		self.safe_distance = 0.01			# Safe distance from obstacle
		self.tau = 2.0						# Look ahead time
		self.dt = 0.1						# Time step

	def _predict_position(self, vl, vr, position, theta, time):
		# original position
		x = position[0]
		y = position[1]
		# Case 1: Travels in straight line
		if abs(vl - vr) < 1e-10:
			new_theta = theta
			new_x = x + vl * time * np.cos(theta)
			new_y = y + vl * time * np.sin(theta)
		# Case 2: Rotates on the spot
		elif abs(vl + vr) < 1e-10:
			width = self.radius * 2
			new_theta = theta + (vr - vl) * time / width
			new_x = x 
			new_y = y
		# Case 3: Travels in circular path
		else:
			width = self.radius * 2
			path_radius = width * (vr + vl) / (2 * (vr - vl))
			new_theta = theta + (vr - vl) * time / width
			new_x = x + path_radius * (np.sin(new_theta) - np.sin(theta))
			new_y = y - path_radius * (np.cos(new_theta) - np.cos(theta))
		# Return new position and orientation
		return np.array([new_x, new_y]), new_theta

	def _distance(self, p1, p2):
		return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

	def has_reached_goal(self):
		return self._distance(self.position, self.environment.goal) < 0.01

	def step(self):
		# Possible velocities
		vls = [self.vl - self.acc * self.dt, self.vl, self.vl + self.acc * self.dt]
		vrs = [self.vr - self.acc * self.dt, self.vr, self.vr + self.acc * self.dt]
		# Best velocities
		best_benefit = -sys.maxsize
		best_vl = 0.0
		best_vr = 0.0
		for vl in vls:
			for vr in vrs:
				# Check velocity does not exceed maximum
				if abs(vl) <= self.vmax and abs(vr) <= self.vmax:
					# Predict new position after look ahead time
					position, theta = self._predict_position(vl, vr, self.position, self.theta, self.tau)
					# Calculate distance benefit (distance to goal)
					original_distance = self._distance(self.position, self.environment.goal)
					new_distance = self._distance(position, self.environment.goal)
					distance_benefit = self.benefit * (original_distance - new_distance)
					# Calculate obstacle cost (distance from closest obstacle)
					closest_obstacle = sys.maxsize
					for obstacle in self.environment.obstacles:
						distance = self._distance(position, (obstacle.x, obstacle.y))
						distance = distance - self.radius - obstacle.radius
						closest_obstacle = min(closest_obstacle, distance)
					obstacle_cost = self.cost * (self.safe_distance - closest_obstacle)
					obstacle_cost = max(obstacle_cost, 0)
					# Calculate total benefit
					benefit = distance_benefit - obstacle_cost
					if benefit > best_benefit:
						best_benefit = benefit
						best_vl = vl 
						best_vr = vr
		# Update position using best velocities in one time step 
		self.vl = best_vl
		self.vr = best_vr
		self.position, self.theta = self._predict_position(self.vl, self.vr, self.position, self.theta, self.dt)






