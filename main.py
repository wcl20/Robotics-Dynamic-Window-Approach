from environment import Environment
from agent import Agent

def main():
	# Create random environment
	environment = Environment(magnification=500)
	# Create agent
	agent = Agent(environment)
	# Main loop
	while not agent.has_reached_goal():
		agent.step()
		environment.show(agent)

if __name__ == '__main__':
	main()