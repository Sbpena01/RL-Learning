from queue import PriorityQueue

from OccupancyGrid import *
from Car import Car
import Utils

MAX_V = 5.0
MAX_PHI = np.deg2rad(30.0)

class Node():
    def __init__(self, state: State, parent: 'Node' = None, cost: float = 0.0):
        self.state = state
        self.parent = parent
        self.cost = cost

    def __lt__(self, other: 'Node'):
        return self.cost < other.cost
    
    def getGridCell(self, grid: OccupancyGrid):
        return grid.worldToGrid(self.state.position)
    
    def draw(self, source:pygame.Surface, color=(0,0,255)):
        pygame.draw.circle(source, color, (self.state.position[0]*50, self.state.position[1]*50), 2.0)

class MotionPlanner():
    def __init__(self, start: State, goal: State, car: Car, grid: OccupancyGrid):
        self.start = start
        self.goal = goal
        self.car = car
        self.grid = grid
        self.dt = 0.2
        self.max_iters = 1000
        self.nodes: list[Node] = []

    def plan(self, source: pygame.Surface = None):
        # Create first node from starting state (self.state)
        self.nodes: list[Node] = []
        starting_node = Node(self.start)

        # Add starting node to priority queue to begin the search process
        queue = PriorityQueue()
        queue.put(starting_node)
        visited_gridcells: list[tuple] = []
        counter = 0

        # Continue running until either the queue is empty or the max loops have been reached.
        while not queue.empty() and counter <= self.max_iters:
            # Pop node from queue
            current_node:Node = queue.get()

            # Check if node's state is the goal state
            if self.checkGoal(current_node):
                self.resetCar()
                return self.getPath(current_node)
            
            # Check if the node has been visited already
            current_gridcell = current_node.getGridCell(self.grid) 
            if current_gridcell in visited_gridcells:
                continue
            visited_gridcells.append(current_gridcell)
            counter += 1
            

            # Expand the node
            new_nodes = self.expandNode(current_node)
            
            # Add all new nodes to frontier
            for node in new_nodes:
                self.nodes.append(node)
                queue.put(node)
            
        self.resetCar()
        return None

    def checkGoal(self, node: Node):
        goal_position = self.goal.position
        checked_position = node.state.position
        return abs(goal_position[0]-checked_position[0]) <= 0.2 and abs(goal_position[1]-checked_position[1]) <= 0.2

    def getPath(self, node: Node):
        path: list[Node] = []
        current = node
        while current is not None:
            path.append(current)
            current = current.parent
        return path

    def expandNode(self, node: Node):
        actions = self.actions()
        new_nodes: list[Node] = []
        for action in actions:
            state = self.iterateThrough(node.state, action, self.dt)
            if state is None:
                continue
            cost = self.calculateCost(state)
            new_node = Node(state, node, cost=cost)
            new_nodes.append(new_node)
        return new_nodes

    def actions(self):
        velocities = [-MAX_V, MAX_V]
        angles = [-MAX_PHI, 0, MAX_PHI]
        actions = []
        for v in velocities:
            for phi in angles:
                actions.append((v, phi))
        return actions
    
    def iterateThrough(self, start: State, action: tuple, dt: float, num_steps: int = 5):
        self.car.setPosition(start.position)
        self.car.setTheta(start.theta)
        self.car.setVelocity(action[0])
        self.car.setSteeringAngle(action[1])
        increment = dt / num_steps
        for _ in range(num_steps):
            self.car.step(increment)
            if self.grid.environment.checkCarCollision(self.car):
                print("Collision Detected")
                return None
        return State(self.car.position, self.car.theta, self.car.v, self.car.phi)
    
    def resetCar(self):
        self.car.setPosition(self.start.position)
        self.car.setTheta(self.start.theta)
        self.car.setVelocity(self.start.v)
        self.car.setSteeringAngle(self.start.phi)

    def calculateCost(self, state: State):
        distance_cost = Utils.euclideanDistance(state.position, self.goal.position)
        # reverse_cost = 50 if state.v < 0 else 0
        return distance_cost
    
    def draw(self, source: pygame.Surface):
        for node in self.nodes:
            node.draw(source)
        starting_node = Node(self.start)
        goal_node = Node(self.goal)
        starting_node.draw(source, color=(0,100,200))
        goal_node.draw(source, color=(0,200,0))
    
    def drawPath(self, path: list[Node], source: pygame.Surface):
        for node in path:
            if node.parent is None:
                continue
            pygame.draw.line(source, (255,0,0), (node.state.position[0]*50, node.state.position[1]*50),
                             (node.parent.state.position[0]*50, node.parent.state.position[1]*50))