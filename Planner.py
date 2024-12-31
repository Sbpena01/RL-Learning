from queue import PriorityQueue

from OccupancyGrid import *
from Car import Car
import Utils

MAX_V = 5.0
MAX_PHI = np.deg2rad(30.0)

class Planner():
    def __init__(self, grid: OccupancyGrid, car: Car, goal: State, dt=0.167, max_steps = 1000):
        # Goal should be a certain position (and later also theta)
        self.grid = grid
        self.car = car
        self.start = State(car.position, car.theta, car.v, car.phi)
        self.goal = goal
        self.dt = dt  # 10 frames at 60fps
        self.max_steps = max_steps
    
    def actions(self):
        velocities = np.linspace(-MAX_V, MAX_V, 4)
        steering_angles = np.linspace(-MAX_PHI, MAX_PHI, 7)
        actions = []
        for v in velocities:
            for phi in steering_angles:
                actions.append((v,phi))
        return actions
    
    def results(self, state: State, action: tuple):
        theta_dot = (action[0]/self.car.l)*np.tan(action[1])
        d_theta = theta_dot * self.dt
        new_theta = state.theta + d_theta
        x_dot = action[0] * np.cos(new_theta)
        y_dot = action[0] * np.sin(new_theta)
        d_position = (x_dot * self.dt, y_dot * self.dt)
        resulting_state = State((state.position[0] + d_position[0], state.position[1] + d_position[1]),
                                state.theta+d_theta, action[0], action[1], parent=state)
        return resulting_state

    def calculateCost(self, state: State):
        distance_cost = Utils.euclideanDistance(state.position, self.goal.position)
        reverse_cost = 50 if state.v < 0 else 0
        return distance_cost + reverse_cost

    def checkGoal(self, state: State):
        state_cell = self.grid.worldToGrid(state.position)
        x_error = abs(self.goal.position[0] - state_cell[0])
        y_error = abs(self.goal.position[1] - state_cell[1])
        return x_error <= 1 and y_error <= 1

    def getPath(self, state: State):
        path: list[State] = []
        current = state
        while current is not None:
            path.append(current)
            current = current.parent
        return path

    def plan(self, source: pygame.Surface=None):
        counter = 0
        frontier = PriorityQueue()
        visited_cells: list[Cell] = []
        frontier.put(self.start, self.start.cost)
        while not frontier.empty() and counter <= self.max_steps:
            popped_state: State = frontier.get()
            popped_cell_coord = self.grid.worldToGrid(popped_state.position)
            popped_cell: Cell = self.grid.cells.get(popped_cell_coord)
            if popped_cell is None:
                continue
            if self.checkGoal(popped_state):
                print("Found Goal!")
                return self.getPath(popped_state)
            if popped_cell in visited_cells:
                continue
            popped_cell.setState(popped_state)
            visited_cells.append(popped_cell)
            counter += 1
            actions = self.actions()
            for action in actions:
                result = self.results(popped_state, action)
                # Check if the result is in bounds and is not in an occupied cell
                result_grid_coord = self.grid.worldToGrid(result.position)
                resulting_cell:Cell = self.grid.cells.get(result_grid_coord)
                if resulting_cell is None:
                    continue
                if resulting_cell.value != 0:
                    continue

                cost = self.calculateCost(popped_state)
                result.setCost(cost)
                frontier.put(result, cost)
        print("Cannot find goal. Returning...")
        return None

    