choice, no_choice = 0,0

from collections import deque
navigation_path = deque()

#data structure for visited cell record
cell_coverage = []


current_cell_Row, current_cell_Col = 0,0
current_orientation = 0
not_visited_cell_count = 144 

g = Graph() # for the graph



############################################################################################################
'''
Webots controller code
'''

from controller import Robot, Motor
import math

TIME_STEP = 32

MAX_SPEED = 6.28


# robot instance
robot = Robot()

# motor control handlers initializers
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftWheelSensor = robot.getDevice("left wheel sensor")
rightWheelSensor = robot.getDevice("right wheel sensor")
leftWheelSensor.enable(TIME_STEP)
rightWheelSensor.enable(TIME_STEP)


leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))



def main_loop():

    if len(navigation_path) < 1:

        g.set_value((current_cell_Row, current_cell_Col), 1)
        not_visited_cell_count -= 1

        if not_visited_cell_count == 0:
            pass       # just not to get a syntax error, no purpose
            # must end the loop


        wall_sensor_monitoring()
        find_nearest_frontier()

    navigate_next_step()




# The real loop that runs on the controller
while robot.step(TIME_STEP) != -1:
    main_loop()



###########################################################################################################
'''
functions related to sensor reading and interpretation
'''

def wall_sensor_monitoring():

    # assume returns false if blocked
    left = left_sensor_read()
    right = right_sensor_read()
    forward = forward_sensor_read()


    # cell_coverage[current_cell_X][current_cell_Y] = True

    update_graph(left, right, forward)


    if (left or right or forward) == False:
        do_a_180()
    

def update_graph(left, right, forward):

    '''
    Completed this function

    considering the current_orientation, should build vertexes
    '''

    if current_orientation == 0:
        if forward : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row + 1, current_cell_Col))

        if left : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row, current_cell_Col + 1))

        if right : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row, current_cell_Col - 1))

    
    elif current_orientation == 1:
        if forward : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row, current_cell_Col - 1))

        if left : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row + 1, current_cell_Col))

        if right : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row - 1, current_cell_Col))

    
    elif current_orientation == 2:
        if forward : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row - 1, current_cell_Col))

        if left : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row, current_cell_Col - 1))

        if right : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row, current_cell_Col + 1))


    elif current_orientation == 3:
        if forward : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row, current_cell_Col + 1))

        if left : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row - 1, current_cell_Col))

        if right : g.add_edge((current_cell_Row, current_cell_Col), (current_cell_Row + 1, current_cell_Col))



###########################################################################################################
'''
functions related to finding the next best point to travel
'''



def find_nearest_frontier():

    '''
    Do a breadth first search and find the shortest length to a 0 value vertex

    add the consequent vertexes upto that in the path into navigation_path[]
    '''

    navigation_path = g.bfs_until_zero_deque((current_cell_Row,current_cell_Col))
    navigation_path.popleft()


###########################################################################################################
'''
functions related to motor controling and navigation
'''


def navigate_next_step():

    '''
    Take the first cell in the navigation_path[] list
    Update the current cell values (current_cell_Row, current_cell_Col) as that
    give instruction on what to do(motor controls - turn left, right) for getting from current cell
      to that considering orientation
    Update current_orientation
    '''

    next_cell = navigation_path.popleft()
    next_cell_Row = next_cell[0]
    next_cell_Col = next_cell[1]

    relative_Row = next_cell_Row - current_cell_Row
    relative_Col = next_cell_Col - current_cell_Col

    next_cell_placement = 0

    if relative_Row == -1:
        next_cell_placement = 2
    elif relative_Col == 1:
        next_cell_placement = 3
    elif relative_Col == -1:
        next_cell_placement = 1

    required_rotation_value = (next_cell_placement - current_orientation) % 4

    if required_rotation_value == 1:
        turn_right()
    elif required_rotation_value == 2:
        do_a_180()
    elif required_rotation_value == 3:
        turn_left()

    goForward()

    current_cell_Row = next_cell_Row
    current_cell_Col = next_cell_Col




def goForward():
    '''
    Related motor control code to go forward
    '''
    # based on wheel radius (S = r * theta)
    goForwardValue = 0.25/0.0205
    motor_control(goForwardValue, 1, 1)
    print("square changed")

def do_a_180():
    '''
    code to do a 180 turn
    '''
    delta_change = (0.013*math.pi)/0.0205 + 0.07
    motor_control(delta_change, 1, -1)
    motor_control(delta_change, 1, -1)

    current_orientation = (current_orientation + 2) % 4

def turn_left():

    # based on axel distance, wheel radius
    delta_change = (0.013*math.pi)/0.0205 + 0.07
    motor_control(delta_change, -1, 1)
    print("turned left")

    current_orientation = (current_orientation - 1) % 4


def turn_right():

    delta_change = (0.013*math.pi)/0.0205 + 0.07
    motor_control(delta_change, 1, -1)
    print("turned right")

    current_orientation = (current_orientation + 1) % 4




def motor_control(angle, Lspeed, Rspeed):

    if angle < 5  :
        Lspeed *= 0.4275
        Rspeed *= 0.4275

    leftMotor.setVelocity(Lspeed * MAX_SPEED)
    rightMotor.setVelocity(Rspeed * MAX_SPEED)
   
    
    presentPosL = leftWheelSensor.getValue()
    presentPosR = rightWheelSensor.getValue()
    
    while robot.step(TIME_STEP) != -1:
        eL = angle - abs(presentPosL - leftWheelSensor.getValue())
        eR = angle - abs(presentPosR - rightWheelSensor.getValue())
        if eL < 0.001 and eR < 0.001:
            break

   

###########################################################################################################
'''
The graph and related functions 
'''

class Graph:
    def __init__(self, rows=12, cols=12):
        self.rows = rows
        self.cols = cols

        # Initialize vertices as grid[row][col]
        self.vertices = {
            (r, c): 0   # default value = 0
            for r in range(rows)
            for c in range(cols)
        }

        # adjacency list
        self.edges = {v: [] for v in self.vertices}

    # ----------------------------
    # vertex operations
    # ----------------------------

    def set_value(self, cell, value):
        """ cell = (row, col) """
        if cell in self.vertices:
            self.vertices[cell] = value

    def get_value(self, cell):
        return self.vertices.get(cell, None)

    # ----------------------------
    # edge operations
    # ----------------------------

    def add_edge(self, v1, v2):
        """ Add an undirected edge """
        if v1 in self.edges and v2 in self.edges:
            if v2 not in self.edges[v1]:
                self.edges[v1].append(v2)
            if v1 not in self.edges[v2]:
                self.edges[v2].append(v1)

    # utility
    def neighbors(self, cell):
        return self.edges.get(cell, [])
    


    def bfs_until_zero_deque(self, start):

        queue = deque([(start, deque([start]))])
        visited = set([start])

        while queue:
            current, path = queue.popleft()

            # check value
            if self.get_value(current) == 0:
                return path     # <-- now a deque is returned

            # explore neighbors
            for nxt in self.neighbors(current):
                if nxt not in visited:
                    visited.add(nxt)

                    # copy path and append new node
                    new_path = deque(path)
                    new_path.append(nxt)

                    queue.append((nxt, new_path))

        return None



# ----------------------------
# Example usage
# ----------------------------
# g = Graph()

# g.set_value((0, 0), 1)
# g.set_value((5, 5), 1)

# g.add_edge((0, 0), (0, 1))
# g.add_edge((1, 1), (2, 3))

# print("Value at (5,5) =", g.get_value((5, 5)))
# print("Neighbors of (0,0):", g.neighbors((0, 0)))

# path = g.bfs_until_zero((2,2))


      