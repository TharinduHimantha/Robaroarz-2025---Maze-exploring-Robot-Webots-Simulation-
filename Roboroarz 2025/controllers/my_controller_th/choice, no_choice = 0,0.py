choice, no_choice = 0,0

from collections import deque

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


      

##########################################################################################

navigation_path = deque()

#data structure for visited cell record
cell_coverage = []


current_cell_Row, current_cell_Col = 0,0
current_orientation = 0
not_visited_cell_count = 144

final_dest_Row, final_dest_Col = 0,0 
all_complete = 0 

g = Graph() # for the graph



############################################################################################################
'''
Webots controller code
'''

from controller import Robot, Motor
import math
import cv2
import numpy as np

# --- CONSTANTS ---
TIME_STEP = 32
MAX_SPEED = 6.1

# --- PID GAINS  ---
Kp = 12.0
Kd = 20000.0
Ki = 0.0

# --- PID STATE VARIABLES ---
prev_error = 0.0
integral = 0.0


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

rightIR = robot.getDevice("ps2")
leftIR = robot.getDevice("ps5")
forward_IR1 = robot.getDevice("ps7")
forward_IR2 = robot.getDevice("ps0")

rightIR.enable(TIME_STEP)
leftIR.enable(TIME_STEP)
forward_IR1.enable(TIME_STEP)
forward_IR2.enable(TIME_STEP)

ir_array = [robot.getDevice("ps6"),
                robot.getDevice("ps1"),
                robot.getDevice("ps4"),
                robot.getDevice("ps3")]
                
for sensor in ir_array:
    sensor.enable(TIME_STEP)

# Lookup Tables
D_lookup = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25]
V_lookup = [4095, 3500, 2500, 1700, 1100, 700]


# Camera
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)


width = camera.getWidth()
height = camera.getHeight()

# Percentage of pixels needed to confirm detection
DETECTION_RATIO = 0.6   # 60%

leds = []
for i in range(8):
    leds.append(robot.getDevice(f"led{i}"))


def main_loop():

    global all_complete
    if all_complete:
        return

    flash_led(0)

    if len(navigation_path) < 1:

        g.set_value((current_cell_Row, current_cell_Col), 1)
        global not_visited_cell_count
        not_visited_cell_count -= 1
        print(f"Not visited cell count : {not_visited_cell_count}")

        if not_visited_cell_count == 0:
            # pass       # just not to get a syntax error, no purpose
            # must end the loop
            print("All done")
            seek_destination()
            
        if not_visited_cell_count == -1:        
            # print("All tasks Completed...")
            flash_led(1)
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            all_complete = 1
            return

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

    green = check_for_green()

    # assume returns false if blocked
    left = left_sensor_read()
    right = right_sensor_read()
    forward = forward_sensor_read()

    left = left and green[0]
    forward = forward and green[1]
    right = right and green[2]


    # cell_coverage[current_cell_X][current_cell_Y] = True

    update_graph(left, right, forward)


    if (left or right or forward) == False:
        do_a_180()
        motor_control(0,1,1)
    

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



def forward_sensor_read():
    # print(f"forw0: {forward_IR1.getValue()} forw1: {forward_IR2.getValue()}")
    if (forward_IR1.getValue() > 700) and (forward_IR2.getValue() > 700):
        return 0
    return 1

def right_sensor_read():
    # print(f"right: {sensor_to_distance(rightIR.getValue())}")
    if (rightIR.getValue()) > 700:
        return 0
    return 1
  
def left_sensor_read():
    # print(f"left: {sensor_to_distance(leftIR.getValue())}")
    if (leftIR.getValue()) > 700:
        return 0
    return 1  


def sensor_to_distance(value):
    # Find first V[i] < value
    try:
        i = next(i for i in range(len(V_lookup)-1) if V_lookup[i] >= value >= V_lookup[i+1])

        d1, v1 = D_lookup[i], V_lookup[i]
        d2, v2 = D_lookup[i+1], V_lookup[i+1]

        ratio = (value - v2) / (v1 - v2)

        return_val = d2 + ratio * (d1 - d2)

        return return_val * math.cos(math.asin(0.0165/0.164))
    
    except StopIteration:
        return 0.25 # Default max distance if out of range
    



def check_for_green():

    # Get image from Webots camera
    image = camera.getImage()

    # Convert raw image buffer to NumPy array (BGRA)
    img = np.frombuffer(image, np.uint8).reshape((height, width, 4))

    # Convert BGRA -> BGR
    frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    frame = cv2.resize(frame, (400, 80), interpolation=cv2.INTER_CUBIC)

    h, w, _ = frame.shape

    # -------------------------------
    # Define regions of interest
    regions = [
        frame[:, : (9 * w) // 100],
        frame[: ((5 * h) // 20), ((42 * w) // 100) : ((59 * w) // 100)],
        frame[:, (91 * w) // 100 :]
    ]

    green_is_Notpresent = []
    
    # Process each region
    for roi in regions:

        # Extract channels
        B = roi[:, :, 0]
        G = roi[:, :, 1]
        R = roi[:, :, 2]
                
        mask = (
            (G > B + 20) &
            (G > R + 20) &
            (G > 60)
        ).astype(np.uint8) * 255
        
        color_pixels = cv2.countNonZero(mask)
        ratio = color_pixels / mask.size

        if ratio > DETECTION_RATIO:
            print(f"[DETECTED] Color in (ratio={ratio:.3f})")
            flash_led(1)
            green_is_Notpresent.append(0)
            update_destination()
        else:
            green_is_Notpresent.append(1)
            
    # cv2.imshow("Webots Camera", frame)
    # cv2.waitKey(1)
    return green_is_Notpresent


###########################################################################################################
'''
functions related to finding the next best point to travel
'''



def find_nearest_frontier():

    '''
    Do a breadth first search and find the shortest length to a 0 value vertex

    add the consequent vertexes upto that in the path into navigation_path[]
    '''
    global navigation_path
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

    global current_cell_Row
    global current_cell_Col
    global navigation_path

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
    delta_change = (2*0.013*math.pi)/0.0205 + 0.35
    motor_control(delta_change, 1, -1)

    global current_orientation
    current_orientation = (current_orientation + 2) % 4

def turn_left():

    # based on axel distance, wheel radius
    delta_change = (0.013*math.pi)/0.0205 + 0.2
    motor_control(delta_change, -1, 1)
    print("turned left")

    global current_orientation
    current_orientation = (current_orientation - 1) % 4


def turn_right():

    delta_change = (0.013*math.pi)/0.0205 + 0.2
    motor_control(delta_change, 1, -1)
    print("turned right")

    global current_orientation
    current_orientation = (current_orientation + 1) % 4




def motor_control(angle, Lspeed, Rspeed):
    
    pid_enable = True

    if angle < 5:
        Lspeed *= 0.4275
        Rspeed *= 0.4275
        pid_enable = False
    else:
        reset_pid()

    leftMotor.setVelocity(Lspeed * MAX_SPEED)
    rightMotor.setVelocity(Rspeed * MAX_SPEED)
   
    presentPosL = leftWheelSensor.getValue()
    presentPosR = rightWheelSensor.getValue()
    
    while robot.step(TIME_STEP) != -1:
        if pid_enable:
            pid_controller()
        eL = angle - abs(presentPosL - leftWheelSensor.getValue())
        eR = angle - abs(presentPosR - rightWheelSensor.getValue())
        
        if eL < 0.001 and eR < 0.001:
            break                




def pid_controller():

    global prev_error, integral
    
    left = leftIR.getValue()
    right = rightIR.getValue()
    
    if left == 700 and right == 700:
        left = 0.094 # Assume centered
    elif left != 700 and right != 700:
        left = sensor_to_distance(left)
        right = sensor_to_distance(right)
        left *= (0.188 / (left + right))
    elif left != 700:
        left = dist_to_left(left, 0)
    else:
        left = (0.188 - dist_to_left(right, 1))
        
    error = left - 0.094
    
    P = Kp * error
    D = Kd * (error - prev_error)
    integral += error
    I = Ki * integral
    
    correction = P + D + I
    prev_error = error
    
    if int(robot.getTime() * 1000) % 320 == 0:
         print(f"Err: {error:.3f} | P: {P:.2f} D: {D:.2f} | Corr: {correction:.2f}")

    if correction > 0.18: correction = 0.18
    if correction < -0.18: correction = -0.18

    leftMotor.setVelocity(MAX_SPEED - correction)
    rightMotor.setVelocity(MAX_SPEED + correction)
    
    
def reset_pid():
    global prev_error, integral
    prev_error = 0.0
    integral = 0.0




def dist_to_left(value, index):
    
    value = sensor_to_distance(value)
    other_read = ir_array[index].getValue()

    if other_read < 1360:
        other_read = ir_array[index + 2].getValue()
        if other_read < 1360:
            return value * math.cos(0.26)
        
        other_read = sensor_to_distance(other_read)
        angle = 0.577 * (1 - ((2 * (value + 0.031)) / (other_read + 0.031)))
        return (value + 0.031) * math.cos(math.atan(angle)) - 0.031
    
    # Standard calculation
    angle = ((1.414 * (value + 0.031)) / (other_read + 0.031)) - 1
    return (value + 0.031) * math.cos(math.atan(angle)) - 0.031



###########################################################################################################
'''
functions related to other tasks
'''
    
def flash_led(state):
    for led in leds:
        led.set(state)
        
def update_destination():
    global final_dest_Row
    global final_dest_Col

    final_dest_Row += current_cell_Row
    final_dest_Col += current_cell_Col

def seek_destination():
    row = final_dest_Row % 12
    col = final_dest_Col % 12

    print(f"Final Destination ({row}, {col})")
    g.set_value((row, col), 0)