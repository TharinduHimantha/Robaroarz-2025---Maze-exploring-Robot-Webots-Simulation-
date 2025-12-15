import cv2
import numpy as np
import math


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

            print(f"Edge created between {v1},{v2}")

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
                print(f"path length : {len(path)}")
                return path     # <-- now a deque is returned

            # explore neighbors
            for nxt in self.neighbors(current):
                if nxt not in visited:
                    visited.add(nxt)

                    # copy path and append new node
                    new_path = deque(path)
                    new_path.append(nxt)

                    queue.append((nxt, new_path))
        print("Returning none")
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


current_cell_Row = 0
current_cell_Col = 0
current_orientation = 0
not_visited_cell_count = 144 

all_done = False

g = Graph() # for the graph



############################################################################################################
'''
Webots controller code
'''


def main_loop():

    print(f"orientation : {current_orientation}")

    if len(navigation_path) < 1:

        g.set_value((current_cell_Row, current_cell_Col), 1)
        global not_visited_cell_count
        not_visited_cell_count -= 1
        print(f"\n######### not visited count : {not_visited_cell_count}\n")

        if not_visited_cell_count == 0:
            print("All done")
            global all_done
            all_done = True
            return       # just not to get a syntax error, no purpose
            # must end the loop


        wall_sensor_monitoring()
        find_nearest_frontier()

    navigate_next_step()




# The real loop that runs on the controller




###########################################################################################################
'''
functions related to sensor reading and interpretation
'''

def wall_sensor_monitoring():

    wall_scan()

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







# -------------------------------
# Triangle robot class (same as before)
# -------------------------------
class TriangleRobot:
    def __init__(self, x, y, heading=0, size=20):
        self.x = x
        self.y = y
        self.heading = heading
        self.size = size
        self.prev_x = x
        self.prev_y = y

    def get_vertices(self):
        angle = math.radians(self.heading)
        tip = (int(self.x + self.size * math.cos(angle)),
               int(self.y + self.size * math.sin(angle)))
        left = (int(self.x + self.size * math.cos(angle + 2.5)),
                int(self.y + self.size * math.sin(angle + 2.5)))
        right = (int(self.x + self.size * math.cos(angle - 2.5)),
                 int(self.y + self.size * math.sin(angle - 2.5)))
        return np.array([tip, left, right], np.int32)

    def draw(self, img):
        pts = self.get_vertices()
        cv2.fillPoly(img, [pts], (0, 255, 0))
        cv2.circle(img, (self.x, self.y), 3, (0, 0, 255), -1)
        self.print_pose()

    def move_to(self, x, y, path_layer):
        self.prev_x, self.prev_y = self.x, self.y
        self.x = x
        self.y = y
        cv2.line(path_layer, (self.prev_x, self.prev_y),
                 (self.x, self.y), (0, 30, 255), 2)
        # self.print_pose()

    def rotate_left(self):
        self.heading = (self.heading - 90) % 360
        self.print_pose()

    def rotate_right(self):
        self.heading = (self.heading + 90) % 360
        self.print_pose()

    def print_pose(self):
        print(f"Position: ({self.x}, {self.y}) | Heading: {self.heading}°")

    def current_loc(self):
        return [self.x, self.y]
    



# -------------------------------
# Load your background image
# -------------------------------

import time

bg_image = cv2.imread("maze.jpg")  # replace with your image path
#bg_image = cv2.resize(bg_image, (500, 500))  # optional resize
path_layer = bg_image.copy()  # path drawn on top of the image

# Initial robot position
robot = TriangleRobot(717, 717, 270)
#robot.rotate_left()




def goForward():
    '''
    Related motor control code to go forward
    '''
    next_step()
    print("square changed")

def do_a_180():
    '''
    code to do a 180 turn
    '''
    turn_left()
    turn_left()

    

def turn_left():

    robot.rotate_left()

    global current_orientation

    current_orientation = (current_orientation - 1) % 4


def turn_right():

    robot.rotate_right()

    global current_orientation

    current_orientation = (current_orientation + 1) % 4


def free_available(x_, y_):
    # print(x_, y_)

    b, g, r = bg_image[y_, x_]

    if (b < 220) or (b > 238):
        return True
    if (g < 153) or (g > 171):
        return True
    if r > 9:
        return True
    
    return False

    # for x in range(x_ -1, x_ +2):
    #     for y in range(y_ -1, y_ +2):
    #         b, g, r = bg_image[y, x]
    #         print(f"pixels {b}, {g}, {r}")
    #         if b == 232 and g == 162 and r == 0:
    #             wall = True
    #             break
    #     if wall:
    #         break
    
    # return wall


free_around = [1, 1, 1, 1]


################################################################################
# Wall generation

maze_paths = [ [ [1,1,1,1] for _ in range(12) ] for __ in range(12) ]

for r in range(12):
    maze_paths[0][r][2] = 0

for r in range(12):
    maze_paths[11][r][0] = 0

for r in range(12):
    maze_paths[r][0][1] = 0

for r in range(12):
    maze_paths[r][11][3] = 0

top_wall = [
                [10, 10], [10, 9], [10, 8], [10, 7], [10, 6], [10, 4], [10, 3], [10, 2], [10, 1],
                [9, 1],
                [8, 10], [8, 8], [8, 7], [8, 5], [8, 4], [8, 3], [8, 2],
                [7, 9], [7, 8], [7, 6], [7, 5], [7, 4], [7, 1],
                [6, 10], [6, 9], [6, 7], [6, 6], [6, 3],
                [5, 6], [5, 5], [5, 0],
                [4,7], [4,2], 
                [3,10], [3,8], [3,7], [3,6], [3,5], [3,4], 
                [2,10], [2,6], [2,2],  
                [1,11], [1,6], [1,5], [1,4], [1,3], 
                [0,9], [0,8], [0,5], [0,4] 

            ]

for t in top_wall:
    maze_paths[ t[0] ] [ t[1] ] [0] = 0
    maze_paths[ t[0] + 1] [ t[1] ] [2] = 0
    # if (t[0] - 1) >= 0:
    #     maze_paths[ t[0] - 1] [ t[1] ] [2] = 0


right_wall = [
                [10,8], [10,7], [10,6], [10,4], [10,1], 
                [9,10], [9,9], [9,7], [9,6], [9,5], [9,3],  [9,1], 
                [8,7], [8,3], [8,1],
                [7,8], [7,7],  [7,5], [7,3], [7,2], [7,1],  
                [6,10], [6,8], [6,5], [6,4], [6,3], [6,2],
                [5,10], [5,9], [5,7], [5,5], [5,4], [5,2],
                [4,9], [4,6], [4,4], [4,2], [4,1],
                [3,9], [3,6], [3,4], [3,3], [3,2], [3,1],
                [2,9], [2,8], [2,5], [2,3], [2,1],
                [1,8], [1,2], [1,1],
                [0,1]

            ]

for t in right_wall:
    maze_paths[ t[0] ] [ t[1] ] [1] = 0
    if (t[1] - 1) >= 0:
        maze_paths[ t[0]] [ t[1] - 1] [3] = 0


#################################################################


def wall_scan():
    row = current_cell_Row
    col = current_cell_Col


    ##################################
    wall_distance = 29

    free_around[0] = maze_paths[row][col][0]
    free_around[1] = maze_paths[row][col][1]
    free_around[2] = maze_paths[row][col][2]
    free_around[3] = maze_paths[row][col][3]



# def wall_scan():
#     x = robot.current_loc()[0]
#     y = robot.current_loc()[1]


#     ##################################
#     wall_distance = 29

#     free_around[0] = True
#     free_around[1] = True
#     free_around[2] = True
#     free_around[3] = True


#     for t in range(y - 18, y - 42, -1):
#         value = free_available(x, t)
#         if value == False:
#             free_around[0] = False
#             break

#     for t in range(y + 12, y + 48, +1):
#         value = free_available(x, t)
#         if value == False:
#             free_around[2] = False
#             break

#     for t in range(x - 18, x - 42, -1):
#         value = free_available(t, y)
#         if value == False:
#             free_around[3] = False
#             break

#     for t in range(x + 12, y + 48, +1):
#         value = free_available(t, y)
#         if value == False:
#             free_around[1] = False
#             break


def forward_sensor_read():
    return free_around[current_orientation]

def right_sensor_read():
    return free_around[((current_orientation + 1 )% 4)]
  
def left_sensor_read():
    return free_around[((current_orientation - 1 )% 4)]


def next_step():
    ##################################
    square_len = 58

    if current_orientation == 0:
        robot.move_to(robot.x, robot.y - square_len, path_layer)
    elif current_orientation == 1:
        robot.move_to(robot.x + square_len, robot.y, path_layer)
    elif current_orientation == 2:
        robot.move_to(robot.x, robot.y + square_len, path_layer)
    elif current_orientation == 3:
        robot.move_to(robot.x - square_len, robot.y, path_layer)

# -------------------------------
# Main loop
# -------------------------------
while True:
    # Fresh frame = path + robot
    frame = path_layer.copy()
    robot.draw(frame)

    cv2.imshow("Triangle on Image Canvas", frame)

    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

    start_time = time.time()  # Record the start time

    # Loop until 0.75 seconds have passed
    while time.time() - start_time < 0.65:
        pass  # The loop does nothing, just waits until the time condition is met

    if all_done == False:

        main_loop()
    # key = cv2.waitKey(0) & 0xFF

    # if key == ord('q'):
    #     break
    # elif key == ord('a'):
    #     robot.rotate_left()
    # elif key == ord('d'):
    #     robot.rotate_right()
    # elif key == ord('w'):
    #     robot.move_to(robot.x, robot.y - 40, path_layer)
    # elif key == ord('s'):
    #     robot.move_to(robot.x, robot.y + 40, path_layer)
    # elif key == ord('x'):
    #     robot.move_to(robot.x + 40, robot.y, path_layer)
    # elif key == ord('z'):
    #     robot.move_to(robot.x - 40, robot.y, path_layer)

    
    

cv2.destroyAllWindows()
