from controller import Robot, Motor

import math

TIME_STEP = 32

MAX_SPEED = 6.1


# create the Robot instance.
robot = Robot()

# get a handler to the motors and set target position to infinity (speed control)
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

# sensor reading lookup table
D = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25]
V = [4095, 3500, 2500, 1700, 1100, 700]

Kp = 3.0
Kd = 0.0
prev_err = 0.0

camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
#original FOV = 0.84
# new Fov = 2.2
# original camera 52,39
# wall height 0.0495-0.033
# dist to wall = 0.164

def motor_control(angle, Lspeed, Rspeed):

    pid_enable = True

    if angle < 5  :
        Lspeed *= 0.4275
        Rspeed *= 0.4275
        pid_enable = False

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


   
def goForward():

    # global GO_FORWARD_VALUE

    # GO_FORWARD_VALUE = GO_FORWARD_VALUE + (0.25/0.0205)
    
    # leftMotor.setPosition(GO_FORWARD_VALUE)
    # rightMotor.setPosition(GO_FORWARD_VALUE)

    ## set up the motor speeds at 10% of the MAX_SPEED.
    # leftMotor.setVelocity(0.75 * MAX_SPEED)
    # rightMotor.setVelocity(0.75 * MAX_SPEED)
    
    print("next square ...")
    print("\n\n\n")
    goForwardValue = 0.25/0.0205
    motor_control(goForwardValue, 1, 1)
    
    
def turn_left():

    # global GO_FORWARD_VALUE

    # GO_FORWARD_VALUE = GO_FORWARD_VALUE + ((0.013*math.pi)/0.0205)
    
    # delta_change = (0.013*math.pi)/0.0205
    
    # leftMotor.setPosition(GO_FORWARD_VALUE)
    # rightMotor.setPosition(GO_FORWARD_VALUE + )

    ##set up the motor speeds at 10% of the MAX_SPEED.
    # leftMotor.setVelocity(-0.75 * MAX_SPEED)
    # rightMotor.setVelocity(0.75 * MAX_SPEED)
    
    delta_change = (0.013*math.pi)/0.0205 + 0.2
    motor_control(delta_change, -1, 1)
    print("turning left")
    


def turn_right():

    delta_change = (0.013*math.pi)/0.0205 + 0.2
    motor_control(delta_change, 1, -1)
    


def do_a_180():
    
    #delta_change = (0.026*math.pi)/0.0205
    delta_change = (2*0.013*math.pi)/0.0205 + 0.35
    motor_control(delta_change, 1, -1)
    # motor_control(delta_change, 1, -1)
    
    
    
def forward_sensor_read():
    print(f"forw0: {forward_IR1.getValue()} forw1: {forward_IR2.getValue()}")
    if (forward_IR1.getValue() > 700) and (forward_IR2.getValue() > 700):
        return 0
    return 1

def right_sensor_read():
    print(f"right: {sensor_to_distance(rightIR.getValue())}")
    if (rightIR.getValue()) > 700:
        return 0
    return 1
  
def left_sensor_read():
    print(f"left: {sensor_to_distance(leftIR.getValue())}")
    if (leftIR.getValue()) > 700:
        return 0
    return 1  
    
    
    
def sensor_to_distance(value):
    
    # Find first V[i] < value
    i = next(i for i in range(len(V)-1) if V[i] >= value >= V[i+1])

    d1, v1 = D[i], V[i]
    d2, v2 = D[i+1], V[i+1]

    ratio = (value - v2) / (v1 - v2)
    return_val = d2 + ratio * (d1 - d2)
    
    return return_val * math.cos(math.asin(0.0165/0.164))
    


def pid_controller():
                
    left = leftIR.getValue()
    right = rightIR.getValue()
    
    if left == 700 and right == 700:
        left = 0.094
        
    elif left != 700 and right != 700:
    
        print(f"Left sensor read: {left}")
       
    
    
        print(dist_to_left(left, 0))
        left = sensor_to_distance(left)
        print(f"Real world left = {left}")
        right = sensor_to_distance(right)
        
        left *= (0.188 / (left + right) )
        
        
    elif left != 700:
        left = dist_to_left(left, 0)
        
    else:
        left = (0.188 - dist_to_left(right, 1))
        
        
    error = left - 0.094
    correction = Kp * error
    global prev_error
    prev_error = error
    if abs(correction) > 0.18:
        correction *= (0.18 / abs(correction))
    print(f"dist: {left}, correction : {correction}")
        
    # else:
        
        # left = sensor_to_distance(left)
        # right = sensor_to_distance(right)
        
        # left *= (0.188 / (left + right) )
        
        # error = left - 0.094
        # correction = Kp * error
        # prev_error = error
        # print(f"dist: {left}, correction : {correction}")

    leftMotor.setVelocity(MAX_SPEED - correction)
    rightMotor.setVelocity(MAX_SPEED + correction)
    


def dist_to_left(value, index):
    print("This is dist_to_left")
    
    value = sensor_to_distance(value)
    print(f"value : {value}")
    
    other_read = ir_array[index].getValue()
    print(f"other_read: {other_read}")
    
    if other_read < 1360:
        other_read = ir_array[index + 2].getValue()
        print(f"new other_read: {other_read}")
        if other_read < 1360:
            return value*math.cos(0.26)
        
        other_read = sensor_to_distance(other_read)
        print(f"other_read distance: {other_read}")
        
        angle = 0.577 * ( 1 - ((2*(value+0.031))/(other_read+0.031)))
        print(f"angle {angle}")
        
        return (value+0.031) * math.cos(math.atan(angle)) - 0.031
    
    other_read = ir_array[index].getValue()
    angle = ((1.414 * (value+0.031))/(other_read+0.031)) - 1
        
    return (value+0.031) * math.cos(math.atan(angle)) - 0.031
    



   
    
def wall_sensor_monitoring():

    # assume returns false if blocked
    left = left_sensor_read()
    right = right_sensor_read()
    forward = forward_sensor_read()
   
    if forward:
        goForward()
    elif left:
        turn_left()
        goForward()
    elif right:
        turn_right()
        goForward()
    else:
        do_a_180()
        goForward()
    
    
while robot.step(TIME_STEP) != -1:
    # goForward()
    #print(Robot.getOrientation())
    # goForward()
    # goForward()
    # goForward()
    # goForward()
    
    # turn_left()
    # goForward()
    # turn_right()
    # goForward()
    # goForward()
    # do_a_180()
    wall_sensor_monitoring()

    pass