from controller import Robot, Motor

import math

TIME_STEP = 32

MAX_SPEED = 6.28


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

# sensor reading lookup table
D = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25]
V = [4095, 3500, 2500, 1700, 1100, 700]

camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
#original FOV = 0.84
# new Fov = 2.2
# original camera 52,39

def motor_control(angle, Lspeed, Rspeed):

    pid_enable = False

    if angle < 5  :
        Lspeed *= 0.4275
        Rspeed *= 0.4275
        pid_enable = True

    leftMotor.setVelocity(Lspeed * MAX_SPEED)
    rightMotor.setVelocity(Rspeed * MAX_SPEED)
   
    
    presentPosL = leftWheelSensor.getValue()
    presentPosR = rightWheelSensor.getValue()
    
    while robot.step(TIME_STEP) != -1:
        eL = angle - abs(presentPosL - leftWheelSensor.getValue())
        eR = angle - abs(presentPosR - rightWheelSensor.getValue())
        if eL < 0.001 and eR < 0.001:
            break
        # if pid_enable == False:
                # print(f"left: {leftIR.getValue()} right: {rightIR.getValue()}")

    
   
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
    
    delta_change = (0.013*math.pi)/0.0205 + 0.07
    motor_control(delta_change, -1, 1)
    print("turning left")
    


def turn_right():

    delta_change = (0.013*math.pi)/0.0205 + 0.07
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
    return d2 + ratio * (d1 - d2)
    
    
    
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