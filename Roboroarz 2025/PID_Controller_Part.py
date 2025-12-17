from controller import Robot, Motor
import math

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

# --- ROBOT SETUP ---
robot = Robot()

# Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Wheel Sensors
leftWheelSensor = robot.getDevice("left wheel sensor")
rightWheelSensor = robot.getDevice("right wheel sensor")
leftWheelSensor.enable(TIME_STEP)
rightWheelSensor.enable(TIME_STEP)

# IR Sensors
rightIR = robot.getDevice("ps2")
leftIR = robot.getDevice("ps5")
forward_IR1 = robot.getDevice("ps7")
forward_IR2 = robot.getDevice("ps0")

rightIR.enable(TIME_STEP)
leftIR.enable(TIME_STEP)
forward_IR1.enable(TIME_STEP)
forward_IR2.enable(TIME_STEP)

ir_array = [
    robot.getDevice("ps6"),
    robot.getDevice("ps1"),
    robot.getDevice("ps4"),
    robot.getDevice("ps3")
]

for sensor in ir_array:
    sensor.enable(TIME_STEP)

# Camera
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

# Lookup Tables
D_lookup = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25]
V_lookup = [4095, 3500, 2500, 1700, 1100, 700]


# --- HELPER FUNCTIONS ---

def reset_pid():
    global prev_error, integral
    prev_error = 0.0
    integral = 0.0


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


# --- CONTROLLER LOGIC ---

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


# --- MOVEMENT COMMANDS ---

def goForward():
    print("Moving Forward...")
    goForwardValue = 0.25 / 0.0205
    motor_control(goForwardValue, 1, 1)

def turn_left():
    print("Turning Left...")
    delta_change = (0.013 * math.pi) / 0.0205 + 0.2
    motor_control(delta_change, -1, 1)

def turn_right():
    print("Turning Right...")
    delta_change = (0.013 * math.pi) / 0.0205 + 0.2
    motor_control(delta_change, 1, -1)

def do_a_180():
    print("Doing 180...")
    delta_change = (2 * 0.013 * math.pi) / 0.0205 + 0.35
    motor_control(delta_change, 1, -1)


# --- SENSOR CHECKS ---

def forward_sensor_read():
    if (forward_IR1.getValue() > 700) and (forward_IR2.getValue() > 700):
        return 0 # Blocked
    return 1 # Clear

def right_sensor_read():
    if (rightIR.getValue()) > 700:
        return 0 # Blocked
    return 1 # Clear
  
def left_sensor_read():
    if (leftIR.getValue()) > 700:
        return 0 # Blocked
    return 1 # Clear 

def wall_sensor_monitoring():
    # 0 = Blocked, 1 = Clear
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


# --- MAIN LOOP ---

print(Kp, Kd, Ki)
robot.step(TIME_STEP)

while robot.step(TIME_STEP) != -1:
    wall_sensor_monitoring()