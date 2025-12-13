from controller import Robot
import math
import random

TIME_STEP = 32

# -------------------- Speeds --------------------
FORWARD_SPEED = 6.28
TURN_SPEED = 2.0

# -------------------- Physical params --------------------
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.053

# -------------------- Angles (tuned for e-puck) --------------------
TURN_ANGLE_LEFT  = math.radians(97.5)
TURN_ANGLE_RIGHT = math.radians(96.95)
TURN_ANGLE_180   = math.radians(196)

WALL_THRESHOLD = 80

robot = Robot()

# -------------------- Motors --------------------
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# -------------------- Encoder --------------------
left_encoder = robot.getDevice("left wheel sensor")
left_encoder.enable(TIME_STEP)

# -------------------- Proximity sensors --------------------
ps = []
for i in range(8):
    s = robot.getDevice(f"ps{i}")
    s.enable(TIME_STEP)
    ps.append(s)

# -------------------- States --------------------
MOVE = 0
TURN_LEFT = 1
TURN_RIGHT = 2
TURN_180 = 3

state = MOVE
left_start = 0.0
target_rotation = 0.0

# -------------------- Target rotations --------------------
TARGET_LEFT  = (TURN_ANGLE_LEFT  * (AXLE_LENGTH / 2)) / WHEEL_RADIUS
TARGET_RIGHT = (TURN_ANGLE_RIGHT * (AXLE_LENGTH / 2)) / WHEEL_RADIUS
TARGET_180   = (TURN_ANGLE_180   * (AXLE_LENGTH / 2)) / WHEEL_RADIUS

# ==================== MAIN LOOP ====================
while robot.step(TIME_STEP) != -1:

    front_wall = ps[0].getValue() > WALL_THRESHOLD or ps[7].getValue() > WALL_THRESHOLD

    # ---------- MOVE ----------
    if state == MOVE:
        left_motor.setVelocity(FORWARD_SPEED)
        right_motor.setVelocity(FORWARD_SPEED)

        if front_wall:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            left_start = left_encoder.getValue()

            # 🔀 Randomly pick LEFT / RIGHT / 180
            state = random.choice([TURN_LEFT, TURN_RIGHT, TURN_180])

            if state == TURN_LEFT:
                target_rotation = TARGET_LEFT
                print("Wall hit → LEFT")
            elif state == TURN_RIGHT:
                target_rotation = TARGET_RIGHT
                print("Wall hit → RIGHT")
            else:
                target_rotation = TARGET_180
                print("Wall hit → 180")

    # ---------- TURN LEFT ----------
    elif state == TURN_LEFT:
        left_motor.setVelocity(-TURN_SPEED)
        right_motor.setVelocity(TURN_SPEED)

        if abs(left_encoder.getValue() - left_start) >= target_rotation:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            state = MOVE

    # ---------- TURN RIGHT ----------
    elif state == TURN_RIGHT:
        left_motor.setVelocity(TURN_SPEED)
        right_motor.setVelocity(-TURN_SPEED)

        if abs(left_encoder.getValue() - left_start) >= target_rotation:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            state = MOVE

    # ---------- TURN 180 ----------
    elif state == TURN_180:
        left_motor.setVelocity(TURN_SPEED)
        right_motor.setVelocity(-TURN_SPEED)

        if abs(left_encoder.getValue() - left_start) >= target_rotation:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            state = MOVE
