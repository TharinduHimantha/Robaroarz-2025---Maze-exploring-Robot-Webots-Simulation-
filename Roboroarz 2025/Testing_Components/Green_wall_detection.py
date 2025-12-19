from controller import Robot
import math

# ---------- CONSTANTS ----------
TIME_STEP = 32
MAX_SPEED = 6.28
CELL_SIZE = 0.25
PROXIMITY_THRESHOLD = 0.35
DETECTION_COOLDOWN = 2.0

# ---------- ROBOT ----------
robot = Robot()

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

# Camera
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

# LEDs
leds = []
for i in range(10):
    led = robot.getDevice(f"led{i}")
    if led:
        leds.append(led)

# Encoders
left_enc = robot.getDevice("left wheel sensor")
right_enc = robot.getDevice("right wheel sensor")
left_enc.enable(TIME_STEP)
right_enc.enable(TIME_STEP)

# ---------- STATE ----------
x = y = theta = 0.0
prev_l = prev_r = 0.0
grid_x = grid_y = 0

found_coords = []
last_detection_time = -DETECTION_COOLDOWN

left_active = right_active = False
left_peak = right_peak = 0.0

checkpoint_active = False
checkpoint_start = 0.0

# ---------- MOTION ----------
def set_speed(l, r):
    left_motor.setVelocity(l)
    right_motor.setVelocity(r)

# ---------- ODOMETRY ----------
def update_odometry():
    global x, y, theta, prev_l, prev_r, grid_x, grid_y

    l = left_enc.getValue()
    r = right_enc.getValue()

    dl = (l - prev_l) * 0.0205
    dr = (r - prev_r) * 0.0205
    prev_l, prev_r = l, r

    da = (dr - dl) / 0.052
    dc = (dr + dl) / 2.0

    theta += da
    x += dc * math.sin(theta)
    y += dc * math.cos(theta)

    grid_x = -int(round(x / CELL_SIZE))
    grid_y = int(round(y / CELL_SIZE))

# ---------- VISION ----------
def is_green(r, g, b):
    return g > 120 and g > r + 40 and g > b + 40

def green_ratio_square(x0, y0, w, h):
    img = camera.getImage()
    if not img:
        return 0.0

    cam_w = camera.getWidth()
    green = total = 0

    for y in range(y0, y0 + h, 4):
        for x in range(x0, x0 + w, 4):
            r = camera.imageGetRed(img, cam_w, x, y)
            g = camera.imageGetGreen(img, cam_w, x, y)
            b = camera.imageGetBlue(img, cam_w, x, y)
            total += 1
            if is_green(r, g, b):
                green += 1

    return green / max(total, 1)

# ---------- CHECKPOINT ----------
def register_checkpoint():
    global last_detection_time, checkpoint_active, checkpoint_start

    now = robot.getTime()
    if now - last_detection_time < DETECTION_COOLDOWN:
        return

    for ox, oy in found_coords:
        if math.hypot(x - ox, y - oy) < PROXIMITY_THRESHOLD:
            return

    found_coords.append((x, y))
    last_detection_time = now
    checkpoint_active = True
    checkpoint_start = now

    print(f"[CHECKPOINT] Grid ({grid_x}, {grid_y})")

def check_checkpoint():
    global left_active, right_active, left_peak, right_peak

    w = camera.getWidth()
    h = camera.getHeight()
    y_near = int(h * 0.55)

    left = green_ratio_square(0, y_near, w // 4, h // 4)
    right = green_ratio_square(3 * w // 4, y_near, w // 4, h // 4)
    center = green_ratio_square(w // 3, y_near, w // 3, h // 4)

    if left > 0.12:
        left_active = True
        left_peak = max(left_peak, left)
    elif left_active and left_peak - left > 0.05:
        register_checkpoint()
        left_active = False
        left_peak = 0.0

    if right > 0.12:
        right_active = True
        right_peak = max(right_peak, right)
    elif right_active and right_peak - right > 0.05:
        register_checkpoint()
        right_active = False
        right_peak = 0.0

    if center > 0.5:
        register_checkpoint()

# ---------- MAIN ----------
robot.step(TIME_STEP)
prev_l = left_enc.getValue()
prev_r = right_enc.getValue()

while robot.step(TIME_STEP) != -1:
    update_odometry()

    if not checkpoint_active:
        check_checkpoint()

    if checkpoint_active:
        set_speed(0, 0)
        t = robot.getTime() - checkpoint_start
        blink = (t * 10) % 2 > 1
        for led in leds:
            led.set(1 if blink else 0)

        if t > 1.0:
            for led in leds:
                led.set(0)
            checkpoint_active = False
        continue

    set_speed(0.5 * MAX_SPEED, 0.5 * MAX_SPEED)
