import RPi.GPIO as GPIO
import time
from heapq import heappop, heappush

# Define GPIO pins for ultrasonic sensors
TRIG_FRONT = 17
ECHO_FRONT = 27
TRIG_LEFT = 22
ECHO_LEFT = 23
TRIG_RIGHT = 24
ECHO_RIGHT = 25

# Define GPIO pins for motor control
PWM_LEFT = 18
PWM_RIGHT = 19
AIN1 = 20
AIN2 = 21
BIN1 = 12
BIN2 = 16

# Set GPIO mode and warnings
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialize ultrasonic sensor GPIO
GPIO.setup(TRIG_FRONT, GPIO.OUT)
GPIO.setup(ECHO_FRONT, GPIO.IN)
GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)

# Initialize motor control GPIO
GPIO.setup(PWM_LEFT, GPIO.OUT)
GPIO.setup(PWM_RIGHT, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)

# Set motor control PWM frequency
pwm_left = GPIO.PWM(PWM_LEFT, 100)
pwm_right = GPIO.PWM(PWM_RIGHT, 100)

# Define maze dimensions and obstacles
maze_width = 5
maze_height = 5
obstacles = [(1, 2), (2, 2), (3, 2), (4, 2)]

# Define movement directions
STOP = (0, 0, 0, 0)
FORWARD = (1, 0, 1, 0)
BACKWARD = (0, 1, 0, 1)
LEFT = (0, 1, 1, 0)
RIGHT = (1, 0, 0, 1)

# Define heuristic function for A* algorithm
def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

# Define A* algorithm for pathfinding
def astar(start, goal):
    queue = []
    heappush(queue, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while queue:
        current = heappop(queue)[1]

        if current == goal:
            break

        for next in get_neighbors(current):
            new_cost = cost_so_far[current] + 1

            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                heappush(queue, (priority, next))
                came_from[next] = current

    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

# Get neighboring cells for A* algorithm
def get_neighbors(cell):
    x, y = cell
    neighbors = []
    if x > 0:
        neighbors.append((x - 1, y))
    if x < maze_width - 1:
        neighbors.append((x + 1, y))
    if y > 0:
        neighbors.append((x, y - 1))
    if y < maze_height - 1:
        neighbors.append((x, y + 1))
    return neighbors

# Move the robot according to the given direction
def move_robot(direction):
    ain1, ain2, bin1, bin2 = direction
    GPIO.output(AIN1, ain1)
    GPIO.output(AIN2, ain2)
    GPIO.output(BIN1, bin1)
    GPIO.output(BIN2, bin2)

# Stop the robot
def stop_robot():
    move_robot(STOP)

# Initialize motors
pwm_left.start(0)
pwm_right.start(0)
move_robot(STOP)

# Measure distance using ultrasonic sensor
def measure_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    start_time = time.time()
    pulse_start = start_time
    pulse_end = start_time

    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
        if pulse_start - start_time > 0.5:
            return -1

    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
        if pulse_end - start_time > 0.5:
            return -1

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

# Main maze solving function
def solve_maze():
    start = (0, 0)
    goal = (maze_width - 1, maze_height - 1)
    path = astar(start, goal)

    for cell in path:
        x, y = cell
        print("Current position:", x, y)

        # Move forward until obstacle detected
        while measure_distance(TRIG_FRONT, ECHO_FRONT) > 10:
            move_robot(FORWARD)
            time.sleep(0.1)
        
        stop_robot()

        if cell != goal:
            # Rotate left or right based on available paths
            if measure_distance(TRIG_LEFT, ECHO_LEFT) > 10:
                move_robot(LEFT)
                time.sleep(0.5)
                stop_robot()
            elif measure_distance(TRIG_RIGHT, ECHO_RIGHT) > 10:
                move_robot(RIGHT)
                time.sleep(0.5)
                stop_robot()
            else:
                print("Stuck! No available path.")
                break
        else:
            print("Maze solved!")

    stop_robot()

# Call the maze solving function
solve_maze()

# Cleanup GPIO
GPIO.cleanup()

                   
