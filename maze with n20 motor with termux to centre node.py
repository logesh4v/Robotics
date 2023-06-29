import RPi.GPIO as GPIO
import time

# Set up GPIO pins for motor control and ultrasonic sensors
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(17, GPIO.IN)
GPIO.setup(27, GPIO.IN)
GPIO.setup(22, GPIO.IN)
GPIO_TRIGGER = 18
GPIO_ECHO1 = 23
GPIO_ECHO2 = 24
GPIO_ECHO3 = 25
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO1, GPIO.IN)
GPIO.setup(GPIO_ECHO2, GPIO.IN)
GPIO.setup(GPIO_ECHO3, GPIO.IN)

# Define functions for motor control
def motor1_forward():
    GPIO.output(13, GPIO.HIGH)
    GPIO.output(19, GPIO.LOW)

def motor1_backward():
    GPIO.output(13, GPIO.LOW)
    GPIO.output(19, GPIO.HIGH)

def motor1_stop():
    GPIO.output(13, GPIO.LOW)
    GPIO.output(19, GPIO.LOW)

def motor2_forward():
    GPIO.output(26, GPIO.HIGH)
    GPIO.output(16, GPIO.LOW)

def motor2_backward():
    GPIO.output(26, GPIO.LOW)
    GPIO.output(16, GPIO.HIGH)

def motor2_stop():
    GPIO.output(26, GPIO.LOW)
    GPIO.output(16, GPIO.LOW)

def motor3_forward():
    GPIO.output(21, GPIO.HIGH)
    GPIO.output(20, GPIO.LOW)

def motor3_backward():
    GPIO.output(21, GPIO.LOW)
    GPIO.output(20, GPIO.HIGH)

def motor3_stop():
    GPIO.output(21, GPIO.LOW)
    GPIO.output(20, GPIO.LOW)

def motor4_forward():
    GPIO.output(12, GPIO.HIGH)
    GPIO.output(21, GPIO.LOW)

def motor4_backward():
    GPIO.output(12, GPIO.LOW)
    GPIO.output(21, GPIO.HIGH)

def motor4_stop():
    GPIO.output(12, GPIO.LOW)
    GPIO.output(21, GPIO.LOW)

# Define function for maze solving algorithm
def solve_maze():
    motor1_forward()
    motor2_forward()
    motor3_forward()
    motor4_forward()
    while True:
        # Measure distances using ultrasonic sensors
        GPIO.output(GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
        start_time = time.time()
        stop_time = time.time()
        while GPIO.input(GPIO_ECHO1) == 0:
            start_time = time.time()
        while GPIO.input(GPIO_ECHO1) == 1:
            stop_time = time.time()
        elapsed_time = stop_time - start_time
        distance1 = elapsed_time * 34300 / 2

        GPIO.output(GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
        start_time = time.time()
        stop_time = time.time()
        while GPIO.input(GPIO_ECHO2) == 0:
            start_time = time.time()
        while GPIO.input(GPIO_ECHO2) == 1:
            stop_time = time.time()
        elapsed_time = stop_time - start_time
       
