import RPi.GPIO as GPIO
import time

# Set up GPIO pins for IR sensor
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN)
GPIO.setup(20, GPIO.IN)
GPIO.setup(16, GPIO.IN)
GPIO.setup(12, GPIO.IN)
GPIO.setup(7, GPIO.IN)
GPIO.setup(8, GPIO.IN)
GPIO.setup(25, GPIO.IN)
GPIO.setup(24, GPIO.IN)

# PID constants
Kp = 0.2
Ki = 0.0
Kd = 0.0

# PID variables
error = 0.0
error_prev = 0.0
integral = 0.0
derivative = 0.0

# Set up GPIO pins for motor control
GPIO.setup(18, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)

# Set up PWM for motor control
pwm1 = GPIO.PWM(18, 100)
pwm2 = GPIO.PWM(23, 100)
pwm1.start(0)
pwm2.start(0)

# Main loop
while True:
    # Read IR sensor values
    ir1 = GPIO.input(21)
    ir2 = GPIO.input(20)
    ir3 = GPIO.input(16)
    ir4 = GPIO.input(12)
    ir5 = GPIO.input(7)
    ir6 = GPIO.input(8)
    ir7 = GPIO.input(25)
    ir8 = GPIO.input(24)
    
    # Calculate error value
    error_prev = error
    error = (ir1*128 + ir2*64 + ir3*32 + ir4*16 + ir5*8 + ir6*4 + ir7*2 + ir8*1) - 255
    integral += error
    derivative = error - error_prev
    
    # Calculate PID output
    output = Kp*error + Ki*integral + Kd*derivative
    
    # Set motor speeds based on PID output
    pwm1.ChangeDutyCycle(50 + output)
    pwm2.ChangeDutyCycle(50 - output)
    
    time.sleep(0.01)
