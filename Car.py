import RPi.GPIO as GPIO
import time
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import serial
import pynmea2
import base64
from picamera import PiCamera
from io import BytesIO
import threading
import random

# Initialize Firebase with your credentials and database URL
cred = credentials.Certificate("securityKey.json")
firebase_admin.initialize_app(cred, {'databaseURL': 'https://rc-car-7eba6-default-rtdb.firebaseio.com/'})
refgps = db.reference('/location')
ref_car_move = db.reference('/Move/move')
ref_pan_tilt_move = db.reference('/PTMove')
ref_video_stream = db.reference('/liveVideoStream')

# Set up the GPIO mode and pin numbers
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Define GPIO pin numbers for motor control
m11 = 16  # Left motor forward
m12 = 12  # Left motor backward
m21 = 21  # Right motor forward
m22 = 20  # Right motor backward

# Set up the GPIO pins for motor direction control
GPIO.setup(m11, GPIO.OUT)
GPIO.setup(m12, GPIO.OUT)
GPIO.setup(m21, GPIO.OUT)
GPIO.setup(m22, GPIO.OUT)

# Create PWM objects for motor speed control
pwm_m11 = GPIO.PWM(m11, 1000)  # 1000 Hz frequency
pwm_m12 = GPIO.PWM(m12, 1000)
pwm_m21 = GPIO.PWM(m21, 1000)
pwm_m22 = GPIO.PWM(m22, 1000)

# Start PWM with 0% duty cycle (motors are initially off)
pwm_m11.start(0)
pwm_m12.start(0)
pwm_m21.start(0)
pwm_m22.start(0)

# Define movement functions with gradual speed control
def stop():
    print("stop")
    pwm_m11.ChangeDutyCycle(0)
    pwm_m12.ChangeDutyCycle(0)
    pwm_m21.ChangeDutyCycle(0)
    pwm_m22.ChangeDutyCycle(0)

def forward(speed):
    print("forward")
    GPIO.output(m11, 1)
    GPIO.output(m12, 0)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)
    pwm_m11.ChangeDutyCycle(speed)
    pwm_m12.ChangeDutyCycle(0)
    pwm_m21.ChangeDutyCycle(speed)
    pwm_m22.ChangeDutyCycle(0)

def backward(speed):
    print("backward")
    GPIO.output(m11, 0)
    GPIO.output(m12, 1)
    GPIO.output(m21, 0)
    GPIO.output(m22, 1)
    pwm_m11.ChangeDutyCycle(0)
    pwm_m12.ChangeDutyCycle(speed)
    pwm_m21.ChangeDutyCycle(0)
    pwm_m22.ChangeDutyCycle(speed)

def left(speed):
    print("left")
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)
    pwm_m11.ChangeDutyCycle(0)
    pwm_m12.ChangeDutyCycle(0)
    pwm_m21.ChangeDutyCycle(speed)
    pwm_m22.ChangeDutyCycle(0)

def right(speed):
    print("right")
    GPIO.output(m11, 1)
    GPIO.output(m12, 0)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)
    pwm_m11.ChangeDutyCycle(speed)
    pwm_m12.ChangeDutyCycle(0)
    pwm_m21.ChangeDutyCycle(0)
    pwm_m22.ChangeDutyCycle(0)

# Function to increase speed slightly for a specific direction
def increase_speed(direction):
    speeds = {"F": 0, "B": 0, "L": 0, "R": 0}
    current_speed = speeds[direction]
    # Increase the speed by a small increment (e.g., 10)
    new_speed = min(current_speed + 50, 100)  # Limit speed to a maximum of 100
    speeds[direction] = new_speed
    return new_speed

# Function to control the RC car based on Firebase commands
def control_rc_car(change):
    command = change.data

    if command == "F":
        forward_speed = increase_speed("F")
        forward(forward_speed)
    elif command == "B":
        backward_speed = increase_speed("B")
        backward(backward_speed)
    elif command == "S":
        stop()
    elif command == "L":
        left_speed = increase_speed("L")
        left(left_speed)
    elif command == "R":
        right_speed = increase_speed("R")
        right(right_speed)

# Function to move pan servo
def move_pan(direction):
    # Add your code to move the pan servo based on the direction received from Firebase
    pass

# Function to move tilt servo
def move_tilt(direction):
    # Add your code to move the tilt servo based on the direction received from Firebase
    pass

# Function to stop pan-tilt motors
def stop_pan_tilt():
    # Add your code to stop the pan-tilt motors
    pass

# Function to send pan-tilt move instruction periodically
def send_pan_tilt_instruction():
    while True:
        # Choose a random move instruction for pan and tilt (you can replace this with your logic)
        pan_instruction = random.choice(["left", "right"])
        tilt_instruction = random.choice(["up", "down"])
        ref_pan_tilt_move.set({"pan": pan_instruction, "tilt": tilt_instruction})

        # Wait for a short duration before sending the next instruction
        time.sleep(1)

# Function to update GPS location in Firebase
def update_gps_location():
    while True:
        port = "/dev/ttyAMA0"
        ser = serial.Serial(port, baudrate=9600, timeout=0.5)
        dataout = pynmea2.NMEAStreamReader()
        newdata = ser.readline()
        n_data = newdata.decode('latin-1')
        if n_data[0:6] == '$GPRMC':
            newmsg = pynmea2.parse(n_data)
            lat = newmsg.latitude
            lng = newmsg.longitude
            gps = "Latitude=" + str(lat) + " and Longitude=" + str(lng)
            print(gps)
            data = {"latitude": lat, "longitude": lng}
            refgps.set(data)
            print("Data sent")
        time.sleep(1)  # Adjust the sleep duration as needed

# Function to handle the live video stream
def video_stream():
    camera = PiCamera()
    camera.resolution = (270, 240)
    camera.framerate = 30

    try:
        stream = BytesIO()
        for foo in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
            stream.seek(0)
            frame_base64 = base64.b64encode(stream.read()).decode('utf-8')
            ref_video_stream.set({'frameData': frame_base64})
            stream.seek(0)
            stream.truncate()

    finally:
        camera.close()


def car_movement_control():
    while True:
        ref_car_move.listen(lambda event: control_rc_car(event))
        
# Initialize speeds for car movement
speeds = {"F": 0, "B": 0, "L": 0, "R": 0}

# Create threads for each function
gps_thread = threading.Thread(target=update_gps_location)
# pan_tilt_instruction_thread = threading.Thread(target=send_pan_tilt_instruction)
video_stream_thread = threading.Thread(target=video_stream)
car_movement_control_thread = threading.Thread(target=car_movement_control)

# Start the threads
gps_thread.start()
#pan_tilt_instruction_thread.start()
video_stream_thread.start()
car_movement_control_thread.start()

try:
    while True:
        pass
except KeyboardInterrupt:
    stop_pan_tilt()
    stop()
    # Clean up PWM and GPIO
    pwm_m11.stop()
    pwm_m12.stop()
    pwm_m21.stop()
    pwm_m22.stop()
    GPIO.cleanup()
