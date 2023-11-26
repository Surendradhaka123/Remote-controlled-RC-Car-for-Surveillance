import RPi.GPIO as GPIO
import time
import firebase_admin
from firebase_admin import credentials, db

# Set your Firebase credentials
cred = credentials.Certificate('securityKey.json')
firebase_admin.initialize_app(cred, {'databaseURL': 'https://rc-car-7eba6-default-rtdb.firebaseio.com/' })
database = db.reference('PTMove')

# Set the GPIO mode and pins
GPIO.setmode(GPIO.BCM)
pan_pin = 18  # GPIO pin for pan servo
tilt_pin = 17  # GPIO pin for tilt servo

# Set the PWM frequency and duty cycle ranges
pan_pwm_frequency = 50  # Hz
tilt_pwm_frequency = 50  # Hz
pwm_range = 100  # Duty cycle range (0 to 100)

# Set the initial position
current_pan_position = 7.5  # Neutral position
current_tilt_position = 7.5  # Neutral position

# Setup PWM for pan servo
GPIO.setup(pan_pin, GPIO.OUT)
pan_pwm = GPIO.PWM(pan_pin, pan_pwm_frequency)
pan_pwm.start(current_pan_position / 10.0 * pwm_range)

# Setup PWM for tilt servo
GPIO.setup(tilt_pin, GPIO.OUT)
tilt_pwm = GPIO.PWM(tilt_pin, tilt_pwm_frequency)
tilt_pwm.start(current_tilt_position / 10.0 * pwm_range)

# Function to move pan servo
def move_pan(direction):
    global current_pan_position
    if direction == 'L':
        current_pan_position += 4
    elif direction == 'R':
        current_pan_position -= 4
    pan_pwm.ChangeDutyCycle(current_pan_position)

# Function to move tilt servo
def move_tilt(direction):
    global current_tilt_position
    if direction == 'U':
        current_tilt_position += 1.5
    elif direction == 'D':
        current_tilt_position -= 1.5
    tilt_pwm.ChangeDutyCycle(current_tilt_position)

# Firebase listener
def on_snapshot(event):
    direction = event.data
    print(direction)
    if direction in ['L', 'R']:
        move_pan(direction)
    elif direction in ['U', 'D']:
        move_tilt(direction)

# Start listening to Firebase changes
database.child('direction').listen(on_snapshot)

# Cleanup GPIO on program exit
def cleanup():
    pan_pwm.stop()
    tilt_pwm.stop()
    GPIO.cleanup()

try:
    # Your main program loop goes here
    while True:
        pass

except KeyboardInterrupt:
    cleanup()
    
