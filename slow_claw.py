import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from pymavlink import mavutil
import sys

# Configuration
CLAW_CHANNEL = 3
CLAW_CLOSED_ANGLE = 80
CLAW_OPEN_ANGLE = 0
BAUD_RATE = 115200
SERVO_DELAY = 0.03 

try:
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50
    claw = servo.Servo(pca.channels[CLAW_CHANNEL])
except Exception as e:
    print(f"Hardware Error: {e}")
    sys.exit(1)

def move_servo_slow(target_angle, current_angle, speed):
    step = 1 if target_angle > current_angle else -1
    for angle in range(int(current_angle), int(target_angle) + step, step):
        clamped_angle = max(0, min(80, angle)) 
        claw.angle = clamped_angle
        time.sleep(speed)
    return target_angle

# Initialize state
current_claw_angle = CLAW_CLOSED_ANGLE
claw.angle = current_claw_angle
claw_is_open = False
print("Claw locked to closed (80 deg).")

print("Connecting to flight controller...")
try:
    master = mavutil.mavlink_connection('/dev/serial0', baud=BAUD_RATE)
    master.wait_heartbeat(timeout=10)
    print("Link established.")
except Exception as e:
    print(f"Connection Error: {e}")
    sys.exit(1)

master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1
)

print("Listening for Switch SB (Channel 8) changes...")

try:
    while True:
        msg = master.recv_match(type='RC_CHANNELS', blocking=True)
        
        if msg:
            if msg.chan8_raw > 1500 and not claw_is_open:
                print("Opening...")
                current_claw_angle = move_servo_slow(CLAW_OPEN_ANGLE, current_claw_angle, SERVO_DELAY)
                claw_is_open = True
                
            elif msg.chan8_raw < 1500 and claw_is_open:
                print("Closing...")
                current_claw_angle = move_servo_slow(CLAW_CLOSED_ANGLE, current_claw_angle, SERVO_DELAY)
                claw_is_open = False

except KeyboardInterrupt:
    print("\nProcess terminated.")
    pca.deinit()
