import time
import cv2
import numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Configuration Constants
OFFSET_X = 0  
OFFSET_Y = -40  
TARGET_LOWERING_ALTITUDE = 1.5  
KP_XY = 0.005 
ALIGNMENT_THRESHOLD = 30  

CLAW_CHANNEL = 3
CLAW_CLOSED_ANGLE = 60
CLAW_OPEN_ANGLE = 0

# Initialize Hardware
print("Initializing PCA9685 driver...")
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50
claw = servo.Servo(pca.channels[CLAW_CHANNEL])

print("Locking claw to closed position.")
claw.angle = CLAW_CLOSED_ANGLE

print("Initializing camera...")
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Connecting to flight controller at 115200 baud...")
master = mavutil.mavlink_connection('/dev/serial0', baud=115200)
master.wait_heartbeat()
print("Heartbeat received.")

# Request MAVLink data streams (RC channels and altitude)
master.mav.request_data_stream_send(
    master.target_system, master.target_component, 
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1
)
master.mav.request_data_stream_send(
    master.target_system, master.target_component, 
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA_1, 10, 1
)

# Helper Functions
def send_hud_alert(message):
    master.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_EMERGENCY, 
        message.encode('utf-8')
    )

def set_guided_mode():
    master.mav.set_mode_send(
        master.target_system, 
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
        4
    )

def set_loiter_mode():
    master.mav.set_mode_send(
        master.target_system, 
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
        5
    )

def send_velocity_command(vx, vy, vz):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component, 
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

# Main Execution State
last_alert_time = 0
in_autonomous_mode = False
claw_is_open = False

print("System ready. Waiting for RC commands.")

try:
    while True:
        # 1. Read Drone State
        rc_msg = master.recv_match(type='RC_CHANNELS', blocking=False)
        alt_msg = master.recv_match(type='VFR_HUD', blocking=False)
        current_alt = alt_msg.alt if alt_msg else 0.0
        
        if rc_msg:
            # Switch SA (Channel 7) - Auto Alignment Trigger
            if rc_msg.chan7_raw > 1500 and not in_autonomous_mode:
                in_autonomous_mode = True
                set_guided_mode()
                send_hud_alert("AUTO ALIGNMENT ENGAGED")
            elif rc_msg.chan7_raw < 1500 and in_autonomous_mode:
                in_autonomous_mode = False
                set_loiter_mode()
                send_hud_alert("MANUAL OVERRIDE")

            # Switch SB (Channel 8) - Payload Drop Trigger
            if rc_msg.chan8_raw > 1500 and not claw_is_open:
                claw.angle = CLAW_OPEN_ANGLE
                claw_is_open = True
                print("Command executed: Claw Opened")
            elif rc_msg.chan8_raw < 1500 and claw_is_open:
                claw.angle = CLAW_CLOSED_ANGLE
                claw_is_open = False
                print("Command executed: Claw Closed")

        # 2. Process Vision Array
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        full_mask = cv2.morphologyEx(mask1 + mask2, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        
        contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_found = False
        error_x = 0
        error_y = 0
        
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(biggest_contour) > 500:
                target_found = True
                x, y, w, h = cv2.boundingRect(biggest_contour)
                
                error_x = (x + (w // 2)) - (320 + OFFSET_X)
                error_y = (y + (h // 2)) - (240 + OFFSET_Y)

                if time.time() - last_alert_time > 5:
                    send_hud_alert("RED TARGET SPOTTED")
                    last_alert_time = time.time()

        # 3. Autonomous Flight Control Vectoring
        if in_autonomous_mode and target_found:
            if abs(error_x) < ALIGNMENT_THRESHOLD and abs(error_y) < ALIGNMENT_THRESHOLD:
                if current_alt > TARGET_LOWERING_ALTITUDE:
                    send_velocity_command(0, 0, 0.5) 
                else:
                    send_hud_alert("LOCKED! FLIP SWITCH SB TO DROP!")
                    set_loiter_mode()
                    in_autonomous_mode = False 
            else:
                vx = max(min(-error_y * KP_XY, 0.5), -0.5)
                vy = max(min(error_x * KP_XY, 0.5), -0.5)
                send_velocity_command(vx, vy, 0)
                
        elif in_autonomous_mode and not target_found:
            send_velocity_command(0, 0, 0)

except KeyboardInterrupt:
    print("\nProcess terminated by user.")
finally:
    picam2.stop()
    pca.deinit()
