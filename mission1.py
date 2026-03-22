import time
import cv2
import numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# ==========================================
# CONSTANTS & TUNING PARAMETERS
# ==========================================

# Camera Alignment (Tune these so the hook is physically centered over the target)
# Positive X = target needs to move right in frame
# Positive Y = target needs to move down in frame
OFFSET_X = 0  
OFFSET_Y = -40  

# Autonomous Descent Altitude (Meters relative to home launch altitude)
TARGET_LOWERING_ALTITUDE = 1.5  

# Servo Configurations
CLAW_CHANNEL = 0
CLAW_CLOSED_ANGLE = 90
CLAW_OPEN_ANGLE = 0

# PID Control Proportional Gain (How aggressively the drone moves to center the target)
KP_XY = 0.005 
# Pixel threshold to be considered centered
ALIGNMENT_THRESHOLD = 30  

# ==========================================
# HARDWARE INITIALIZATION
# ==========================================

print("Initializing PCA9685 Claw...")
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50
claw = servo.Servo(pca.channels[CLAW_CHANNEL])

# Lock the lifesaver in
claw.angle = CLAW_CLOSED_ANGLE

print("Initializing Camera...")
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Connecting to Flight Controller...")
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
master.wait_heartbeat()

# Request RC channels and Altitude data
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1)
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA_1, 10, 1)

# ==========================================
# HELPER FUNCTIONS
# ==========================================

def send_hud_alert(message):
    """Sends a high-priority text alert to the Mission Planner HUD."""
    master.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_EMERGENCY,
        message.encode('utf-8')
    )

def set_guided_mode():
    """Forces the flight controller into GUIDED mode for autonomous movement."""
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4 # Mode 4 is GUIDED in ArduCopter
    )

def set_loiter_mode():
    """Returns control back to the pilot with GPS hold."""
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        5 # Mode 5 is LOITER in ArduCopter
    )

def send_velocity_command(vx, vy, vz):
    """
    Sends velocity vector to the drone (Body Frame).
    vx = Forward(+)/Backward(-)
    vy = Right(+)/Left(-)
    vz = Down(+)/Up(-)
    """
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111, # Bitmask to enable velocity
        0, 0, 0, # Positions (ignored)
        vx, vy, vz, # Velocities m/s
        0, 0, 0, 0, 0)

# ==========================================
# MAIN MISSION LOOP
# ==========================================

last_alert_time = 0
in_autonomous_mode = False
target_centered = False

try:
    while True:
        # Read Drone State
        rc_msg = master.recv_match(type='RC_CHANNELS', blocking=False)
        alt_msg = master.recv_match(type='VFR_HUD', blocking=False)
        
        current_alt = alt_msg.alt if alt_msg else 0.0
        
        if rc_msg:
            # Switch SA (Channel 7) triggers autonomous alignment
            if rc_msg.chan7_raw > 1500 and not in_autonomous_mode:
                in_autonomous_mode = True
                set_guided_mode()
                send_hud_alert("AUTO ALIGNMENT ENGAGED")
            elif rc_msg.chan7_raw < 1500 and in_autonomous_mode:
                in_autonomous_mode = False
                set_loiter_mode()
                send_hud_alert("MANUAL OVERRIDE")

        # Process Vision
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        
        # Red HSV ranges
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        full_mask = mask1 + mask2
        
        # Noise reduction
        kernel = np.ones((5, 5), np.uint8)
        full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_found = False
        error_x = 0
        error_y = 0
        
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(biggest_contour) > 500:
                target_found = True
                x, y, w, h = cv2.boundingRect(biggest_contour)
                
                # Raw center
                raw_target_x = x + (w // 2)
                raw_target_y = y + (h // 2)
                
                # Apply offsets
                aim_point_x = 320 + OFFSET_X
                aim_point_y = 240 + OFFSET_Y
                
                error_x = raw_target_x - aim_point_x
                error_y = raw_target_y - aim_point_y

                # Debounce HUD alerts to once every 5 seconds
                if time.time() - last_alert_time > 5:
                    send_hud_alert("RED TARGET SPOTTED")
                    last_alert_time = time.time()

        # Autonomous Flight Control Logic
        if in_autonomous_mode and target_found:
            # Check if aligned within threshold
            if abs(error_x) < ALIGNMENT_THRESHOLD and abs(error_y) < ALIGNMENT_THRESHOLD:
                
                if current_alt > TARGET_LOWERING_ALTITUDE:
                    # Centered, but too high -> Descend straight down
                    send_velocity_command(0, 0, 0.5) 
                else:
                    # Centered and at proper altitude -> Hand off to pilot to pull
                    send_hud_alert("TARGET LOCKED. READY TO PULL")
                    set_loiter_mode()
                    in_autonomous_mode = False 
            else:
                # Not centered -> Calculate velocity vector (Assumes top of frame = drone forward)
                # If error_x is positive (target is right), drone must roll right (positive vy)
                # If error_y is positive (target is below center), drone must pitch backward (negative vx)
                vx = -error_y * KP_XY
                vy = error_x * KP_XY
                
                # Cap maximum speed for safety
                vx = max(min(vx, 0.5), -0.5)
                vy = max(min(vy, 0.5), -0.5)
                
                send_velocity_command(vx, vy, 0)
                
        elif in_autonomous_mode and not target_found:
            # Target lost during auto mode -> Stop moving and hold position
            send_velocity_command(0, 0, 0)

except KeyboardInterrupt:
    print("\nMission Aborted.")
finally:
    picam2.stop()