import time
import cv2
import numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil

# --- Configuration Constants ---
OFFSET_X = 0  
OFFSET_Y = -40  
TARGET_LOWERING_ALTITUDE = 1.5  
KP_XY = 0.005 
ALIGNMENT_THRESHOLD = 30  

# --- Initialize Hardware ---
print("Initializing camera...")
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Connecting to flight controller at 115200 baud...")
# Ensure SERIALx_BAUD in ArduPilot is set to 115
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
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10, 1
)

# --- Helper Functions ---
def send_hud_alert(message):
    master.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_EMERGENCY, 
        message.encode('utf-8')
    )

def set_guided_mode():
    master.mav.set_mode_send(
        master.target_system, 
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
        4 # ArduCopter GUIDED mode
    )

def set_loiter_mode():
    master.mav.set_mode_send(
        master.target_system, 
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
        5 # ArduCopter LOITER mode
    )

def send_velocity_command(vx, vy, vz):
    # Sends velocity commands in Body NED frame (X forward, Y right, Z down)
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component, 
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

# --- Main Execution State ---
last_alert_time = 0
in_autonomous_mode = False
current_alt = 0.0 
rc_msg = None

print("System ready. Waiting for RC commands.")

try:
    while True:
        # 1. Drain MAVLink Buffer for Freshest State
        while True:
            msg = master.recv_match(blocking=False)
            if not msg:
                break # Buffer is empty, we have the latest data
            
            if msg.get_type() == 'RC_CHANNELS':
                rc_msg = msg
            elif msg.get_type() == 'VFR_HUD':
                current_alt = msg.alt
        
        # 2. Process RC Commands (Flight Mode Triggers)
        if rc_msg:
            # Switch SD (Channel 11) - Auto Alignment Trigger
            if getattr(rc_msg, 'chan11_raw', 0) > 1500 and not in_autonomous_mode:
                in_autonomous_mode = True
                set_guided_mode()
                send_hud_alert("AUTO ALIGNMENT ENGAGED")
            elif getattr(rc_msg, 'chan11_raw', 0) < 1500 and in_autonomous_mode:
                in_autonomous_mode = False
                set_loiter_mode()
                send_hud_alert("MANUAL OVERRIDE")

        # 3. Process Vision Array
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        
        # Red requires two masks due to hue wrap-around in OpenCV
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        full_mask = cv2.morphologyEx(mask1 + mask2, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        
        contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_found = False
        error_x = 0
        error_y = 0
        
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            # Only track if the contour is reasonably sized (noise filtering)
            if cv2.contourArea(biggest_contour) > 500:
                target_found = True
                x, y, w, h = cv2.boundingRect(biggest_contour)
                
                # Calculate pixel error from center (with offsets)
                error_x = (x + (w // 2)) - (320 + OFFSET_X)
                error_y = (y + (h // 2)) - (240 + OFFSET_Y)

                # Send a HUD message every 5 seconds if target is visible
                if time.time() - last_alert_time > 5:
                    send_hud_alert("RED TARGET SPOTTED")
                    last_alert_time = time.time()

        # 4. Autonomous Flight Control Vectoring
        if in_autonomous_mode and target_found:
            # Check if drone is centered over the target
            if abs(error_x) < ALIGNMENT_THRESHOLD and abs(error_y) < ALIGNMENT_THRESHOLD:
                # If centered but too high, lower the drone
                if current_alt > TARGET_LOWERING_ALTITUDE:
                    send_velocity_command(0, 0, 0.5) 
                else:
                    # If centered and at drop altitude, lock in place and alert pilot
                    send_hud_alert("LOCKED! FLIP SWITCH SB TO DROP!")
                    set_loiter_mode()
                    in_autonomous_mode = False 
            else:
                # Calculate velocity based on pixel error
                vx = max(min(-error_y * KP_XY, 0.5), -0.5)
                vy = max(min(error_x * KP_XY, 0.5), -0.5)
                send_velocity_command(vx, vy, 0)
                
        elif in_autonomous_mode and not target_found:
            # Stop moving if target is lost while in auto mode to prevent flyaways
            send_velocity_command(0, 0, 0)

except KeyboardInterrupt:
    print("\nProcess terminated by user.")
finally:
    picam2.stop()
