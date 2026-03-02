from pymavlink import mavutil
import time

# 1. Connect to the Cube via the Pi's hardware serial port
# TELEM 1 uses 57600 baud by default
print("Connecting to the Cube...")
connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# 2. Wait for the drone to say "hello" back
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Heartbeat found! The Pi and the Cube are talking.")

print("Listening to Channel 7... Flip your switch!")
print("-" * 40)

# 3. Create a loop to constantly read the incoming data
while True:
    # Sniff the MAVLink stream specifically for RC_CHANNELS packets
    message = connection.recv_match(type='RC_CHANNELS', blocking=True)
    
    if message:
        # Extract the raw PWM value from Channel 7
        ch7_pwm = message.chan7_raw
        
        # Standard RC PWM ranges from ~1000 to ~2000. 1500 is the middle.
        if ch7_pwm > 1500:
            print(f"Switch ON! (PWM: {ch7_pwm}) -> The claw would CLOSE now!")
        else:
            print(f"Switch OFF (PWM: {ch7_pwm})")
            
        # A tiny delay to keep your terminal from scrolling too fast
        time.sleep(0.2)