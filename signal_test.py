from pymavlink import mavutil
import sys

# Establish Connection
try:
    master = mavutil.mavlink_connection('/dev/serial0', baud=115200)
    master.wait_heartbeat(timeout=10)
    if not master.target_system:
        print("ERROR: No heartbeat detected. Check serial wiring and baud rate.")
        sys.exit(1)
except Exception as e:
    print(f"Connection Failed: {e}")
    sys.exit(1)

# Request Data Streams (RC Channels and Heartbeat State)
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1
)

# State Trackers
flight_mode = "UNKNOWN"
is_armed = False

try:
    while True:
        # Listen for any incoming MAVLink message
        msg = master.recv_match(blocking=True, timeout=1.0)
        
        if not msg:
            continue
            
        msg_type = msg.get_type()

        # Track Drone State (Mode and Arming)
        if msg_type == 'HEARTBEAT':
            flight_mode = mavutil.mode_string_v10(msg)
            is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

        # Track RadioMaster Switch Data
        if msg_type == 'RC_CHANNELS':
            ch7 = msg.chan7_raw  # Switch SA (Primary Auto/Drop)
            ch8 = msg.chan8_raw  # Switch SB (Available)
            ch9 = msg.chan9_raw  # Switch S1 (Available)
            
            # --- RENDER LIVE DASHBOARD ---
            # \033[H\033[J clears the terminal screen so data updates in place
            sys.stdout.write("\033[H\033[J") 
            print("========================================")
            print("INDOOR SIGNAL DIAGNOSTICS DASHBOARD")
            print("========================================")
            print("LINK STATUS: Pi <-> Cube Connected")
            print(f"FLIGHT MODE: {flight_mode}")
            
            arm_status = "ARMED (DANGER: PROPS LIVE!)" if is_armed else "DISARMED (SAFE)"
            print(f"ARM STATE:   {arm_status}")
            print("----------------------------------------")
            print("RADIOMASTER SWITCHES (Live RC Data):")
            print(f"   CH 7 (SA) : {ch7}  [{'DOWN/ACTIVE' if ch7 > 1500 else 'UP/IDLE'}]")
            print(f"   CH 8 (SB) : {ch8}  [{'DOWN/ACTIVE' if ch8 > 1500 else 'UP/IDLE'}]")
            print(f"   CH 9 (S1) : {ch9}  [{'HIGH' if ch9 > 1500 else 'LOW'}]")
            
            if is_armed:
                print("\n WARNING: DRONE IS ARMED INDOORS! DO NOT TOUCH THROTTLE!")
            
            print("\n(Flick your switches to verify real-time transmission)")
            sys.stdout.flush()

except KeyboardInterrupt:
    print("\n\n Diagnostics terminated. Safe to power down.")
