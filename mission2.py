import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from pymavlink import mavutil

# Configuration
CLAW_CHANNEL = 0
CLAW_CLOSED_ANGLE = 0
CLAW_OPEN_ANGLE = 60
BAUD_RATE = 115200

# Initialize PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50
claw = servo.Servo(pca.channels[CLAW_CHANNEL])

# Lock claw on startup
claw.angle = CLAW_CLOSED_ANGLE
claw_is_open = False

# Connect to Flight Controller
master = mavutil.mavlink_connection('/dev/serial0', baud=BAUD_RATE)
master.wait_heartbeat()

# Request RC channel data stream
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1
)

try:
    while True:
        msg = master.recv_match(type='RC_CHANNELS', blocking=True)
        
        if msg:
            # Switch SB is mapped to Channel 8
            if msg.chan8_raw > 1500 and not claw_is_open:
                claw.angle = CLAW_OPEN_ANGLE
                claw_is_open = True
            elif msg.chan8_raw < 1500 and claw_is_open:
                claw.angle = CLAW_CLOSED_ANGLE
                claw_is_open = False

except KeyboardInterrupt:
    pca.deinit()
