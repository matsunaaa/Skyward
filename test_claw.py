import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize both servos as 270-degree
claw_servo = servo.Servo(pca.channels[0], actuation_range=270, min_pulse=500, max_pulse=2500)
winch_servo = servo.Servo(pca.channels[1], actuation_range=270, min_pulse=500, max_pulse=2500)

# Center both servos to start
claw_servo.angle = 135
winch_servo.angle = 135
time.sleep(1)

try:
    while True:
        # Close the claw
        print("Claw: CLOSING (Moving to 160 deg)")
        claw_servo.angle = 160 
        time.sleep(2)

        # Lower the winch
        print("Winch: LOWERING (Moving to 0 deg)")
        winch_servo.angle = 0
        time.sleep(3) 

        # Open the claw
        print("Claw: OPENING (Moving to 110 deg)")
        claw_servo.angle = 110
        time.sleep(2)

        # Raise the winch
        print("Winch: RAISING (Moving to 270 deg)")
        winch_servo.angle = 270
        time.sleep(3)

except KeyboardInterrupt:
    print("\nTest stopped by user.")
    
finally:
    pca.deinit()