import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize servo as 270-degree
claw_servo = servo.Servo(pca.channels[0], actuation_range=270, min_pulse=500, max_pulse=2500)

# Center servos to start
claw_servo.angle = 135
time.sleep(1)

try:
    while True:
        print("Claw:60 deg")
        claw_servo.angle = 60 
        time.sleep(2)


        print("Claw:110 deg")
        claw_servo.angle = 110
        time.sleep(2)
        
        print("Claw:180 deg")
        claw_servo.angle = 180 
        time.sleep(2)


        

except KeyboardInterrupt:
    print("\nTest stopped by user.")
    
finally:
    pca.deinit()
