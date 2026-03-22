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

# Center both servos to start
claw_servo.angle = 135
winch_servo.angle = 135
time.sleep(1)

try:
    while True:
        #wide open
        print("Claw:110 deg")
        claw_servo.angle = 110
        time.sleep(2)
        
        print("Claw:160 deg")
        claw_servo.angle = 160 
        time.sleep(2)

        print("Claw:200 deg")
        claw_servo.angle = 200 
        time.sleep(2)

        print("Claw:220 deg")
        claw_servo.angle = 220 
        time.sleep(2)



        

except KeyboardInterrupt:
    print("\nTest stopped by user.")
    
finally:
    pca.deinit()
