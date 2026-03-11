from flask import Flask, Response
import cv2
import numpy as np
from picamera2 import Picamera2

app = Flask(__name__)

# Boot up the Arducam
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

def generate_frames():
    while True:
        # Grab frame from hardware ISP
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        
        # Define Red color range
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks and combine them
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        full_mask = mask1 + mask2
        
        # Clean up the mask (erases tiny random red specks from the background)
        kernel = np.ones((5, 5), np.uint8)
        full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, kernel)
        
        # Find the outlines of the red objects
        contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the biggest red object (assuming this is our swimmer)
            biggest_contour = max(contours, key=cv2.contourArea)
            
            # Only track it if it's decently sized
            if cv2.contourArea(biggest_contour) > 500:
                x, y, w, h = cv2.boundingRect(biggest_contour)
                
                # Draw a targeting box around the swimmer
                cv2.rectangle(frame_bgr, (x, y), (x+w, y+h), (0, 255, 0), 3)
                
                # Find the center point of the target, draw
                target_x = x + (w // 2)
                target_y = y + (h // 2)
                cv2.circle(frame_bgr, (target_x, target_y), 5, (0, 0, 255), -1)
                
                # Calculate pixel offset from the true center of the camera (320, 240)
                error_x = target_x - 320
                error_y = target_y - 240
                
                # Print targeting coordinates onto the video feed
                cv2.putText(frame_bgr, f"Target X:{error_x} Y:{error_y}", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Draw a white crosshair in the center of the camera view
        cv2.line(frame_bgr, (320, 220), (320, 260), (255, 255, 255), 2)
        cv2.line(frame_bgr, (300, 240), (340, 240), (255, 255, 255), 2)

        # STREAM TO WEB BROWSER
        ret, buffer = cv2.imencode('.jpg', frame_bgr)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)