import subprocess
import cv2
import numpy as np
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading

WIDTH = 640
HEIGHT = 480
frame_lock = threading.Lock()
current_frame = None

bg_subtractor = cv2.createBackgroundSubtractorMOG2(
    history=500,
    varThreshold=80,
    detectShadows=False
)
kernel = np.ones((5, 5), np.uint8)

def capture_loop():
    global current_frame
    cmd = [
        'rpicam-vid',
        '--width', str(WIDTH),
        '--height', str(HEIGHT),
        '--framerate', '30',
        '--codec', 'yuv420',
        '--timeout', '0',
        '--nopreview',
        '-o', '-'
    ]
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    frame_size = int(WIDTH * HEIGHT * 1.5)

    while True:
        raw = process.stdout.read(frame_size)
        if len(raw) != frame_size:
            continue

        yuv = np.frombuffer(raw, dtype=np.uint8).reshape((int(HEIGHT * 1.5), WIDTH))
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
        bgr = cv2.rotate(bgr, cv2.ROTATE_90_CLOCKWISE)
        fg_mask = bg_subtractor.apply(bgr)
        fg_mask = cv2.erode(fg_mask, kernel, iterations=1)
        fg_mask = cv2.dilate(fg_mask, kernel, iterations=2)

        contours, _ = cv2.findContours(
            fg_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 1500:
                x, y, w, h = cv2.boundingRect(largest)
                center_x = x + w // 2
                center_y = y + h // 2
                cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(bgr, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(bgr, f'Target: ({center_x}, {center_y})',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        _, jpeg = cv2.imencode('.jpg', bgr)
        with frame_lock:
            current_frame = jpeg.tobytes()

class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with frame_lock:
                    frame = current_frame
                if frame:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')

if __name__ == '__main__':
    t = threading.Thread(target=capture_loop)
    t.daemon = True
    t.start()
    print('Detection stream running at http://192.168.1.96:8080')
    HTTPServer(('0.0.0.0', 8080), StreamHandler).serve_forever()
