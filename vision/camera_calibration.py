"""
camera_calibrate.py  –  Calibrate focal length for phone camera (IP Webcam)

How to use:
1. Place the ball exactly CALIB_DISTANCE_CM away from the robot's foot (or camera).
2. Run this script. It will capture a frame, detect the ball, and print the focal length.
3. Copy the printed FOCAL_LENGTH_PX into your main ball_tracker.py constants.
"""

import cv2
import numpy as np
import urllib.request
from ultralytics import YOLO

# ---------- CONFIGURATION (same as in your main script) ----------
PHONE_IP = "192.168.1.18"          # ← change to your phone's IP (IP Webcam app)
PHONE_PORT = 8080                   # default IP Webcam port
SNAPSHOT_URL = f"http://{PHONE_IP}:{PHONE_PORT}/shot.jpg"

ALLOWED_BALL_CLASSES  = [32, 47]                  # COCO "sports ball"
CONFIDENCE_THRESHOLD = 0.3
BALL_REAL_DIAMETER_CM = 6.2         # change if using a mini ball
CALIB_DISTANCE_CM = 32.0        # the exact distance you placed the ball

# ---------- FUNCTIONS (copy from original) ----------
def capture_frame():
    """Grabs a single JPEG frame from the phone."""
    try:
        resp = urllib.request.urlopen(SNAPSHOT_URL, timeout=5)
        img_array = np.frombuffer(resp.read(), dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        return frame
    except Exception as e:
        print(f"[ERROR] Frame capture failed: {e}")
        return None

def detect_ball(frame):
    """Runs YOLOv8, returns the best ball detection (cx, cy, pixel_diameter, confidence)."""
    model = YOLO("yolov8s.pt")  # first run will download the model if needed
    results = model(frame, verbose=False)
    annotated = results[0].plot()
    cv2.imwrite("debug_detections.jpg", annotated) # for debugging. check this image to see what the model detected
    print("[INFO] Saved debug_detections.jpg")
    best = None
    best_conf = 0.0
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            print(f"[DEBUG] Detected class {class_id} ({model.names[class_id]}) with confidence {confidence:.3f}")

            if class_id not in ALLOWED_BALL_CLASSES or confidence < CONFIDENCE_THRESHOLD:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            if confidence > best_conf:
                best_conf = confidence
                best = {
                    "cx": (x1 + x2) // 2,
                    "cy": (y1 + y2) // 2,
                    "pixel_diameter": ((x2 - x1) + (y2 - y1)) / 2.0,
                    "confidence": confidence
                }
    return best

# ---------- MAIN CALIBRATION ROUTINE ----------
def calibrate_focal_length():
    print(f"\n[CALIBRATION] Place the ball exactly {CALIB_DISTANCE_CM} cm from the robot (or camera).")
    input("Press Enter when ready...")

    frame = capture_frame()
    if frame is None:
        print("[ERROR] Could not capture frame for calibration. Check Wi-Fi and URL.")
        return

    detection = detect_ball(frame)
    if detection is None:
        print("[ERROR] No ball detected. Ensure ball is clearly visible and well-lit.")
        return

    pixel_d = detection["pixel_diameter"]
    focal_length = (pixel_d * CALIB_DISTANCE_CM) / BALL_REAL_DIAMETER_CM

    print("\n[CALIBRATION RESULT]")
    print(f"  Pixel diameter at {CALIB_DISTANCE_CM} cm = {pixel_d:.1f} px")
    print(f"  Focal length = {focal_length:.2f} px")
    print(f"\n  → Copy this value into your main script:")
    print(f"     FOCAL_LENGTH_PX = {focal_length:.2f}")
    print("\n[DONE] Calibration complete.\n")

if __name__ == "__main__":
    calibrate_focal_length()