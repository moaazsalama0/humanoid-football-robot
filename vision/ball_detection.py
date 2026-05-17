# ============================================================
#  ball_tracker.py  —  Hybrid YOLO + HSV Ball Tracking
#  Computer Vision for Humanoid Robot
# ============================================================

import cv2
import numpy as np
import serial
import time
from ultralytics import YOLO


# ─────────────────────────────────────────────
# SECTION 1: IMPORTS & CONSTANTS
# ─────────────────────────────────────────────

# --- Camera Settings ---
WEBCAM_INDEX = 1
WEBCAM_WIDTH = 1280
WEBCAM_HEIGHT = 720

# --- Display Settings ---
SHOW_DEBUG_WINDOW = True
DEBUG_WINDOW_NAME = "Hybrid Ball Tracker"

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.7

FONT_COLOR = (255, 255, 255)
BOX_COLOR = (0, 255, 0)

# --- Serial Settings ---
SERIAL_PORT = "COM7"
BAUD_RATE = 115200

# --- Ball Physical Properties ---
BALL_REAL_DIAMETER_CM = 6.2

# --- Calibration Constants ---
FOCAL_LENGTH_PX = 1316.13 #check camera_calibration.py to get this value for your camera

# --- Navigation Thresholds ---
KICK_DISTANCE_CM = 12.0
CENTER_TOLERANCE_PX = 100

# --- YOLO Settings ---
BALL_CLASS_IDS = [32, 47, 49, 77]
CONFIDENCE_THRESHOLD = 0.4

# --- HSV SETTINGS (GREEN BALL EXAMPLE) ---
# CHANGE THESE VALUES TO MATCH YOUR BALL COLOR

LOWER_HSV = np.array([35, 70, 70])
UPPER_HSV = np.array([85, 255, 255])

MIN_HSV_AREA = 1200

# --- Hybrid Settings ---
HSV_CLOSE_DISTANCE_CM = 25

# --- Timing ---
LOOP_DELAY_SEC = 0.03


# ─────────────────────────────────────────────
# SECTION 2: SETUP
# ─────────────────────────────────────────────

# Load YOLO model
model = YOLO("yolov8s.pt")

# Open webcam
cap = cv2.VideoCapture(WEBCAM_INDEX)

if not cap.isOpened():
    print(f"[ERROR] Could not open webcam index {WEBCAM_INDEX}")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, WEBCAM_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WEBCAM_HEIGHT)

# Open serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)

    time.sleep(2)

    print(f"[OK] Serial connected on {SERIAL_PORT}")
    print(f"[OK] Webcam opened (index {WEBCAM_INDEX})")

except Exception as e:
    print(f"[ERROR] Could not open serial port: {e}")

    cap.release()
    exit(1)


# ─────────────────────────────────────────────
# SECTION 3: HELPER FUNCTIONS
# ─────────────────────────────────────────────

def capture_frame():

    ret, frame = cap.read()

    if not ret:
        print("[WARN] Failed to capture frame")
        return None

    return frame

# YOLO DETECTION
def detect_ball_yolo(frame):

    results = model(frame, verbose=False)

    annotated_frame = results[0].plot()

    best = None
    best_conf = 0.0

    for result in results:

        for box in result.boxes:

            class_id = int(box.cls[0])
            confidence = float(box.conf[0])

            if class_id not in BALL_CLASS_IDS:
                continue

            if confidence < CONFIDENCE_THRESHOLD:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            if confidence > best_conf:

                best_conf = confidence

                best = {
                    "cx": (x1 + x2) // 2,
                    "cy": (y1 + y2) // 2,
                    "pixel_diameter": ((x2 - x1) + (y2 - y1)) / 2.0,
                    "confidence": confidence,
                    "bbox": (x1, y1, x2, y2),
                    "method": "YOLO"
                }

    return best, annotated_frame


# ============================================================
# HSV DETECTION
# ============================================================

def detect_ball_hsv(frame):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

    # Noise cleanup
    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) == 0:
        return None

    largest = max(contours, key=cv2.contourArea)

    area = cv2.contourArea(largest)

    if area < MIN_HSV_AREA:
        return None

    ((x, y), radius) = cv2.minEnclosingCircle(largest)

    if radius < 10:
        return None

    x = int(x)
    y = int(y)
    radius = int(radius)

    detection = {
        "cx": x,
        "cy": y,
        "pixel_diameter": radius * 2,
        "confidence": 1.0,
        "bbox": (
            x - radius,
            y - radius,
            x + radius,
            y + radius
        ),
        "method": "HSV"
    }

    return detection


# ============================================================
# DISTANCE ESTIMATION
# ============================================================

def estimate_distance(pixel_diameter):

    if pixel_diameter <= 0:
        return float('inf')

    return (BALL_REAL_DIAMETER_CM * FOCAL_LENGTH_PX) / pixel_diameter


# ============================================================
# HORIZONTAL OFFSET
# ============================================================

def estimate_horizontal_offset(ball_cx, frame_width):

    frame_center_x = frame_width // 2

    return ball_cx - frame_center_x


# ============================================================
# DRAW OVERLAY
# ============================================================

def draw_info_overlay(frame, detection, distance_cm, offset_px, command):

    h, w = frame.shape[:2]

    cv2.putText(
        frame,
        "Hybrid Ball Tracker",
        (10, 30),
        FONT,
        0.8,
        FONT_COLOR,
        2
    )

    if detection:

        method = detection["method"]

        status_text = f"✅ Ball Detected ({method})"

        color = (0, 255, 0)

        cv2.putText(
            frame,
            status_text,
            (10, 70),
            FONT,
            0.7,
            color,
            2
        )

        cv2.putText(
            frame,
            f"Distance: {distance_cm:.1f} cm",
            (10, 110),
            FONT,
            0.7,
            FONT_COLOR,
            2
        )

        cv2.putText(
            frame,
            f"Offset: {offset_px:+d}px",
            (10, 145),
            FONT,
            0.7,
            FONT_COLOR,
            2
        )

        cv2.putText(
            frame,
            f"Command: {command}",
            (10, 180),
            FONT,
            0.7,
            (0, 255, 255),
            2
        )

        # Draw bounding box
        x1, y1, x2, y2 = detection["bbox"]

        if method == "YOLO":
            color = (0, 255, 0)
        else:
            color = (255, 0, 255)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)

        cv2.circle(
            frame,
            (detection["cx"], detection["cy"]),
            5,
            (0, 0, 255),
            -1
        )

    else:

        cv2.putText(
            frame,
            "❌ Searching...",
            (10, 70),
            FONT,
            0.7,
            (0, 0, 255),
            2
        )

    cv2.putText(
        frame,
        "Press 'q' to quit",
        (10, h - 10),
        FONT,
        0.5,
        (200, 200, 200),
        1
    )

    return frame


# ============================================================
# COMMAND DECISION
# ============================================================

def decide_command(distance_cm, offset_px):

    if distance_cm <= KICK_DISTANCE_CM:
        return f"KICK,{distance_cm:.1f}"

    if offset_px > CENTER_TOLERANCE_PX:
        return f"RIGHT,{offset_px}"

    if offset_px < -CENTER_TOLERANCE_PX:
        return f"LEFT,{abs(offset_px)}"

    return f"FORWARD,{distance_cm:.1f}"


# ============================================================
# SERIAL SEND
# ============================================================

def send_command(command):

    try:

        message = command + "\n"

        ser.write(message.encode('utf-8'))

        print(f"[SENT] {command}")

    except Exception as e:

        print(f"[ERROR] Serial send failed: {e}")


# ─────────────────────────────────────────────
# SECTION 4: MAIN LOOP
# ─────────────────────────────────────────────

def main():

    print("\n[START] Hybrid robot navigation started.")

    search_sent = False

    while True:

        # =====================================
        # KEYBOARD INPUT
        # =====================================

        if SHOW_DEBUG_WINDOW:

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):

                print("\n[QUIT] User requested exit.")
                break

        # =====================================
        # CAPTURE FRAME
        # =====================================

        frame = capture_frame()

        if frame is None:

            time.sleep(LOOP_DELAY_SEC)
            continue

        frame_height, frame_width = frame.shape[:2]

        # =====================================
        # TRY YOLO FIRST
        # =====================================

        yolo_detection, display_frame = detect_ball_yolo(frame)

        final_detection = None

        # =====================================
        # YOLO SUCCESS
        # =====================================

        if yolo_detection is not None:

            distance_cm = estimate_distance(
                yolo_detection["pixel_diameter"]
            )

            # =================================
            # CLOSE RANGE → TRY HSV
            # =================================

            if distance_cm < HSV_CLOSE_DISTANCE_CM:

                hsv_detection = detect_ball_hsv(frame)

                if hsv_detection is not None:

                    final_detection = hsv_detection

                    print("[MODE] Using HSV (close range)")

                else:

                    final_detection = yolo_detection

                    print("[MODE] HSV failed → fallback to YOLO")

            else:

                final_detection = yolo_detection

                print("[MODE] Using YOLO")

        # =====================================
        # YOLO FAILED → TRY HSV
        # =====================================

        else:

            hsv_detection = detect_ball_hsv(frame)

            if hsv_detection is not None:

                final_detection = hsv_detection

                print("[MODE] YOLO failed → HSV succeeded")

        # =====================================
        # NOTHING DETECTED
        # =====================================

        if final_detection is None:

            if not search_sent:

                send_command("SEARCH")

                search_sent = True

            if SHOW_DEBUG_WINDOW:

                display_frame = draw_info_overlay(
                    display_frame,
                    None,
                    None,
                    None,
                    "SEARCH"
                )

                cv2.imshow(DEBUG_WINDOW_NAME, display_frame)

            time.sleep(LOOP_DELAY_SEC)

            continue

        # =====================================
        # BALL FOUND
        # =====================================

        search_sent = False

        distance_cm = estimate_distance(
            final_detection["pixel_diameter"]
        )

        offset_px = estimate_horizontal_offset(
            final_detection["cx"],
            frame_width
        )

        print(
            f"[DETECT] "
            f"{final_detection['method']} | "
            f"dist={distance_cm:.1f}cm | "
            f"offset={offset_px:+d}px"
        )

        # =====================================
        # DECIDE COMMAND
        # =====================================

        command = decide_command(
            distance_cm,
            offset_px
        )

        # =====================================
        # DRAW DISPLAY
        # =====================================

        if SHOW_DEBUG_WINDOW:

            display_frame = draw_info_overlay(
                display_frame,
                final_detection,
                distance_cm,
                offset_px,
                command
            )

            cv2.imshow(
                DEBUG_WINDOW_NAME,
                display_frame
            )

        # =====================================
        # SEND COMMAND
        # =====================================

        send_command(command)

        # =====================================
        # LOOP DELAY
        # =====================================

        time.sleep(LOOP_DELAY_SEC)

    # =========================================
    # CLEANUP
    # =========================================

    if SHOW_DEBUG_WINDOW:
        cv2.destroyAllWindows()

    cap.release()

    ser.close()

    print("[EXIT] Resources closed.")


# ─────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────

if __name__ == "__main__":

    try:
        main()

    except KeyboardInterrupt:

        print("\n[INTERRUPT] Stopped by user.")

        if SHOW_DEBUG_WINDOW:
            cv2.destroyAllWindows()

        cap.release()
        ser.close()