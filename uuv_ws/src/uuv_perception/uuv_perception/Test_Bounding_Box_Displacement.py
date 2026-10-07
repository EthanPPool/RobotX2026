import cv2
import time
import torch
import numpy as np
from ultralytics import YOLO

# Select device \(GPU if available)
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# Load YOLO segmentation model
# Can load custom model if developed
# .pt is extension for created models.
model = YOLO("Buoy-Recog-v2.pt")
model.to(device)

# Webcam setup, destination zero is client.
# Destinations 1 and 2 are external sources.
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
assert cap.isOpened(), "Error opening webcam"

# Sets frame buffer, can be adjusted for latency.
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# From resolution setting, change interior values.
# Can also make w,h controllable variables prior,
# not a huge difference either way.
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# FPS fallback
fps = cap.get(cv2.CAP_PROP_FPS)
if fps == 0:
    fps = 30

# Video writer
video_writer = cv2.VideoWriter(
    "instance-segmentation.avi",
    cv2.VideoWriter_fourcc(*"mp4v"),
    fps,
    (w, h)
)

# Set time initial time to 0 for FPS calc.
prev_time = 0

# Main loop
while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("Failed to grab frame.")
        break

    # Resize for faster inference
    frame_small = cv2.resize(frame, (640, 640))

    # Run YOLO for frame caps
    results = model(frame_small, device=device)
    result = results[0]

    # Plot detections (on small frame)
    annotated = result.plot()

    # Scaling factors
    scale_x = w / 640 # uses half of frame size
    scale_y = h / 640

    # Frame center (original resolution)
    frame_cx = w // 2
    frame_cy = h // 2

    # Draw frame center (convert to small frame coords)
    center_small = (int(frame_cx / scale_x), int(frame_cy / scale_y))
    cv2.circle(annotated, center_small, 6, (255, 0, 0), -1)

    # Process detections
    if result.boxes is not None:
        for box in result.boxes:
            # Assigns coordinates to box corners
            # Use these to calc image centers.
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            # Scale to original resolution
            x1_o = x1 * scale_x
            x2_o = x2 * scale_x
            y1_o = y1 * scale_y
            y2_o = y2 * scale_y

            # Bounding box center (original resolution)
            cx = int((x1_o + x2_o) / 2)
            cy = int((y1_o + y2_o) / 2)

            # Convert center back to small frame for drawing
            cx_s = int(cx / scale_x)
            cy_s = int(cy / scale_y)

            # -----------------------------
            # Displacement calculations
            # -----------------------------
            dx = cx - frame_cx
            dy = -(cy - frame_cy)
            dist = np.sqrt(dx**2 + dy**2)

            # Normalized displacement (-1 to 1)
            dx_norm = dx / (w / 2)
            dy_norm = dy / (h / 2)

            # Draw frame center circle
            cv2.circle(annotated, (cx_s, cy_s), 5, (0, 0, 255), -1)

            # Line drawing, uses center coordinates and individual scaled centers to draw.
            cv2.line(
                annotated,
                (cx_s, cy_s),
                center_small,
                (255, 255, 0),
                2
            )

            # Display info
            label_x = int(x1)
            label_y = int(y1) - 10

            # dy and dx coordinate labeling
            cv2.putText(
                annotated,
                f"dx:{dx:.0f}, dy:{dy:.0f}",
                (label_x, label_y - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1
            )

            # Pythagorean theorem distance display (total)
            cv2.putText(
                annotated,
                f"d:{dist:.1f}",
                (label_x, label_y - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1
            )

            # Normalized distance display
            cv2.putText(
                annotated,
                f"nx:{dx_norm:.2f}, ny:{dy_norm:.2f}",
                (label_x, label_y - 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1
            )

    # Resize back to original resolution
    annotated = cv2.resize(annotated, (w, h))

    # FPS calc
    curr_time = time.time()
    fps_calc = 1 / (curr_time - prev_time) if prev_time != 0 else 0
    prev_time = curr_time

    # FPS text display
    cv2.putText(
        annotated,
        f"FPS: {fps_calc:.2f}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    # Display
    cv2.imshow("YOLO Segmentation + Displacement", annotated)

    # Save output
    video_writer.write(annotated)

    # Quit, key can be changed from q if needed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
video_writer.release()
cv2.destroyAllWindows()