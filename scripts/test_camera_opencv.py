import cv2
import time

cap = cv2.VideoCapture("/dev/video0")  # Try 0, or use 2, 3, etc., based on working devices

if not cap.isOpened():
    print("❌ Failed to open camera")
    exit()

print("📸 Camera opened successfully. Press Ctrl+C to stop.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame")
        break

    # Just show shape for now — no GUI
    print("✅ Frame shape:", frame.shape)
    time.sleep(1)

cap.release()
