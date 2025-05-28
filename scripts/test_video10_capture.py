import cv2

cap = cv2.VideoCapture(10, cv2.CAP_V4L2)
if not cap.isOpened():
    print("❌ Failed to open /dev/video10")
    exit(1)

print("✅ Successfully opened /dev/video10")

ret, frame = cap.read()
print(f"Ret = {ret}, Frame shape = {frame.shape if ret else 'None'}")

cap.release()
