from robot_hat import Pin
import time

# Setup pins as digital input
left_sensor = Pin("D0", mode=Pin.IN, active_state=True)
center_sensor = Pin("D1", mode=Pin.IN, active_state=True)
right_sensor = Pin("D2", mode=Pin.IN, active_state=True)

print("📟 Starting grayscale sensor test. Move reflective/non-reflective surfaces under the sensors.")

try:
    while True:
        left_val = left_sensor.value()
        center_val = center_sensor.value()
        right_val = right_sensor.value()

        print(f"⬅️ Left: {left_val} | ⬛ Center: {center_val} | ➡️ Right: {right_val}")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\n🛑 Test stopped by user.")
