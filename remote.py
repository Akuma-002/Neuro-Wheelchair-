import network
import urequests
import time
from machine import Pin, I2C
from mpu6050 import MPU6050

# ---- I2C Setup ----
i2c = I2C(0, scl=Pin(22), sda=Pin(21))

# ---- Scan I2C Devices ----
devices = i2c.scan()
if 0x68 not in devices:
    print("MPU6050 not detected. Check wiring and power.")
    raise SystemExit()

# ---- MPU6050 Setup ----
mpu = MPU6050(i2c)

# ---- Tilt Threshold ----
TILT_THRESHOLD = 0.2

# Wi-Fi setup
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.connect('ESP32-AP', '')  # Connect to server's AP

# LED setup (onboard LED is usually GPIO 2)
led = Pin(2, Pin.OUT)
led.value(0)  # Turn off initially

connected_printed = False

def send_command(cmd):
    try:
        url = f"http://192.168.4.1/{cmd}"
        response = urequests.post(url)
        print(f"Sent: {cmd}, Response: {response.text}")
        response.close()
    except Exception as e:
        print("Request failed:", e)

while True:
    if sta.isconnected():
        led.value(1)  # Connected indicator
        if not connected_printed:
            print("Connected to ESP32-AP")
            connected_printed = True
    else:
        led.value(0)
        if connected_printed:
            print("Disconnected. Reconnecting...")
        connected_printed = False
        time.sleep(1)
        continue

    try:
        accel = mpu.get_accel()
        print("Raw accel data:", accel)
        x, y, z = accel

                      # Unpack values from tuple

        direction = "Level"  # Default

        if abs(x) < TILT_THRESHOLD and abs(y) < TILT_THRESHOLD:
            direction = "Level"
            send_command("level")

        elif y > TILT_THRESHOLD:
            direction = "FORWARD"
            send_command("forward")

        elif y < -TILT_THRESHOLD:
            direction = "BACKWARD"
            send_command("backward")

        elif x > TILT_THRESHOLD:
            direction = "RIGHT"
            send_command("right")

        elif x < -TILT_THRESHOLD:
            direction = "LEFT"
            send_command("left")

        print(f"X: {x:.2f}g, Y: {y:.2f}g → {direction}")
        time.sleep(0.1)

    except OSError as e:
        print("MPU6050 read error:", e)
        time.sleep(0.5)
    
    time.sleep(0.05)



