import socket
import network
import time
from machine import Pin, I2C

# Start access point mode
import network
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid='ESP32-AP', password='')
# ---- Motor Pins Setup ----
r_f = Pin(15, Pin.OUT)
r_b = Pin(18, Pin.OUT)
l_f = Pin(4, Pin.OUT)
l_b = Pin(5, Pin.OUT)

def all_off():
    r_f.value(1)
    r_b.value(1)
    l_f.value(1)
    l_b.value(1)

# ---- Initial State ----
all_off()

# Create socket
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)
print('Listening on', addr)

while True:
    cl, addr = s.accept()
    print('Client connected from', addr)
    request = cl.recv(1024).decode()
    
    if 'POST /forward' in request:
        print("Command: FORWARD")
        r_f.value(0); r_b.value(1)
        l_f.value(0); l_b.value(1)
    elif 'POST /right' in request:
        print("Command: RIGHT")
        r_f.value(1); r_b.value(0)
        l_f.value(0); l_b.value(1)
    elif 'POST /left' in request:
        print("Command: LEFT")
        r_f.value(0); r_b.value(1)
        l_f.value(1); l_b.value(0)
    elif 'POST /backward' in request:
        print("Command: BREAK")
        all_off()
    elif 'POST /level' in request:
        print("Command: BREAK")
        all_off()
    cl.send("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nOK")
    cl.close()
