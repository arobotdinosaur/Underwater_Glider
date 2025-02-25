import serial
import time
import sys
import tty
import termios

def get_char():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)  # Read a single character
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

try:
    arduino = serial.Serial('/dev/cu.usbserial-10', 9600)
    time.sleep(2)  # Wait for serial connection to stabilize
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

print("Enter e for throttle up, w for throttle down, q for zero throttle, d for pitch up, s for pitch down, l yaw up, k yaw down. Enter 'p' to quit.")

while True:
    key = get_char()  # Get a single character without pressing enter
    print(f"Sent: {key}")  # Optional: print the character sent
    #if key in ['a', 'd', 'w', 's']:
    arduino.write(key.encode())  
    if key == 'p':
        print("Exiting...")
        break

arduino.close()
