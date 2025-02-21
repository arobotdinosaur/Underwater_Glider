import serial
import time

try:
    arduino = serial.Serial('/dev/cu.usbserial-10', 9600)
    time.sleep(2)  # Wait for serial connection to stabilize
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

print("Enter 'a' for topleft, 'd' for topright, 'w' for bottomleft, 's' for bottomright. Enter 'q' to quit.")

while True:
    key = input("Enter command: ")  # Waits for user input
    if key in ['a', 'd', 'w', 's']:
        arduino.write(key.encode())  # Send input as bytes
    elif key == 'q':
        print("Exiting...")
        break

arduino.close()
