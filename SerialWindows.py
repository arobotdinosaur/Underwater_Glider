#Thanks to mr. gpt for this code
import serial
import time
import msvcrt

def get_char():
    return msvcrt.getch().decode('utf-8')

try:
    arduino = serial.Serial('COM3', 9600)  # Adjust port for Windows
    time.sleep(2)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

print("Enter 'a' for topleft, 'd' for topright, 'w' for bottomleft, 's' for bottomright. Enter 'q' to quit.")

while True:
    key = get_char()
    print(f"Sent: {key}")
    if key in ['a', 'd', 'w', 's']:
        arduino.write(key.encode())
    elif key == 'q':
        print("Exiting...")
        break

arduino.close()
