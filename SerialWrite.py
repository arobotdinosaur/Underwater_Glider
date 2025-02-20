#thanks to mr. gpt for some help on this, hope that's allowed in this class
import serial
import keyboard

# Change 'COMX' to the correct port
arduino = serial.Serial('/dev/cu.usbserial-10', 9600)

print("Press 'a' for topleft, 'd' for topright, 'w' for bottomleft 's' for bottomright Press 'q' to quit.")

while True:
    if keyboard.is_pressed('a'):
        arduino.write('a')  # Send a over serial
    if keyboard.is_pressed('d'):
	arduino.write(b'd')
    if keyboard.is_pressed('w'):
	arduino.write(b'w')
    if keyboard.is_pressed('s'):
	arduino.write(b's')
    if keyboard.is_pressed('q'):
        print("Exiting...")
        break

arduino.close()
