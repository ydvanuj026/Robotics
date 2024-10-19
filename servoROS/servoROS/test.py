import serial

ser = serial.Serial('/dev/ttyACM2', 115200)

while True:
    for i in range(0, 180):
        ser.write(i.to_bytes(4, byteorder='big'))