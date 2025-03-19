import serial
import time

#list min: 992
#listed max: 2000
#actual min: 1040
#actual max: 1950

SERIAL_PORT = "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Micro_Maestro_6-Servo_Controller_00454274-if00"
BAUD_RATE = 9600

def send_command(channel, target):
    command = bytearray([0x84, channel, target & 0x7F, (target >> 7) & 0x7F])
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        ser.write(command)
        time.sleep(0.1)

def activate_servo():
    print("Activating Servo...")
    send_command(0, 8000)
    time.sleep(10)

def retract_servo():
    print("Retracting Servo...")
    send_command(0, 4000)
    time.sleep(5)

activate_servo()
time.sleep(2)
retract_servo()