import time

# Simulate the serial.Serial class
class MockSerial:
    def __init__(self, port, baud_rate, timeout):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        print(f"MockSerial connected on {self.port} at {self.baud_rate} baud.")

    def write(self, data):
        # Simulate sending a command
        print(f"Sending command: {data.decode('utf-8')}")

    def close(self):
        print("MockSerial closed.")

# Replace the real 'serial.Serial' with the mock during testing
serial = MockSerial

# Your servo control function
def activate_servo():
    try:
        # Open serial connection (simulated)
        ser = serial("COM3", 9600, timeout=1)  # Change COM port if necessary
        time.sleep(2)  # Allow time for the connection to establish

        # Command to move the linear servo (simulated)
        command = "M\n"  # Replace with the actual command from the servo's manual
        ser.write(command.encode('utf-8'))
        print("Servo activated!")

        # Keep the servo engaged for 2 seconds before stopping
        time.sleep(2)

        # Stop the servo (simulated)
        stop_command = "S\n"  # Replace with the actual stop command
        ser.write(stop_command.encode('utf-8'))
        print("Servo stopped.")

        # Close the serial connection (simulated)
        ser.close()

    except Exception as e:
        print(f"Error: {e}")


