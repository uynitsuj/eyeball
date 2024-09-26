import serial
import struct
import threading
import time

class RS485MotorController:
    def __init__(self, port, baudrate=115200, timeout=0.1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.lock = threading.Lock()  # Ensure thread safety
        self.response_thread = threading.Thread(target=self.read_responses, daemon=True)
        self.response_thread.start()
        self.responses = {}

    def calculate_checksum(self, data):
        return sum(data) & 0xFF  # Keeping the lower 8 bits, discarding the high bits

    def send_command(self, motor_id, command, data=[]):
        with self.lock:
            frame = [0x3E, command, motor_id, len(data)]  # Frame head, Command, ID, Data length
            frame += data
            frame.append(self.calculate_checksum(frame))
            self.ser.write(bytearray(frame))

    def read_responses(self):
        while True:
            response = self.ser.read(100)  # Adjust as needed based on expected response size
            if response:
                motor_id = response[2]  # Assuming motor ID is the 3rd byte
                self.responses[motor_id] = response

    def get_response(self, motor_id):
        # Return the latest response for the given motor
        return self.responses.get(motor_id)

    def motor_on(self, motor_id):
        self.send_command(motor_id, 0x88)

    def motor_off(self, motor_id):
        self.send_command(motor_id, 0x80)

    def read_motor_state(self, motor_id):
        self.send_command(motor_id, 0x9A)
        return self.get_response(motor_id)

    def speed_control(self, motor_id, speed):
        speed_data = list(struct.pack('<I', int(speed * 100)))
        self.send_command(motor_id, 0xA2, speed_data)

    def angle_control(self, motor_id, angle):
        angle_data = list(struct.pack('<Q', int(angle * 100)))
        self.send_command(motor_id, 0xA3, angle_data)

    def stop_motor(self, motor_id):
        self.send_command(motor_id, 0x81)

    def close(self):
        self.ser.close()

# Example usage
if __name__ == "__main__":
    controller = RS485MotorController(port='/dev/ttyUSB0', baudrate=115200)

    # Send commands to motor with ID 1 at high frequency without blocking
    for i in range(100):
        controller.speed_control(1, 100 + i)  # Gradually increase speed
        time.sleep(0.01)  # Small delay for example purposes, adjust as needed

    controller.close()
