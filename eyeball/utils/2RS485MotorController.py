import serial
import struct
import time
import threading
from queue import Queue

class RS485MotorController:
    def __init__(self, port, baudrate=115200, timeout=None):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        # self.lock = threading.Lock()
        # self.response_event = threading.Event()  # Event to signal when a response is ready
        self.responses = {}
        # self.command_queue = Queue()  # Queue to manage commands
        self.response_bytes = 13

        # Start a separate thread for reading responses
        # self.response_thread = threading.Thread(target=self.read_response)
        # self.response_thread.start()

        # # Start a thread to process queued commands
        # self.command_thread = threading.Thread(target=self.process_commands)
        # self.command_thread.start()
        
        self.ser.flushInput()    #flush input buffer, discarding all its contents
        self.ser.flushOutput()   #flush output buffer, aborting current output

    def calculate_checksum(self, data):
        return sum(data) & 0xFF

    def validate_checksum(self, response):
        if len(response) > 1:
            expected_checksum = sum(response[:-1]) & 0xFF
            actual_checksum = response[-1]
            return expected_checksum == actual_checksum
        return False

    def interpret_response_as_integers(self, response):
        return [int(byte) for byte in response]

    def send_command(self, motor_id, command, data=[]):
        """
        Queue the command for asynchronous processing.
        """
        frame = [0x3E, command, motor_id, len(data)]
        frame.append(self.calculate_checksum(frame))
        frame += data
        if data != []:
            frame.append(self.calculate_checksum(data))

        b = bytearray(frame)
        self.ser.write(b)  # Queue the command instead of sending it directly

    def read_response(self):
        """
        Continuously read responses from the motor.
        """
        start_time = time.time()
        response = self.ser.read(self.response_bytes)
        print(f"Time taken to read response: {time.time() - start_time}")
        # print(f"Raw response (Hex): {[f'0x{byte:02X}' for byte in response]}")
        motor_id = response[2]
        self.responses[motor_id] = response

    def parse_response(self, response):
        if len(response) < 13:
            return "Invalid response: too short"
        
        frame_head = response[0]
        if frame_head != 0x3E:
            return "Invalid response: incorrect frame head"
        
        if not self.validate_checksum(response[0:5]):
            print("Invalid CMD checksum")
        if not self.validate_checksum(response[5:]):
            print("Invalid Data checksum")

        command = response[1]
        motor_id = response[2]
        data_length = response[3]
        data = response[5:]

        if command == 0x9A:
            if data_length >= 7:
                motor_state = {
                    'motor_id': motor_id,
                    'temperature_c': data[0],  # 1 byte
                    'motor_voltage': struct.unpack('<H', bytearray(data[1:3]))[0]*0.01,  # 2 bytes (16-bit unsigned)
                    'motor_on': 1 if data[5] == 0x00 else 0,  # 1 byte
                    'error_code': data[6]  # 1 byte
                }
                return motor_state
            else:
                return "Invalid data length for motor state"
        if command in [0x9C, 0xA3, 0xA4]:
            if data_length >= 7:
                motor_state = {
                    'motor_id': motor_id,
                    'temperature_c': data[0],  # 1 byte
                    'torque': struct.unpack('<H', bytearray(data[1:3]))[0],  # 2 bytes (16-bit unsigned)
                    'speed': struct.unpack('<H', bytearray(data[3:5]))[0],  # 2 bytes (16-bit unsigned)
                    'encoder_pos': struct.unpack('<H', bytearray(data[5:7]))[0],  # 2 bytes (16-bit unsigned)
                }
                return motor_state
            else:
                return "Invalid data length for motor state"
        elif command in [0x40]:
            if data_length >= 7:
                motor_state = {
                    'kp': struct.unpack('<H', bytearray(data[0:2]))[0],  # 2 bytes (16-bit unsigned)
                    'ki': struct.unpack('<H', bytearray(data[2:4]))[0],  # 2 bytes (16-bit unsigned)
                    'kd': struct.unpack('<H', bytearray(data[4:6]))[0],  # 2 bytes (16-bit unsigned)
                }
                return motor_state
            else:
                return "Invalid data length for motor state"
        else:
            return f"Unknown command: {command}"

    def motor_on(self, motor_id):
        self.send_command(motor_id, 0x88)

    def motor_off(self, motor_id):
        self.send_command(motor_id, 0x80)

    def multi_loop_angle_control(self, motor_id, target_angle, maxspeed=None):
        """
        Send a multi-loop angle control command (CMD 0xA3 or 0xA4).
        """
        if maxspeed is None:
            target_angle_fixed = int(target_angle * 100)
            angle_data = list(struct.pack('<Q', target_angle_fixed))
            start_time = time.time()
            self.send_command(motor_id, 0xA3, angle_data)
            print(f"Time taken to send command: {time.time() - start_time}")
            self.read_response()
        else:
            target_angle_fixed = int(target_angle * 100)
            maxspeed_fixed = int(maxspeed * 100)
            angle_data = list(struct.pack('<Q', target_angle_fixed))
            maxspeed_data = list(struct.pack('<L', maxspeed_fixed))
            self.send_command(motor_id, 0xA4, angle_data + maxspeed_data)
            self.read_response()

    def close(self):
        self.ser.close()

# Example usage
if __name__ == "__main__":
    controller = RS485MotorController(port='/dev/ttyUSB0')

    # Send angle commands to motor in a non-blocking way
    while True:
        for angle in range(45, 135):
            controller.multi_loop_angle_control(1, angle)
            controller.multi_loop_angle_control(2, angle)

            # for motor_id, response in controller.responses.items():
            #     if response:
            #         parsed_response = controller.parse_response(response)
            #         print(f"Motor {motor_id} response: {parsed_response}")

            # time.sleep(0.001)  # Adjust the sleep time to control the loop frequency
    # while True:
    #     controller.multi_loop_angle_control(2, 20)
    #     controller.multi_loop_angle_control(1, 20)

    controller.close()
