import serial
import struct
import time

class RS485MotorController:
    def __init__(self, port, baudrate=460800, timeout=None):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def calculate_checksum(self, data):
        return sum(data) & 0xFF

    def validate_checksum(self, response):
        if len(response) > 1:
            expected_checksum = sum(response[:-1]) & 0xFF
            actual_checksum = response[-1]
            # print(f"Expected checksum: {expected_checksum}, Actual checksum: {actual_checksum}")
            return expected_checksum == actual_checksum
        return False

    def interpret_response_as_integers(self, response):
        return [int(byte) for byte in response]

    def send_command(self, motor_id, command, data=[]):
        # with self.lock:
        frame = [0x3E, command, motor_id, len(data)]
        frame.append(self.calculate_checksum(frame))
        frame += data
        if data != []:
            frame.append(self.calculate_checksum(data))

        b = bytearray(frame)
        print("Sending:", "".join([f"\\x{byte:02x}" for byte in b]))
        self.ser.write(b)

    def read_response(self, bytes=13):
        while True:
            try:
                response = self.ser.read(bytes)
                if response:
                    if len(response) > 2:
                        return response
            except OSError as e:
                print(f"Error reading from serial port: {e}")
                break
        
    def parse_response(self, response):
        # Ensure the response is long enough to contain the minimum required information
        if len(response) < 13:
            return "Invalid response: too short"
        frame_head = response[0]
        if frame_head != 0x3E:
            return "Invalid response: incorrect frame head"
        
        # print(f"Raw response (Hex): {[f'0x{byte:02X}' for byte in response]}")
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
        else:
            return f"Unknown command: {command}"
        
    def motor_on(self, motor_id):
        self.send_command(motor_id, 0x88)

    def motor_off(self, motor_id):
        self.send_command(motor_id, 0x80)

    def read_motor_state(self, motor_id, type=1):
        """
        type: 0 - Read motor temperature, voltage, and error code
              1 - Read motor temperature, torque, output power, speed, encoder position
        """
        if type == 0:
            self.send_command(motor_id, 0x9A)
            return self.parse_response(self.read_response(13))
        elif type == 1:
            self.send_command(motor_id, 0x9C)
            return self.parse_response(self.read_response(13))
    
    def multi_loop_angle_control(self, motor_id, target_angle, maxspeed = None):
        """
        Multi-loop angle control command (CMD 0xA3 or 0xA4).
        Takes the target angle in float, converts it to a 64-bit integer, and sends it over RS485.
        
        :param motor_id: ID of the motor to control
        :param target_angle: Desired angle as a float (in degrees)
        """
        
        if maxspeed is None:
            target_angle_fixed = int(target_angle * 100)            
            angle_data = list(struct.pack('<Q', target_angle_fixed))  # '<Q' 64-bit unsigned integer, little-endian
            command_id = 0xA3
            
            self.send_command(motor_id, command_id, angle_data)
            return self.parse_response(self.read_response(13))
        else:
            target_angle_fixed = int(target_angle * 100)
            maxspeed_fixed = int(maxspeed * 100)
            angle_data = list(struct.pack('<Q', target_angle_fixed))
            maxspeed_data = list(struct.pack('<L', maxspeed_fixed))
            command_id = 0xA4
            
            self.send_command(motor_id, command_id, angle_data + maxspeed_data)
            return self.parse_response(self.read_response(13))
        
    def close(self):
        self.ser.close()

# Example usage
if __name__ == "__main__":
    controller = RS485MotorController(port='/dev/ttyUSB0', baudrate=115200)

    # Send read motor state command and print response
    # while True:
    #     # record response frequency
    #     # start_time = time.time()
    #     response = controller.read_motor_state(1)
        
    #     if response:
    #         print("Response received:", response)
    #     else:
    #         print("No response received")
        # print(f"Response time(hz): {1/(time.time()-start_time)}")
    
    # response = controller.multi_loop_angle_control(1, 0)
    # print("Response received:", response)
    # while True:
    #     # turn position back and forth between 0 and 360 degrees
    #     for angle in range(0, 360):
    #         response = controller.multi_loop_angle_control(1, angle)
    #         print("Response received:", response)
            # time.sleep(0.1)
            
    # 29 deg lower bound/zero axis1
    response = controller.multi_loop_angle_control(1, 29+90 + -45, 360)
    print("Response received:", response)
    response = controller.multi_loop_angle_control(2, 120+ 10, 360)
    print("Response received:", response)