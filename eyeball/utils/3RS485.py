import asyncio
import serial_asyncio
import struct

class RS485MotorController:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.responses = {}

    async def connect(self):
        self.ser = await serial_asyncio.open_serial_connection(url=self.port, baudrate=self.baudrate)
        self.reader, self.writer = self.ser

    async def calculate_checksum(self, data):
        return sum(data) & 0xFF

    async def validate_checksum(self, response):
        if len(response) > 1:
            expected_checksum = sum(response[:-1]) & 0xFF
            actual_checksum = response[-1]
            return expected_checksum == actual_checksum
        return False

    async def send_command(self, motor_id, command, data=[]):
        """
        Send command asynchronously to the motor.
        """
        frame = [0x3E, command, motor_id, len(data)]
        frame.append(await self.calculate_checksum(frame))
        frame += data
        if data:
            frame.append(await self.calculate_checksum(data))

        b = bytearray(frame)
        self.writer.write(b)  # Send the command asynchronously
        await self.writer.drain()

    async def read_response(self):
        """
        Read motor response asynchronously.
        """
        response = await self.reader.read(13)  # Read 13 bytes asynchronously
        if response:
            motor_id = response[2]
            self.responses[motor_id] = response

    async def multi_loop_angle_control(self, motor_id, target_angle, maxspeed=None):
        """
        Send a multi-loop angle control command (CMD 0xA3 or 0xA4).
        """
        if maxspeed is None:
            target_angle_fixed = int(target_angle * 100)
            angle_data = list(struct.pack('<Q', target_angle_fixed))
            await self.send_command(motor_id, 0xA3, angle_data)
        else:
            target_angle_fixed = int(target_angle * 100)
            maxspeed_fixed = int(maxspeed * 100)
            angle_data = list(struct.pack('<Q', target_angle_fixed))
            maxspeed_data = list(struct.pack('<L', maxspeed_fixed))
            await self.send_command(motor_id, 0xA4, angle_data + maxspeed_data)

        asyncio.ensure_future(self.read_response())

    async def close(self):
        self.writer.close()
        await self.writer.wait_closed()

# Example usage with asyncio
async def main():
    controller = RS485MotorController(port='/dev/ttyUSB0')

    # Connect to the serial port
    await controller.connect()

    # Continuously send commands to the motors
    while True:
        for angle in range(45, 135):
            await controller.multi_loop_angle_control(1, angle)
            await controller.multi_loop_angle_control(2, angle)

        # Wait a small time between commands
        # await asyncio.sleep(0.1)

    # Close the connection
    await controller.close()

# Run the asyncio event loop
asyncio.run(main())
