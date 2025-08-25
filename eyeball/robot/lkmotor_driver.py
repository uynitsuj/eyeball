from array import array
import math
import time, sys, os
from struct import *
import struct
import numpy as np
import serial, json
from concurrent.futures import ThreadPoolExecutor, Future
import threading


CMD_HEADER                     = 0x3E
CMD_ASK_MULTI_LOOP_ANGLE       = 0x92        #Read multi -loop Angle command
CMD_ASK_SINGLE_LOOP_ANGLE      = 0x94        #Read single -loop Angle command
CMD_ABS_MULTI_LOOP_ANGLE_SPEED = 0xA4        #MULTI ROTATION ABS ANGLE Multi position closed loop control command 2
CMD_ABS_SINGLE_ANGLE_SPEED     = 0xA6        #Single position closed loop control command 2 Angle 0...359.99 deg Rotation direction is set by outside
CMD_INC_ANGLE_SPEED            = 0xA8        #INCREMENT angle with speed
CMD_SET_ZERO                   = 0x19        #Set current poosition as zero for driver
CMD_MOTOR_SHUTDOWN             = 0x80        #MOTOR stop (but power on coils will keep?)
CMD_MOTOR_STOP                 = 0x81        #MOTOR stop (but power on coils will keep?)
CMD_MOTOR_START                = 0x88        #MOTOR operation
CMD_MOTOR_MODEL                = 0x12        #Read driver and motor model commands


class Serializer():
    def serialize(self):
        members = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")]

        ret_dict = {}

        for member in members:
            m_name = str(member)
            m_value = getattr(self, member)

            if hasattr(m_value, 'serialize'):
                ret_dict[m_name] = m_value.serialize()
            else:
                ret_dict[m_name] = getattr(self, member)
        
        return ret_dict

    def json_serialize(self):
        return json.dumps(self.serialize(), sort_keys=True, indent=4)

def time_of_function(function):
    def wrapped(*args):
        start_time = time.perf_counter_ns()
        res = function(*args)
        print("Time of function", (time.perf_counter_ns() - start_time) / 10**6, "millisec")
        return res
    return wrapped

class Motor():
    serial_port                    = None
    id: hex                        = 0x00
    name: str                      = None
    __cur_multi_loop_angle: float    = 0.0
    __cur_single_loop_angle: float   = 0.0
    
    tolerance: float               = 0.1 #deg. Used for different operatins like wait_stop Must be > 0.01!

    # Simulation rotation
    sim_CW: bool                   = True
    
    # Tooltip shift. Motor's zero point is bottom axis of motor.
    sim_shifts                     = [0.0, 0.0, 0.0]
    sim_rot_plane: str             = "YZ"

    def __init__(self, id: hex, serial_port, tolerance: float, name: str, CW: bool, zero_angle: float, sim_shifts: list, sim_rot_plane: str):
        assert id           >= 0
        assert tolerance    >= 0.01
        #assert serial_port  != None    #no check for case of simulation
        assert name         != None
        
        #Yep, I know about enums!
        assert sim_rot_plane.upper() in ["XY", "XZ", "YZ"], "Wrong rotation plane. It must be XY, XZ or YZ"        

        self.serial_port    = serial_port
        self.id             = id
        self.tolerance      = tolerance
        self.name           = name
        self.zero_angle     = zero_angle

        self.sim_CW         = CW
        self.sim_shifts     = sim_shifts
        self.sim_rot_plane  = sim_rot_plane.upper()
    
    def __read_response(self, bytes_expect: int):
        self.serial_port.timeout = 0.1
        
        res = self.serial_port.read(bytes_expect)

        if (res is None or (len(res) != bytes_expect)): 
            raise TimeoutError(f"Motor did not respond with {bytes_expect} byte(s), result is '{res}', length is {len(res)}")

        return res
    
    # Writes the current encoder position of the motor into ROM as the initial position.  Attention:
    # 1. This command needs to restart to take effect.
    # 2. This command will write zero point into ROM of the driver, multiple writing will affect the chip life,which is not recommended for frequent use
    def set_zero_cur_position(self):
        data_length = 0x00        
        header_crc  = (CMD_HEADER + CMD_SET_ZERO + self.id + data_length) % 256
        
        snd = bytearray(pack('<BBBBB', CMD_HEADER, CMD_SET_ZERO, self.id, data_length, header_crc))
        self.serial_port.write(snd)        
        res = self.__read_response(26)      #wait 26 bytes

    def _build_frame(self, cmd: int, payload: bytes) -> bytes:
        """
        Build a host frame for this motor.

        Frame layout:
        [0]=0x3E (CMD_HEADER)
        [1]=CMD
        [2]=ID (0x00..0xFE, 0xFF=broadcast)
        [3]=DATA_LEN (number of payload bytes)
        [4]=HEADER_CRC = (0x3E + CMD + ID + DATA_LEN) % 256
        [5:5+DATA_LEN]=PAYLOAD (optional)
        [end]=DATA_CRC = sum(PAYLOAD) % 256  (only if DATA_LEN > 0)

        Args:
            cmd: Command byte.
            payload: Payload bytes (may be empty).

        Returns:
            Complete frame bytes ready to send.
        """
        data_len = len(payload)
        header_sum = (CMD_HEADER + cmd + self.id + data_len) & 0xFF
        if data_len:
            data_sum = sum(payload) & 0xFF
            return struct.pack("<BBBBB", CMD_HEADER, cmd, self.id, data_len, header_sum) + payload + struct.pack("<B", data_sum)
        else:
            return struct.pack("<BBBBB", CMD_HEADER, cmd, self.id, data_len, header_sum)

    def _send(self, cmd: int, payload: bytes, expect_len: int | None = None) -> bytes:
        """
        Send a command and receive a full reply (dynamic length).
        If expect_len is provided, only warn on mismatch.
        """
        assert self.serial_port is not None, "Serial port required for real device"
        frame = self._build_frame(cmd, payload)
        self.serial_port.write(frame)
        res = self._recv_frame()
        if expect_len is not None and len(res) != expect_len:
            print(f"[warn] expected {expect_len} bytes, got {len(res)} (cmd=0x{cmd:02X})")
        return res


    def _recv_frame(self) -> bytes:
        """Read one full reply frame using header's data_len."""
        hdr = self.__read_response(5)
        data_len = hdr[3]
        if data_len:
            rest = self.__read_response(data_len + 1)  # payload + data_crc
            return hdr + rest
        return hdr  # no payload, no data_crc


    # ---------------- Power / Stop ----------------

    def motor_off(self):
        """
        Disable the motor driver (phases off).

        Protocol:
            CMD 0x80, no payload. Device replies with a 5-byte echo header.

        Returns:
            Raw 5-byte reply.
        """
        return self._send(0x80, b"", 5)

    def motor_on(self):
        """
        Enable the motor driver (phases on; allows motion).

        Protocol:
            CMD 0x88, no payload. Device replies with a 5-byte echo header.

        Returns:
            Raw 5-byte reply.
        """
        return self._send(0x88, b"", 5)

    def motor_stop(self):
        """
        Immediate driver-side stop without disabling power.

        Protocol:
            CMD 0x81, no payload. Device replies with a 5-byte echo header.

        Returns:
            Raw 5-byte reply.
        """
        return self._send(0x81, b"", 5)

    # ---------------- State / Errors ----------------

    def read_state1_and_error(self):
        """
        Read basic state (temperature, voltage) and error bits.

        Protocol:
            CMD 0x9A, no payload.
            Reply: 5 + 7 data + 1 checksum = 13 bytes.
            Data[0]  : int8  temperature (°C)
            Data[1:3]: uint16 bus voltage in centivolts (V * 100)
            Data[3:5]: reserved
            Data[5]  : motor state (0x00 ON, 0x10 OFF)
            Data[6]  : error bitmask

        Returns:
            dict with keys:
                temperature_C (int),
                voltage_V (float),
                motor_on (bool),
                error_bits (int)
        """
        res = self._send(0x9A, b"", 13)
        data = res[5:12]
        temp = struct.unpack("<b", data[0:1])[0]
        voltage = struct.unpack("<H", data[1:3])[0] / 100.0
        motor_state = data[5]
        error_state = data[6]
        return {
            "temperature_C": temp,
            "voltage_V": voltage,
            "motor_on": (motor_state == 0x00),
            "error_bits": error_state
        }

    def clear_error(self):
        """
        Clear driver error status.

        Protocol:
            CMD 0x9B, no payload.
            Reply: same length/format as state1 (13 bytes).

        Returns:
            dict identical to `read_state1_and_error()` with the post-clear state.
        """
        self._send(0x9B, b"", 13)
        return self.read_state1_and_error()

    def read_state2(self):
        """
        Read temperature, iq/power surrogate, speed (dps), and encoder.

        Protocol:
            CMD 0x9C, no payload.
            Reply: 13 bytes total.
            Data[0]  : int8  temperature (°C)
            Data[1:3]: int16 iq/power reading (device-specific)
            Data[3:5]: int16 speed in deg/s (1 dps/LSB)
            Data[5:7]: uint16 encoder counts

        Returns:
            dict with keys:
                temperature_C (int),
                iq_or_power (int),
                speed_dps (int),
                encoder (int)
        """
        res = self._send(0x9C, b"", 13)
        d = res[5:12]
        temp = struct.unpack("<b", d[0:1])[0]
        iq_or_power = struct.unpack("<h", d[1:3])[0]
        speed_dps = struct.unpack("<h", d[3:5])[0]
        encoder = struct.unpack("<H", d[5:7])[0]
        return {"temperature_C": temp, "iq_or_power": iq_or_power, "speed_dps": speed_dps, "encoder": encoder}

    def read_state3(self):
        """
        Read temperature and per-phase currents (A/B/C). (MF/MG series)

        Protocol:
            CMD 0x9D, no payload.
            Reply: 13 bytes total.
            Data[0]  : int8  temperature (°C)
            Data[1:3]: int16 phase A current (1/64 A per LSB)
            Data[3:5]: int16 phase B current
            Data[5:7]: int16 phase C current

        Returns:
            dict with temperature_C (int), phase_A (float), phase_B (float), phase_C (float)
        """
        res = self._send(0x9D, b"", 13)
        d = res[5:12]
        temp = struct.unpack("<b", d[0:1])[0]
        iA = struct.unpack("<h", d[1:3])[0] / 64.0
        iB = struct.unpack("<h", d[3:5])[0] / 64.0
        iC = struct.unpack("<h", d[5:7])[0] / 64.0
        return {"temperature_C": temp, "phase_A": iA, "phase_B": iB, "phase_C": iC}

    # ---------------- Brake ----------------

    def brake_set(self, enable: bool):
        """
        Set brake state.

        Args:
            enable: True to release the brake (motor free to move),
                    False to engage/hold the brake.

        Protocol:
            CMD 0x8C, 1-byte payload:
                0x01 = release (enable True), 0x00 = hold (enable False)
            Reply: 7 bytes (5 header + 1 data echo + 1 checksum).

        Returns:
            Raw 7-byte reply.
        """
        val = b"\x01" if enable else b"\x00"
        return self._send(0x8C, val, 7)

    def brake_read(self) -> bool:
        """
        Read current brake state.

        Protocol:
            CMD 0x8C, payload 0x10 (read request).
            Reply: 7 bytes; DATA[0] nonzero means brake released.

        Returns:
            True if brake is released, False if engaged/holding.
        """
        res = self._send(0x8C, b"\x10", 7)
        state = res[5]
        return bool(state)

    # ---------------- Control Modes ----------------

    def open_loop_power(self, power_ctrl: int):
        """
        Open-loop power/iq drive.

        Args:
            power_ctrl: int16 in device-defined range (e.g., [-850, 850]).

        Behavior:
            Sends CMD 0xA0 with int16 payload, then reads state2 snapshot.

        Returns:
            dict from `read_state2()`.
        """
        payload = struct.pack("<h", int(power_ctrl))
        self._send(0xA0, payload, 8)  # short ack
        return self.read_state2()

    def torque_control(self, iq_ctrl: int):
        """
        Closed-loop torque (iq) control.

        Args:
            iq_ctrl: int16 iq command (device-defined scale).

        Returns:
            dict from `read_state2()`.
        """
        payload = struct.pack("<h", int(iq_ctrl))
        self._send(0xA1, payload, 8)
        return self.read_state2()

    def speed_control(self, speed_dps: float):
        """
        Closed-loop speed control in deg/s.

        Args:
            speed_dps: target speed in deg/s (converted to 0.01 dps/LSB for the wire).

        Returns:
            dict from `read_state2()`.
        """
        val = int(round(speed_dps * 100))
        payload = struct.pack("<i", val)
        self._send(0xA2, payload, 10)
        return self.read_state2()

    def abs_multi_loop_angle_cmd1(self, angle_deg: float):
        """
        Absolute multi-turn position command (Command 1 flavor).

        Args:
            angle_deg: absolute multi-turn angle in degrees (0.01°/LSB on wire).

        Returns:
            dict from `read_state2()`.
        """
        val = int(round(angle_deg * 100))
        payload = struct.pack("<q", val)  # int64
        self._send(0xA3, payload, 13)
        return self.read_state2()

    def abs_single_loop_angle_cmd1(self, angle_deg: float, cw: bool):
        """
        Absolute single-turn position (0..360°) with explicit direction.

        Args:
            angle_deg: angle in [0, 360) degrees (wrapped and quantized to 0.01°).
            cw: True for clockwise, False for counter-clockwise.

        Notes:
            Wire format uses u16 ticks (0..35999) and a spinDirection byte.

        Returns:
            dict from `read_state2()`.
        """
        angle_ticks = int(round(angle_deg * 100)) % 36000
        spin = 0x00 if cw else 0x01
        payload = struct.pack("<BHB", spin, angle_ticks, 0x00)
        self._send(0xA5, payload, 10)
        return self.read_state2()

    def inc_angle_cmd1(self, delta_deg: float):
        """
        Incremental (relative) position step.

        Args:
            delta_deg: signed relative angle in degrees (0.01°/LSB on wire).

        Returns:
            dict from `read_state2()`.
        """
        val = int(round(delta_deg * 100))
        payload = struct.pack("<i", val)
        self._send(0xA7, payload, 10)
        return self.read_state2()

    # ---------------- Encoder / Angles / Zeros ----------------

    def read_encoder(self):
        """
        Read encoder registers.

        Protocol:
            CMD 0x90, no payload.
            Reply: 12 bytes total; DATA = [encoder:uint16, raw:uint16, offset:uint16] plus padding/flags per model.

        Returns:
            dict with keys: encoder, raw, offset (ints).
        """
        res = self._send(0x90, b"", 12)
        d = res[5:11]
        encoder, raw, offset = struct.unpack("<HHH", d[0:6])
        return {"encoder": encoder, "raw": raw, "offset": offset}

    def get_multi_loop_angle(self):
        """
        Read current absolute multi-turn angle (degrees).

        Protocol:
            CMD 0x92, no payload.
            Reply: 14 bytes total; DATA=int64 ticks at 0.01°/LSB.

        Returns:
            float degrees (can be unbounded across multiple turns).
        """
        res = self._send(0x92, b"", 14)
        angle_ticks = struct.unpack("<q", res[5:13])[0]
        self.__cur_multi_loop_angle = angle_ticks / 100.0
        return self.__cur_multi_loop_angle

    def get_single_loop_angle(self):
        """
        Read current single-turn angle (degrees in [0, 360)).

        Protocol:
            CMD 0x94, no payload.
            Reply: 10 bytes total; DATA=uint32 ticks at 0.01°/LSB.

        Returns:
            float degrees in [0, 360).
        """
        res = self._send(0x94, b"", 10)
        circle = struct.unpack("<I", res[5:9])[0]
        self.__cur_single_loop_angle = circle / 100.0
        return self.__cur_single_loop_angle

    def clear_multi_loop(self):
        """
        Clear the multi-turn accumulator (software counter).

        Protocol:
            CMD 0x93, no payload.
            Reply: 5-byte echo header.

        Returns:
            Raw 5-byte reply.
        """
        return self._send(0x93, b"", 5)

    def set_zero_cur_position_ram(self):
        """
        Set the current mechanical position as zero (RAM only).

        Notes:
            Volatile: takes effect immediately but is cleared on power cycle.
            Also clears multi-turn information.

        Protocol:
            CMD 0x95, no payload. Reply: 5-byte echo.
        """
        return self._send(0x95, b"", 5)

    # (Your existing ROM zero method)
    def set_zero_cur_position(self):
        """
        Persist the current position as zero (writes ROM).

        Warnings:
            - Takes effect after reboot.
            - ROM writes have limited endurance: avoid frequent use.

        Protocol:
            CMD 0x19, no payload. Device replies with a longer status frame (26 bytes).
        """
        data_length = 0x00
        header_crc  = (CMD_HEADER + CMD_SET_ZERO + self.id + data_length) % 256
        snd = bytearray(pack('<BBBBB', CMD_HEADER, CMD_SET_ZERO, self.id, data_length, header_crc))
        self.serial_port.write(snd)
        res = self.__read_response(26)
        return res

    # ---------------- PID & Identification ----------------

    def pid_read(self, param_id: int):
        """
        Read a 6-byte PID/config block from RAM for the given parameter ID.

        Args:
            param_id: Parameter selector (see device table).

        Protocol:
            CMD 0x40, payload = [param_id, 0x00].
            Reply: 13 bytes; first data byte echoes param_id followed by 6 bytes.

        Returns:
            (param_id:int, values:bytes[6])
        """
        payload = struct.pack("<BB", param_id & 0xFF, 0x00)
        res = self._send(0x40, payload, 13)
        d = res[5:12]
        pid = d[0]
        vals = d[1:]
        return pid, vals

    def pid_write_ram(self, param_id: int, six_bytes: bytes):
        """
        Write a 6-byte PID/config block to RAM (volatile).

        Args:
            param_id: Parameter selector.
            six_bytes: Exactly 6 bytes to write.

        Returns:
            True if ACK indicates success, False otherwise.
        """
        assert len(six_bytes) == 6
        payload = struct.pack("<B", param_id & 0xFF) + six_bytes
        res = self._send(0x42, payload, 8)
        ok = (res[5] == (param_id & 0xFF) and res[6] == 0x00)
        return ok

    def pid_write_rom(self, param_id: int, six_bytes: bytes):
        """
        Write a 6-byte PID/config block to ROM (persistent).

        Args:
            param_id: Parameter selector.
            six_bytes: Exactly 6 bytes to write.

        Returns:
            True if ACK indicates success, False otherwise.
        """
        assert len(six_bytes) == 6
        payload = struct.pack("<B", param_id & 0xFF) + six_bytes
        res = self._send(0x44, payload, 8)
        ok = (res[5] == (param_id & 0xFF) and res[6] == 0x00)
        return ok

    def read_driver_motor_info(self):
        """
        Read identification strings and versions.

        Protocol:
            CMD 0x12, no payload.
            Reply: 64 bytes total with a 58-byte product info structure:
                - driver_name[20], motor_name[20], motor_id[12],
                - hardware_version:uint16 /10,
                - motor_version:uint16 /10,
                - firmware_version:uint16 /10.

        Returns:
            dict with driver_name, motor_name, motor_id, hardware_version,
            motor_version, firmware_version.
        """
        res = self._send(0x12, b"", 64)
        d = res[5:63]
        driver_name = d[0:20].split(b"\x00",1)[0].decode(errors="ignore")
        motor_name  = d[20:40].split(b"\x00",1)[0].decode(errors="ignore")
        motor_id    = d[40:52].decode(errors="ignore")
        hw_ver, m_ver, fw_ver = struct.unpack("<HHH", d[52:58])
        return {
            "driver_name": driver_name,
            "motor_name": motor_name,
            "motor_id": motor_id,
            "hardware_version": hw_ver / 10.0,
            "motor_version": m_ver / 10.0,
            "firmware_version": fw_ver / 10.0,
        }

        # ---------- Generic frame parsing / verification ----------

    def _parse_reply(self, res: bytes) -> dict:
        """
        Parse a device reply frame and validate checksums.

        Reply layout:
        [0]=0x3E, [1]=CMD, [2]=ID, [3]=DATA_LEN, [4]=HEADER_CRC,
        [5:5+DATA_LEN]=DATA, [5+DATA_LEN]=DATA_CRC (present iff DATA_LEN>0)

        Returns:
            dict with keys:
            cmd (int), id (int), data_len (int), data (bytes),
            header_crc_ok (bool), data_crc_ok (bool)
        """
        if len(res) < 5:
            return {"cmd": None, "id": None, "data_len": 0, "data": b"", "header_crc_ok": False, "data_crc_ok": False}

        cmd = res[1]
        dev_id = res[2]
        data_len = res[3]
        hdr_crc = res[4]
        header_crc_ok = ((CMD_HEADER + cmd + dev_id + data_len) & 0xFF) == hdr_crc

        if data_len == 0:
            return {
                "cmd": cmd, "id": dev_id, "data_len": 0, "data": b"",
                "header_crc_ok": header_crc_ok, "data_crc_ok": True
            }

        if len(res) != 5 + data_len + 1:
            # length mismatch -> cannot check data CRC
            return {
                "cmd": cmd, "id": dev_id, "data_len": data_len, "data": res[5:5+data_len],
                "header_crc_ok": header_crc_ok, "data_crc_ok": False
            }

        data = res[5:5+data_len]
        data_crc = res[5+data_len]
        data_crc_ok = ((sum(data) & 0xFF) == data_crc)
        return {
            "cmd": cmd, "id": dev_id, "data_len": data_len, "data": data,
            "header_crc_ok": header_crc_ok, "data_crc_ok": data_crc_ok
        }

    # ---------- PID decode helpers ----------

    @staticmethod
    def _interpret_six_bytes(data6: bytes) -> dict:
        """
        Provide multiple 'views' of a 6-byte PID/config block for quick sanity-checking.
        Useful when documentation for a particular param_id is unclear.
        """
        if len(data6) != 6:
            raise ValueError("data6 must be exactly 6 bytes")

        u8 = list(data6)
        u16 = struct.unpack("<3H", data6)     # three unsigned 16-bit
        s16 = struct.unpack("<3h", data6)     # three signed 16-bit

        # Common fixed-point guesses (adjust to taste when you learn the real scalings):
        # e.g., gains often come as integers, speeds/angles as centi-units.
        view = {
            "u8": u8,
            "u16": {"v0": u16[0], "v1": u16[1], "v2": u16[2]},
            "s16": {"v0": s16[0], "v1": s16[1], "v2": s16[2]},
            "as_float_div10":  [x / 10.0 for x in u16],
            "as_float_div100": [x / 100.0 for x in u16],
            "as_float_div1000":[x / 1000.0 for x in u16],
        }
        return view

    # Optional: known param registry (fill in as you learn your model’s table)
    # Maps param_id -> tuple of ("name", [("field", "fmt", scale), ...])
    # fmt: "u16" or "s16"; scale is a divisor (1, 10, 100, ...)
    _PID_REGISTRY = {
        # EXAMPLES ONLY — replace with your device’s real map when you have it.
        # 0x90: ("speed_loop", [("kp","u16",1), ("ki","u16",1), ("kd","u16",1)]),
        # 0x91: ("pos_loop",   [("kp","u16",1), ("ki","u16",1), ("kd","u16",1)]),
        # 0xA0: ("limits",     [("max_speed_dps","u16",1), ("max_accel_dps2","u16",1), ("iq_limit","u16",1)]),
        # 0x96: ("(unknown_0x96)", [("v0","u16",1), ("v1","u16",1), ("v2","u16",1)]),
    }

    @classmethod
    def _decode_pid_by_registry(cls, param_id: int, data6: bytes) -> dict | None:
        """
        If the param_id exists in _PID_REGISTRY, decode it to named fields using the
        specified formats and scales. Otherwise return None.
        """
        spec = cls._PID_REGISTRY.get(param_id & 0xFF)
        if spec is None:
            return None
        name, fields = spec
        vals = {}
        # Pre-unpack once
        u16 = struct.unpack("<3H", data6)
        s16 = struct.unpack("<3h", data6)
        for i, (field, fmt, scale) in enumerate(fields):
            raw = u16[i] if fmt == "u16" else s16[i]
            vals[field] = raw / float(scale)
        return {"name": name, "fields": vals}

    def pid_read_verbose(self, param_id: int) -> dict:
        """
        Read a PID/config block and return a rich, human-friendly dict with:
        - echo_id: echoed param_id from device
        - raw6: hex string of the six data bytes
        - header_crc_ok / data_crc_ok: checksum results
        - views: multiple interpretations (u8/u16/s16 and common fixed-point scalings)
        - registry_decode (if available): named, scaled fields from _PID_REGISTRY

        This is ideal for debugging when devices reply with zeros or unexpected IDs.
        """
        payload = struct.pack("<BB", param_id & 0xFF, 0x00)
        res = self._send(0x40, payload, 13)
        parsed = self._parse_reply(res)
        info = {
            "request_id": param_id & 0xFF,
            "header_crc_ok": parsed["header_crc_ok"],
            "data_crc_ok": parsed["data_crc_ok"],
            "reply_cmd": parsed["cmd"],
            "reply_id": parsed["id"],
            "echo_id": None,
            "raw6": None,
            "views": None,
            "registry_decode": None,
        }
        if parsed["data_len"] < 7:
            return info

        echo_id = parsed["data"][0]
        data6 = parsed["data"][1:7]
        info["echo_id"] = echo_id
        info["raw6"] = " ".join(f"{b:02X}" for b in data6)
        info["views"] = self._interpret_six_bytes(data6)
        info["registry_decode"] = self._decode_pid_by_registry(echo_id, data6)
        return info

    def move_abs_multi(self, angle_deg: float, speed_dps: float | None = None):
        """
        Absolute multi-turn move. If speed_dps is given -> cmd2 (0xA4); else cmd1 (0xA3).
        Closed-loop on the driver.
        """
        if speed_dps is None:
            return self.abs_multi_loop_angle_cmd1(angle_deg)      # 0xA3 (angle only)
        else:
            # 0xA4 (angle+speed) — you already have this as abs_multi_loop_angle_speed()
            return self.abs_multi_loop_angle_speed(angle_deg, speed_dps)

    def move_abs_single(self, angle_deg: float, cw: bool | None = None, speed_dps: float | None = None):
        """
        Absolute single-turn move (0..360).
        - If speed_dps is None -> cmd1 (0xA5). Requires cw direction (True/False).
        - If speed_dps is given -> cmd2 (0xA6). Direction is chosen internally.
        Closed-loop on the driver.
        """
        if speed_dps is None:
            if cw is None:
                raise ValueError("cmd1 (0xA5) requires cw=True/False")
            return self.abs_single_loop_angle_cmd1(angle_deg, cw)  # 0xA5
        else:
            return self.abs_single_loop_angle_speed(angle_deg, speed_dps, True)  # 0xA6; cw arg is ignored by device

    def move_inc(self, delta_deg: float, speed_dps: float | None = None):
        """
        Incremental move by delta_deg.
        - If speed_dps is None -> cmd1 (0xA7)
        - If speed_dps is given -> cmd2 (0xA8)
        Closed-loop on the driver.
        """
        if speed_dps is None:
            return self.inc_angle_cmd1(delta_deg)                  # 0xA7
        else:
            return self.inc_angle_speed(delta_deg, speed_dps)      # 0xA8




    # Single position closed loop control command 1 Single position closed loop control command 1 
    # Angle 0...359.99 deg 
    # Rotation direction is set by outside
    def abs_single_loop_angle_speed(self, angle: float, speed: float, CW: bool):

        if angle == 360.0: angle = 0

        assert speed > 0, "Speed must be grater than zero"
        assert angle >= 0.0, f"Angle must be grather or equal zero {angle}"
        assert angle < 360, f"Angle must be less 360 deg Angle is {angle}"
        

        if not self.serial_port == None:
            data_length = 0x08
            
            header_crc  = (CMD_HEADER + CMD_ABS_SINGLE_ANGLE_SPEED + self.id + data_length) % 256

            angle       = int(angle * 100)
            speed       = int(speed * 100)    #according documentaton
            
            if  CW: 
                r_dir = 0x00 
            else: 
                r_dir = 0x01

            data        = pack('<BHBi', r_dir, angle, 0x00, speed)
            data_crc    = sum(data) % 256

            snd = bytearray(pack('<BBBBBBHBiB', CMD_HEADER, CMD_ABS_SINGLE_ANGLE_SPEED, self.id, data_length, header_crc, r_dir, angle, 0x00, speed, data_crc))
            
            
            self.serial_port.write(snd)

            res = self.__read_response(13)      #wait 13 bytes
        else:  #if no serial port than simulate
            res = 1
            self.__cur_single_loop_angle = angle

        return res
    # motor rotation direction is determined by the difference between the target position and the current position
    def abs_multi_loop_angle_speed(self, angle: float, speed: float):
        assert speed > 0, "Speed must be grater than zero"
        
        if not self.serial_port == None:
            #if not self.CW:  angle = -1 * angle
            
            data_length = 0x0C
            
            header_crc  = (CMD_HEADER + CMD_ABS_MULTI_LOOP_ANGLE_SPEED + self.id + data_length) % 256

            angle       = int(angle * 100)
            speed       = int(speed * 100)    #according documentaton

            data        = pack('<qi', angle, speed)
            data_crc    = sum(data) % 256

            snd = bytearray(pack('<BBBBBqiB', CMD_HEADER, CMD_ABS_MULTI_LOOP_ANGLE_SPEED, self.id, data_length, header_crc, angle, speed, data_crc))
            
            self.serial_port.write(snd)

            res = self.__read_response(13)      #wait 13 bytes
        else:  #if no serial port than simulate
            res = 1
            self.__cur_multi_loop_angle = angle

        return res

    def inc_angle_speed(self, angle: float, speed: float):
        assert speed > 0, "Speed must be grater than zero"

        if not self.serial_port == None:
            data_length = 0x08

            #if not self.CW:  angle = -1 * angle
            
            header_crc  = (CMD_HEADER + CMD_INC_ANGLE_SPEED + self.id + data_length) % 256

            angle       = int(angle * 100)
            speed       = int(speed * 100)    #according documentaton

            data        = pack('<ii', angle, speed)
            data_crc    = sum(data) % 256

            snd = bytearray(pack('<BBBBBiiB', CMD_HEADER, CMD_INC_ANGLE_SPEED, self.id, data_length, header_crc, angle, speed, data_crc))
            
            print(" ".join(map(lambda b: format(b, "02x"), snd)))
            self.serial_port.write(snd)

            res = self.__read_response(13)      #wait 13 bytes
        else:  #if no serial port than simulate
            res = 1
            self.__cur_multi_loop_angle = self.__cur_multi_loop_angle + angle
        return res

    # 0 ... 365.99 deg
    def get_single_loop_angle(self):
        if self.serial_port == None:
            pass
        else:        
            header_crc = (CMD_HEADER + CMD_ASK_SINGLE_LOOP_ANGLE + self.id + 0x00) % 256
            r = bytearray(pack('BBBBB', CMD_HEADER, CMD_ASK_SINGLE_LOOP_ANGLE, self.id, 0x00, header_crc))

            self.serial_port.write(r)

            #GET RESPONCE FROM MOTOR
            #TODO add motor ID check

            time.sleep(.01)  #give the serial port sometime to receive the data
            res = self.__read_response(8)

            b = bytearray()
            b.append(res[5])
            b.append(res[6])
            
            angle = unpack("H", b)

            self.__cur_single_loop_angle = float(angle[0]/100)

        return self.__cur_single_loop_angle

    # 0 ... INF deg
    def get_multi_loop_angle(self):
        if self.serial_port == None:
            pass
        else:

            header_crc = (CMD_HEADER + CMD_ASK_MULTI_LOOP_ANGLE + self.id + 0x00) % 256
            r = bytearray(pack('BBBBB', CMD_HEADER, CMD_ASK_MULTI_LOOP_ANGLE, self.id, 0x00, header_crc))

            self.serial_port.write(r)

            #GET RESPONCE FROM MOTOR
            #TODO add motor ID check
            res = self.__read_response(14)
            
            angle = unpack("HHHH", res[5:13])
            
            #update value beore return
            self.__cur_multi_loop_angle = float(angle[0]/100)
       

        return self.__cur_multi_loop_angle

    #blocks code execution till motor stop (angle do not change because of ANY reason)
    #tolerance to detect angle similarity,  request_period to reduce ammout of requests
    def wait_stop(self, request_period: float = 0.1, timeout: float = 15):
        prev_value  = 9999999
        start       = time.time()

        assert request_period   >= 0.01
        assert timeout          >= 0.1, "Timeout must be > 0.1 sec"

        while (time.time() - start < timeout):
            cur_multi_loop_angle   = self.get_multi_loop_angle()
            delta                  = abs(prev_value - cur_multi_loop_angle)
            
            if (delta < self.tolerance): return

            prev_value = cur_multi_loop_angle
            time.sleep(request_period)

        raise TimeoutError("Motor does not stopped in defined time")

    '''
    ------- SIMULATION FUNCTIONS -------  USE FOR simulation and angles <-> coordinates conversion
    See https://www.bsuir.by/m/12_113415_1_70397.pdf
    https://studref.com/472293/tehnika/matrichnye_metody_preobrazovaniya_koordinat_robototehnike?
    '''
    
    def T(self, angle_deg: float) -> np.array:
        angle_deg = math.radians(angle_deg)

        if self.sim_CW:   k = 1
        else:             k = -1
        
        #Generate translation and rotation matrix T        

        
        Ts = np.array([   [1, 0,  0,  self.sim_shifts[0]],          #shift matrix
                          [0, 1,  0,  self.sim_shifts[1]],
                          [0, 0,  1,  self.sim_shifts[2]],
                          [0, 0,  0,  1]  ])
        
        sine = math.sin(k*angle_deg)
        cosine = math.cos(k*angle_deg)
               
        if self.sim_rot_plane == "XY":
            Ta = np.array([ [cosine , 0 , -sine , 0],       #Rotation affects X axis
                            [-sine, 0 ,  cosine , 0],       #Rotation affects Y axis (ie XY plane)
                            [0    , 0 , 1 , 0],                            
                            [0    , 0 , 0 , 1]    ])  #Scale always 1
        elif self.sim_rot_plane == "XZ":
            Ta = np.array([ [cosine , 0 , -sine , 0],       #Rotation affects X axis
                            [0      , 1 , 0     , 0],
                            [-sine  , 0 , cosine , 0],       #Rotation affects Z axis (ie XZ plane)
                            [0      , 0 , 0      , 1]    ])
        elif self.sim_rot_plane == "YZ":
            Ta = np.array([ [1, 0 , 0 , 0],       #X axis (no changes because of angle)
                            [0, cosine , -sine  , 0],       #Rotation in YZ plane
                            [0, sine ,  cosine , 0],
                            [0, 0 , 0  , 1]    ])            
        
        else:
            raise Exception('Wrong rotation plane. It must be XY, XZ or YZ')
        
        return np.matmul(Ta, Ts)         

class Robot(Serializer):
    motors = list()
    __port: serial.Serial = None    #if sumulation, serial port is not assigned
    coords = "NO"

    def __init__(self, port_name: str) -> None:
        try:
            self.__port = serial.Serial(
                port        =   f"/dev/{port_name}",
                baudrate    =   115200,
                parity      =   serial.PARITY_NONE,
                stopbits    =   serial.STOPBITS_ONE,
                bytesize    =   serial.EIGHTBITS,
                #timeout=1
            )

        except Exception as e:
            print ("Error open serial port: " + str(e))

    def add_motor(self, 
                    id: hex, tolerance: float, 
                    name: str, CW: bool, zero_angle: float, sim_shifts: list, 
                    sim_rot_plane: str) -> Motor :

        new_motor = Motor(id, self.__port, tolerance, name, CW, zero_angle, sim_shifts, sim_rot_plane)
        self.motors.append(new_motor)
        return new_motor

    @property
    def motors_count (self):
        return len(self.__motors)

    def goto_zero(self, speed: float = 36000):
        for motor in self.motors:
            motor.abs_multi_loop_angle_speed(motor.zero_angle, speed)
        self.wait_stop()

    # Will rotate each motor to reach angle with multiturn (multiloop). No direction selection. I.e 1 deg -> 355deg will run whole loop
    def goto_abs_multi_loop_angles_speeds(self, angles: list, speeds: list):

        assert len(angles) == len(speeds) == len(self.motors), "Ammout of motors, speeds and angles must be same"

        for motor, angle, speed in zip(self.motors, angles, speeds):
            try:
                angle = float(angle)
                speed = float(speed)
            except ValueError as e:
                print("Error in robot.goto_abs_multi_loop_angles_speeds", str(e))
                exit()
            motor.abs_multi_loop_angle_speed(angle, speed)
        self.wait_stop()

    def goto_abs_single_loop_angles_speeds(self, angles: list, speeds: list, dirs: list):
        assert len(angles) == len(speeds) == len(dirs) == len(self.motors), "Ammout of motors, speeds and angles and directions must be same"

        for motor, angle, speed, dir in zip(self.motors, angles, speeds, dirs):
            try:
                angle = float(angle)
                speed = float(speed)
                dir   = bool(dir)
            except ValueError as e:
                print("Error in robot.goto_abs_single_loop_angles_speeds", str(e))
                exit()
            motor.abs_single_loop_angle_speed(angle, speed, dir)
        self.wait_stop()

    #blocks code execution till ALL motors stop (angle do not change because of ANY reason)
    #tolerance to detect angle similarity,  request_period to reduce ammout of requests
    def wait_stop(self, request_period: float = 0.1, motor_timeout: float = 20):
        for motor in self.motors: motor.wait_stop(request_period, motor_timeout)

    def get_multi_loop_angles(self):
        results = list()

        for motor in self.motors: results.append(motor.get_multi_loop_angle())
        
        return results

    def get_single_loop_angles(self):
        results = list()

        for motor in self.motors: results.append(motor.get_single_loop_angle())
        
        return results        

    # Calculates coordinates usng angles
    def sim_angles_to_coords(self, angles: np.array) -> np.array:
        T       = np.identity(4)
        result  = list()
        P       = list()

        for motor, angle in zip(self.motors, angles):
            try:
                angle   = float(angle)
                Tm      = motor.T(angle)
                T       = np.matmul(T, Tm)
                P.append([0]) # build 1 coloum matrix
            except ValueError as e:
                print("Error in sim_angles_to_coords", str(e))
                exit()

        P.append([1]) # end of creating matrix 1 coloumn. Last one is scale factor

        R = np.dot(T, np.array(P))

        for r in R[:-1]: result.append(r[0])

        return np.array(result)
    
    # Calculates chains angles using from target XYZ coordinates
    #@time_of_function
    def sim_coords_to_angles(self, target: np.array, guess: np.array): # -> np.array, np.array:
        def is_dir_cw(s_angle, e_angle):
            delta = e_angle - s_angle
            if delta < 0: delta = 360 + delta
            return (True if (delta < 180) else False)

        def jacobian(f, x: np.array, h = 0.01):
            n   = len(x)
            Jac = np.zeros([n,n])
            f0  = f(x)
            

            for i in range(0, n, 1):
                tt      = x[i]
                x[i]    = tt + h

                f1      = f(x)
                x[i]    = tt
                Jac [:,i] = (f1 - f0)/h
            return Jac, f0
        
        def newton(f, x: np.array, tol=1.0e-2, h = 0.002):
            iterMax = 350
            for i in range(iterMax):
                Jac, fO = jacobian(f, x, h)
                err = math.sqrt(np.dot(fO, fO) / len(x))

                if err < tol:   return x, i

                dx = np.linalg.solve(Jac, fO)
                x = x - dx
            raise ("Too many iterations for the Newton method")
        
        def f(x: np.array):
            f = np.zeros([3])
            r = self.sim_angles_to_coords([x[0], x[1], x[2]])
            f = r - target

            return f

        
        #guess is angles
        r, iter = newton(f, guess)
         
        # TODO: this area sould be reconsidered for linear axises

        dirs = list()
        angles = list()
    
        for a, g in zip(r, guess):
            new_a = a - int(a / 360) * 360  #Reduce angles to range +/- 360 deg or U get > 360 deg    Means remove multiturns
            
            new_a = round(new_a, 4)
            if new_a == 360 : new_a = 0   #fix jams when after round it is 360 deg and passed functions which acceprt up to 359.99

            if new_a < 0: new_a = 360 + new_a   #Make angle positive

            # Directions To make rotations shortest
            # example: was 10 deg. We go to 358 deg => CCW was 10 deg go to 180 => CW
            dirs.append(is_dir_cw(g, new_a))
            angles.append(round(new_a))

        #result and tolerance

        C_XYZ = self.sim_angles_to_coords(angles)
        TOL = [ (C_XYZ[0] - target[0]).round(4), 
                (C_XYZ[1] - target[1]).round(4),
                (C_XYZ[2] - target[2]).round(4) ]
      
        return angles, TOL, dirs

    def get_coords(self) -> np.array:
        angles = self.get_single_loop_angles()
        return self.sim_angles_to_coords(angles)

# MAIN
if __name__ == '__main__':
     
    robot = Robot("ttyUSB0")
    robot.add_motor(0x01, 0.1, "motor1", True, 0.0, [0.0, 0.0, 0.0], "YZ")
    robot.add_motor(0x02, 0.1, "motor2", True, 0.0, [0.0, 0.0, 0.0], "YZ")

    # robot.goto_zero()

    ang = np.linspace(-30.0, 30.0, 2)
    counter = 0
    import time
    while True:
        print(f"angle: {ang[counter]}")
        t0 = time.time()
        for motor in robot.motors:
            print(motor.move_abs_multi(ang[counter]))
        t1 = time.time()
        # print(f"time: {t1 - t0}")
        print(f"hz: {1 / (t1 - t0)}")
        if counter == 1:
            counter -= 1
        else:
            counter += 1



    for i, motor in enumerate(robot.motors):
        print(motor.read_encoder())
    #     if i == 1:
    #         motor.set_zero_cur_position()
    #         print(f"Motor {i} zero angle: {motor.zero_angle}")