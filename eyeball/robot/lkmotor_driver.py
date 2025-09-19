from array import array
import math
import time, sys, os
from struct import *
import struct
import numpy as np
import serial, json
from concurrent.futures import ThreadPoolExecutor, Future
import threading
from eyeball.robot.robot import Robot, ActionSpec
from dm_env.specs import Array
from typing import Dict, Any
from abc import abstractmethod
from typing import Literal
from typing import Optional

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

    def __init__(self, id: hex, serial_port, tolerance: float, name: str, CW: bool, zero_angle: float, encoder_bits: int = 15, angle_range: Literal["360", "180"] = "360", joint_limits_deg: Optional[tuple[float, float]] = None): # default to 15-bit encoder, can be overridden
        assert id           >= 0
        assert tolerance    >= 0.01
        #assert serial_port  != None    #no check for case of simulation
        assert name         != None
        

        self.serial_port    = serial_port
        self.id             = id
        self.tolerance      = tolerance
        self.name           = name
        self.zero_angle     = zero_angle
        self.angle_range    = angle_range # for returns in "[0, 360]" or "[-180, 180]"

        self.CPR = 2**encoder_bits # counts per revolution
        if joint_limits_deg is not None:
            self.joint_limits_deg = joint_limits_deg
    
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

    def pid_read(self, param_id: int) -> dict:
        """
        Read PID/config parameters for a given ParamID.

        Args:
            param_id: Parameter selector (see datasheet table).

        Returns:
            dict with:
                - 'param_id': echoed param id
                - 'raw': raw 6-byte payload (bytes)
                - 'decoded': structured interpretation (dict), if known
        """

        payload = struct.pack("<BB", param_id, 0x00)
        res = self._send(0x40, payload)

        echoed = res[5]
        raw6 = res[6:12]

        return {
            "param_id": echoed,
            "raw": raw6,
            "decoded": self._decode_pid_block(echoed, raw6),
        }


    def _decode_pid_block(self, param_id: int, raw6: bytes) -> dict | None:
        """
        Decode the 6-byte parameter block according to the datasheet table.
        Returns None if param_id is unknown.
        """
        if param_id == 0x96:  # 150 decimal, Angle PID
            kp, ki, kd = struct.unpack("<HHH", raw6)
            return {"anglePidKp": kp, "anglePidKi": ki, "anglePidKd": kd}
        elif param_id == 0x97:  # Speed PID
            kp, ki, kd = struct.unpack("<HHH", raw6)
            return {"speedPidKp": kp, "speedPidKi": ki, "speedPidKd": kd}
        elif param_id == 0x98:  # Current PID
            kp, ki, kd = struct.unpack("<HHH", raw6)
            return {"currentPidKp": kp, "currentPidKi": ki, "currentPidKd": kd}
        elif param_id == 0x99:  # maxTorqueCurrent (int16)
            (val,) = struct.unpack("<h", raw6[:2])
            return {"maxTorqueCurrent": val}
        elif param_id == 0x9A:  # maxSpeed (int32)
            (val,) = struct.unpack("<i", raw6[:4])
            return {"maxSpeed": val}
        elif param_id == 0x9B:  # low 4 bytes of maxAngle
            (val,) = struct.unpack("<i", raw6[:4])
            return {"maxAngle_low": val}
        elif param_id == 0x9C:  # high 4 bytes of maxAngle
            (val,) = struct.unpack("<i", raw6[:4])
            return {"maxAngle_high": val}
        elif param_id == 0x9D:  # currentRamp
            (val,) = struct.unpack("<h", raw6[:2])
            return {"currentRamp": val}
        elif param_id == 0x9E:  # speedRamp
            (val,) = struct.unpack("<i", raw6[:4])
            return {"speedRamp": val}
        else:
            return None
    
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

    # # 0 ... 365.99 deg
    def get_single_loop_angle(self):
        if self.serial_port is None:
            return None

        header_crc = (CMD_HEADER + CMD_ASK_SINGLE_LOOP_ANGLE + self.id + 0x00) % 256
        req = pack('BBBBB', CMD_HEADER, CMD_ASK_SINGLE_LOOP_ANGLE, self.id, 0x00, header_crc)
        self.serial_port.write(req)

        # Expect 10 bytes: 5-byte header + 4 data + 1 data checksum
        res = self.__read_response(10)

        # (optional) sanity checks:
        # assert res[0] == CMD_HEADER and res[1] == CMD_ASK_SINGLE_LOOP_ANGLE and res[2] == self.id and res[3] == 0x04
        # assert (sum(res[0:4]) & 0xFF) == res[4]
        # assert (sum(res[5:9]) & 0xFF) == res[9]

        circle_raw, = unpack("<I", res[5:9])      # uint32 little-endian
        self.__cur_single_loop_angle = circle_raw * 0.01  # degrees, 0..359.99

        return self.__cur_single_loop_angle

    # # 0 ... INF deg
    def get_multi_loop_angle(self):
        if self.serial_port == None:
            pass
        else:

            header_crc = (CMD_HEADER + CMD_ASK_MULTI_LOOP_ANGLE + self.id + 0x00) % 256
            r = bytearray(pack('BBBBB', CMD_HEADER, CMD_ASK_MULTI_LOOP_ANGLE, self.id, 0x00, header_crc))

            self.serial_port.write(r)

            res = self.__read_response(14)
            
            angle = unpack("<q", res[5:13])
            
            #update value beore return
            self.__cur_multi_loop_angle = float(angle[0]/100)
       

        return self.__cur_multi_loop_angle

        # ---------- Unified helpers ----------

    def _parse_state2_reply(self, res: bytes) -> dict:
        """
        Parse the standard 13-byte telemetry block (as used by 0x9C and most motion acks).
        Returns a dict with consistent keys.
        """
        # Defensive: some cmds (e.g., 0xA5) may return only 10 bytes (5 hdr + 4 data + 1 crc).
        # When 13 bytes are present, DATA is 7 bytes at res[5:12].
        # We'll be tolerant: try to parse the 7-byte layout if available; otherwise fall back to a state read.
        if len(res) >= 12:
            d = res[5:12]
            if len(d) == 7:
                temp        = struct.unpack("<b", d[0:1])[0]
                iq_or_power = struct.unpack("<h", d[1:3])[0]
                speed_dps   = struct.unpack("<h", d[3:5])[0]
                speed_radps = speed_dps * np.pi / 180.0
                encoder     = struct.unpack("<H", d[5:7])[0]
                angle_deg   = encoder / float(self.CPR) * 360.0
                if angle_deg > 180.0 and self.angle_range == "180":
                    angle_deg = angle_deg - 360.0
                
                angle_rad   = angle_deg * np.pi / 180.0
                return {
                    "temperature_C": temp,
                    "iq_or_power": iq_or_power,
                    "speed_dps": speed_dps,
                    "speed_radps": speed_radps,
                    "encoder": encoder,
                    "angle_deg": angle_deg,
                    "angle_rad": angle_rad,
                }

        # Fallback: ask device for state2 explicitly (13B), to keep a unified return.
        return self.read_state2()  # already returns same key names

    def _send_status(self, cmd: int, payload: bytes, expect_len: int | None = 13) -> dict:
        """
        Send a command and parse the resulting telemetry into a consistent dict.
        If the reply length is not the nominal 13B, we still try to parse;
        otherwise we fetch state2() as a fallback.
        """
        res = self._send(cmd, payload, expect_len=expect_len)
        return self._parse_state2_reply(res)

    # ---------- Unified motion commands ----------

    def abs_multi_loop_angle_cmd1(self, angle_deg: float) -> dict:
        """
        A3: Absolute multi-turn position (no speed). Always returns parsed telemetry dict.
        """
        wire_angle = int(round(angle_deg * 100))  # q0.01 deg, int64
        payload = struct.pack("<q", wire_angle)
        return self._send_status(0xA3, payload, expect_len=13)

    def abs_multi_loop_angle_speed(self, angle_deg: float, speed_dps: float) -> dict:
        """
        A4: Absolute multi-turn position with speed. Returns parsed telemetry dict.
        """
        assert speed_dps > 0, "Speed must be greater than zero"
        wire_angle = int(round(angle_deg * 100))  # q0.01 deg
        wire_speed = int(round(speed_dps * 100))  # q0.01 dps
        payload = struct.pack("<qi", wire_angle, wire_speed)
        return self._send_status(0xA4, payload, expect_len=13)

    def abs_single_loop_angle_cmd1(self, angle_deg: float, cw: bool) -> dict:
        """
        A5: Absolute single-turn (0..360) with explicit direction. Returns parsed telemetry dict.
        Note: device returns a shorter frame on some models; we normalize output via _parse_state2_reply().
        """
        angle_ticks = int(round(angle_deg * 100)) % 36000  # 0..35999
        spin = 0x00 if cw else 0x01
        payload = struct.pack("<BHB", spin, angle_ticks, 0x00)
        # Many firmwares reply with 10B here; parser handles it or falls back to state2.
        return self._send_status(0xA5, payload, expect_len=10)

    def abs_single_loop_angle_speed(self, angle_deg: float, speed_dps: float, cw: bool | None = None) -> dict:
        """
        A6: Absolute single-turn with speed. Direction is chosen internally by device; 'cw' is ignored by most models.
        Returns parsed telemetry dict.
        """
        assert speed_dps > 0, "Speed must be greater than zero"
        if angle_deg == 360.0:
            angle_deg = 0.0
        assert 0.0 <= angle_deg < 360.0, f"Angle must be in [0, 360): {angle_deg}"
        angle_ticks = int(round(angle_deg * 100)) % 36000
        wire_speed  = int(round(speed_dps * 100))
        # Some datasheets show a 'spin' byte for A6 but device ignores it; keep a placeholder 0x00.
        payload = struct.pack("<BHBi", 0x00, angle_ticks, 0x00, wire_speed)
        return self._send_status(0xA6, payload, expect_len=13)

    def inc_angle_cmd1(self, delta_deg: float) -> dict:
        """
        A7: Incremental move (no speed). To keep return shape unified, we read state2 after ack if needed.
        """
        wire_delta = int(round(delta_deg * 100))
        payload = struct.pack("<i", wire_delta)
        # Some firmwares ack with 10B; normalize via parser (which will fall back to read_state2()).
        return self._send_status(0xA7, payload, expect_len=10)

    def inc_angle_speed(self, delta_deg: float, speed_dps: float) -> dict:
        """
        A8: Incremental move with speed. Returns parsed telemetry dict.
        """
        assert speed_dps > 0, "Speed must be greater than zero"
        wire_delta = int(round(delta_deg * 100))
        wire_speed = int(round(speed_dps * 100))
        payload = struct.pack("<ii", wire_delta, wire_speed)
        return self._send_status(0xA8, payload, expect_len=13)

    # ---------- High-level shims (kept for compatibility) ----------

    def move_abs_multi(self, angle_deg: float, speed_dps: float | None = None) -> dict:
        if self.joint_limits_deg is not None:
            angle_deg = np.clip(angle_deg, self.joint_limits_deg[0], self.joint_limits_deg[1])
        return self.abs_multi_loop_angle_cmd1(angle_deg) if speed_dps is None \
            else self.abs_multi_loop_angle_speed(angle_deg, speed_dps)

    def move_abs_single(self, angle_deg: float, cw: bool | None = None, speed_dps: float | None = None) -> dict:
        if speed_dps is None:
            if cw is None:
                raise ValueError("cmd1 (0xA5) requires cw=True/False")
            return self.abs_single_loop_angle_cmd1(angle_deg, cw)
        else:
            return self.abs_single_loop_angle_speed(angle_deg, speed_dps, cw)

    def move_inc(self, delta_deg: float, speed_dps: float | None = None) -> dict:
        return self.inc_angle_cmd1(delta_deg) if speed_dps is None \
            else self.inc_angle_speed(delta_deg, speed_dps)


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

        raise TimeoutError("Motor did not stop in the defined time. Is motor clear of obstructions?")

class LKMotorChain(Serializer):
    motors = list()
    __port: serial.Serial = None    #if sumulation, serial port is not assigned

    def __init__(self, port_name: str) -> None:
        try:
            self.__port = serial.Serial(
                port        =   f"/dev/{port_name}",
                baudrate    =   115200,
                parity      =   serial.PARITY_NONE,
                stopbits    =   serial.STOPBITS_ONE,
                bytesize    =   serial.EIGHTBITS,

                timeout=0,                  # non-blocking
                write_timeout=0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                inter_byte_timeout=None,    # avoid per-byte delays
            )

        except Exception as e:
            print ("Error open serial port: " + str(e))

    def add_motor(self, 
                    id: hex, tolerance: float, 
                    name: str, CW: bool, zero_angle: float, encoder_bits: int = 15, angle_range: Literal["360", "180"] = "360", joint_limits_deg: Optional[tuple[float, float]] = None) -> Motor :
        """
        Add a motor to the chain.
        id: hex address of the motor
        tolerance: tolerance for the motor wait_stop function
        name: pretty print name of the motor
        CW: direction of the motor, defaults to True for clockwise, set to False for counterclockwise
        zero_angle: zero angle of the motor
        encoder_bits: number of bits of encoder resolution
        angle_range: 180 for returning angles in range [-180, 180], 360 for [0, 360]
        joint_limits_deg: joint limits in degrees
        """

        new_motor = Motor(id, self.__port, tolerance, name, CW, zero_angle, encoder_bits, angle_range, joint_limits_deg)
        self.motors.append(new_motor)
        return new_motor

    @property
    def motors_count (self):
        return len(self.__motors)

    def goto_zero(self, speed: float = 36000):
        for motor in self.motors:
            motor.abs_multi_loop_angle_speed(motor.zero_angle, speed)
        self.wait_stop()

    def goto_abs_multi_loop_angles_speeds(self, angles: list, speeds: list = None):
        results = list()
        idxs = range(len(self.motors))
        for idx, motor, angle in zip(idxs, self.motors, angles):
            if speeds is None:
                speed = None
            else:
                speed = speeds[idx]
            res = motor.move_abs_multi(angle, speed_dps=speed)
            results.append(res)
        return results

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

# MAIN
if __name__ == '__main__':
     
    robot = LKMotorChain("ttyUSB0")
    robot.add_motor(0x01, tolerance=0.1, name="motor1", CW=True, zero_angle=0.0)
    robot.add_motor(0x02, tolerance=0.1, name="motor2", CW=True, zero_angle=0.0)

    robot.goto_zero()


    ang = np.linspace(-30.0, 30.0, 8)

    counter = 0
    up = True
    import time
    while True:
        print("\n")
        print(f"Commanded angle: {ang[counter]}")
        t0 = time.time()
        res = robot.goto_abs_multi_loop_angles_speeds([ang[counter], ang[counter]])
        print(res)
        t1 = time.time()
        # print(f"time: {t1 - t0}")
        print(f"hz: {1 / (t1 - t0)}")


        if counter == len(ang) - 1 and up:
            up = False
        elif counter == 0 and not up:
            up = True

        if up:
            counter += 1
        else:
            counter -= 1            