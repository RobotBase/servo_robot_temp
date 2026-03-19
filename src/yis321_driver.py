"""
YIS321 IMU 串口驱动 (PC 端)
============================
直接通过 USB-TTL 串口读取 YIS321 IMU 数据，无需 STM32 主控。

协议说明 (来源: yis321_driver.c):
- 帧头: 0x59 0x53
- 帧长: 67 字节 (固定)
- 数据域起始偏移: 5
- 数据域长度: 60 字节
- 校验: CK1(字节累加) + CK2(CK1累加), 位于 byte[65] 和 byte[66]
- 数据类型:
    0x10 - 加速度 (3 x int32, 缩放 1e-6 m/s²)
    0x20 - 角速度 (3 x int32, 缩放 1e-6 rad/s)
    0x40 - 欧拉角 (3 x int32, 缩放 1e-6 度 -> 再转弧度)
    0x41 - 四元数 (4 x float)

用法:
    python src/yis321_driver.py --port COM5 --baud 460800
"""

import argparse
import struct
import time
import math
from dataclasses import dataclass, field
from typing import Optional

import serial


# ======================== 协议常量 ========================

FRAME_HEADER = bytes([0x59, 0x53])
FRAME_SIZE = 67
DATA_FIELD_OFFSET = 5
DATA_FIELD_SIZE = 60

# 数据标识
DATA_ID_ACCEL = 0x10
DATA_ID_GYRO = 0x20
DATA_ID_EULER = 0x40
DATA_ID_QUAT = 0x41


# ======================== 数据结构 ========================

@dataclass
class IMUData:
    """IMU 数据"""
    # 加速度 (m/s²)
    accel: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # 角速度 (rad/s)
    gyro: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # 欧拉角 (rad)
    euler: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # 四元数 (qx, qy, qz, qw)
    quat: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    # 时间戳
    timestamp: float = 0.0

    @property
    def euler_deg(self) -> list:
        """欧拉角 (度)"""
        return [math.degrees(a) for a in self.euler]

    @property
    def pitch(self) -> float:
        """俯仰角 (rad) — euler[0]"""
        return self.euler[0]

    @property
    def roll(self) -> float:
        """横滚角 (rad) — euler[1]"""
        return self.euler[1]

    @property
    def yaw(self) -> float:
        """偏航角 (rad) — euler[2]"""
        return self.euler[2]

    @property
    def pitch_deg(self) -> float:
        return math.degrees(self.euler[0])

    @property
    def roll_deg(self) -> float:
        return math.degrees(self.euler[1])

    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.euler[2])

    def __str__(self):
        ed = self.euler_deg
        return (
            f"加速度: [{self.accel[0]:+8.4f}, {self.accel[1]:+8.4f}, {self.accel[2]:+8.4f}] m/s²  |  "
            f"角速度: [{self.gyro[0]:+8.4f}, {self.gyro[1]:+8.4f}, {self.gyro[2]:+8.4f}] rad/s  |  "
            f"欧拉角: [{ed[0]:+7.2f}, {ed[1]:+7.2f}, {ed[2]:+7.2f}]°"
        )


# ======================== 帧解析 ========================

def verify_checksum(frame: bytes) -> bool:
    """
    校验帧数据的 CK1/CK2。
    CK1 = sum(byte[2] .. byte[64]) & 0xFF
    CK2 = sum(CK1 的累加过程) & 0xFF
    """
    ck1 = 0
    ck2 = 0
    for i in range(63):
        ck1 = (ck1 + frame[2 + i]) & 0xFF
        ck2 = (ck2 + ck1) & 0xFF
    return ck1 == frame[65] and ck2 == frame[66]


def parse_frame(frame: bytes) -> Optional[IMUData]:
    """
    解析一帧 YIS321 数据。
    返回 IMUData，若校验失败返回 None。
    """
    if len(frame) != FRAME_SIZE:
        return None
    if frame[0] != 0x59 or frame[1] != 0x53:
        return None
    if not verify_checksum(frame):
        return None

    data = IMUData(timestamp=time.time())
    data_field = frame[DATA_FIELD_OFFSET:]

    offset = 0
    while offset < DATA_FIELD_SIZE:
        data_id = data_field[offset]
        data_len = data_field[offset + 1]
        content = data_field[offset + 2: offset + 2 + data_len]

        if data_id == DATA_ID_ACCEL and data_len >= 12:
            vals = struct.unpack('<3i', content[:12])
            data.accel = [v * 1e-6 for v in vals]

        elif data_id == DATA_ID_GYRO and data_len >= 12:
            vals = struct.unpack('<3i', content[:12])
            data.gyro = [v * 1e-6 for v in vals]

        elif data_id == DATA_ID_EULER and data_len >= 12:
            vals = struct.unpack('<3i', content[:12])
            data.euler = [v * 1e-6 * (math.pi / 180.0) for v in vals]

        elif data_id == DATA_ID_QUAT and data_len >= 16:
            data.quat = list(struct.unpack('<4f', content[:16]))

        offset += 2 + data_len

    return data


# ======================== YIS321 驱动类 ========================

class YIS321Driver:
    """
    YIS321 IMU 串口驱动。

    用法:
        driver = YIS321Driver("COM5", baudrate=460800)
        driver.open()
        while True:
            imu = driver.read()
            if imu:
                print(imu)
        driver.close()
    """

    def __init__(self, port: str, baudrate: int = 460800, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._buffer = bytearray()

        # 统计
        self.frame_count = 0
        self.error_count = 0
        self.latest_data: Optional[IMUData] = None

    def open(self):
        """打开串口"""
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout
        )
        self._buffer.clear()
        print(f"[YIS321] 串口已打开: {self.port} @ {self.baudrate} bps")

    def close(self):
        """关闭串口"""
        if self._serial and self._serial.is_open:
            self._serial.close()
            print(f"[YIS321] 串口已关闭: {self.port}")

    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def read(self) -> Optional[IMUData]:
        """
        从串口读取并解析一帧 IMU 数据，非阻塞式。
        成功返回 IMUData，无数据或错误返回 None。
        """
        if not self.is_open():
            return None

        available = self._serial.in_waiting
        if available > 0:
            self._buffer.extend(self._serial.read(available))

        while True:
            header_pos = self._buffer.find(FRAME_HEADER)
            if header_pos == -1:
                if len(self._buffer) > 1:
                    self._buffer = self._buffer[-1:]
                return None

            if header_pos > 0:
                self._buffer = self._buffer[header_pos:]

            if len(self._buffer) < FRAME_SIZE:
                return None

            frame = bytes(self._buffer[:FRAME_SIZE])
            self._buffer = self._buffer[FRAME_SIZE:]

            result = parse_frame(frame)
            if result:
                self.frame_count += 1
                self.latest_data = result
                return result
            else:
                self.error_count += 1
                continue

    def read_blocking(self, timeout: float = 5.0) -> Optional[IMUData]:
        """阻塞式读取，直到获取到有效数据或超时"""
        start = time.time()
        while time.time() - start < timeout:
            result = self.read()
            if result:
                return result
            time.sleep(0.001)
        return None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# ======================== 命令行入口 ========================

def main():
    parser = argparse.ArgumentParser(description="YIS321 IMU 串口读取工具")
    parser.add_argument("--port", "-p", type=str, required=True,
                        help="串口号，如 COM5 或 /dev/ttyUSB0")
    parser.add_argument("--baud", "-b", type=int, default=460800,
                        help="波特率 (默认: 460800)")
    parser.add_argument("--rate", "-r", type=float, default=10.0,
                        help="显示刷新率 Hz (默认: 10)")
    parser.add_argument("--raw", action="store_true",
                        help="显示原始数值 (含四元数)")
    args = parser.parse_args()

    print("=" * 80)
    print("  YIS321 IMU 串口读取工具")
    print(f"  串口: {args.port}  波特率: {args.baud}")
    print("  按 Ctrl+C 退出")
    print("=" * 80)

    display_interval = 1.0 / args.rate
    last_display = 0.0

    with YIS321Driver(args.port, args.baud) as driver:
        try:
            while True:
                imu = driver.read()
                now = time.time()

                if imu and (now - last_display) >= display_interval:
                    last_display = now

                    if args.raw:
                        print(f"\n--- 帧 #{driver.frame_count} (错误: {driver.error_count}) ---")
                        print(f"  加速度: {imu.accel}")
                        print(f"  角速度: {imu.gyro}")
                        print(f"  欧拉角: {imu.euler} rad = {imu.euler_deg} °")
                        print(f"  四元数: {imu.quat}")
                    else:
                        stats = f"[#{driver.frame_count} err:{driver.error_count}]"
                        print(f"\r{stats} {imu}", end="", flush=True)

                time.sleep(0.001)

        except KeyboardInterrupt:
            print(f"\n\n停止。共接收 {driver.frame_count} 帧，错误 {driver.error_count} 帧。")


if __name__ == "__main__":
    main()
