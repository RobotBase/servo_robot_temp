"""
servo_bus.py — LOBOT LX 系列串行总线舵机底层驱动

基于 LX 直连协议 (115200 baud, 8N1)，封装帧构建、校验和、收发、互斥锁。
支持 LX-16A / LX-15D / LX-224 舵机。
"""

import threading
import time
from typing import Optional

import serial


class ServoBus:
    """LOBOT LX 系列总线舵机控制器
    
    半双工串行总线驱动，线程安全。
    所有串口操作通过互斥锁保护。
    """

    BROADCAST_ID = 254

    # ── LX 协议指令编号 ──────────────────────────────────────
    CMD_MOVE_TIME_WRITE = 1
    CMD_MOVE_TIME_READ = 2
    CMD_ID_WRITE = 13
    CMD_ID_READ = 14
    CMD_ANGLE_OFFSET_ADJUST = 17
    CMD_ANGLE_OFFSET_WRITE = 18
    CMD_ANGLE_OFFSET_READ = 19
    CMD_ANGLE_LIMIT_WRITE = 20
    CMD_ANGLE_LIMIT_READ = 21
    CMD_VIN_LIMIT_WRITE = 22
    CMD_VIN_LIMIT_READ = 23
    CMD_TEMP_MAX_LIMIT_WRITE = 24
    CMD_TEMP_MAX_LIMIT_READ = 25
    CMD_TEMP_READ = 26
    CMD_VIN_READ = 27
    CMD_POS_READ = 28
    CMD_SERVO_OR_MOTOR_MODE_WRITE = 29
    CMD_SERVO_OR_MOTOR_MODE_READ = 30
    CMD_LOAD_OR_UNLOAD_WRITE = 31
    CMD_LOAD_OR_UNLOAD_READ = 32
    CMD_LED_CTRL_WRITE = 33
    CMD_LED_CTRL_READ = 34
    CMD_LED_ERROR_WRITE = 35
    CMD_LED_ERROR_READ = 36

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        """
        Args:
            port: 串口名 (如 'COM3', '/dev/ttyUSB0')
            baud: 波特率 (默认 115200)
            timeout: 串口读超时 (秒)
        """
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()

    # ── 连接管理 ──────────────────────────────────────────────

    def connect(self) -> None:
        """打开串口连接"""
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(
            port=self._port,
            baudrate=self._baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self._timeout,
        )
        time.sleep(0.1)  # 等待串口稳定

    def disconnect(self) -> None:
        """关闭串口连接"""
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ── 协议帧构建 ────────────────────────────────────────────

    @staticmethod
    def _checksum(servo_id: int, length: int, cmd: int, params: bytes = b"") -> int:
        """计算 LX 协议校验和: (~(ID + LEN + CMD + Σparams)) & 0xFF"""
        s = servo_id + length + cmd
        for b in params:
            s += b
        return (~s) & 0xFF

    @staticmethod
    def _build_cmd(servo_id: int, cmd: int, params: bytes = b"") -> bytes:
        """构建 LX 协议完整帧"""
        length = len(params) + 3
        cs = ServoBus._checksum(servo_id, length, cmd, params)
        buf = bytearray([0x55, 0x55, servo_id, length, cmd])
        buf.extend(params)
        buf.append(cs)
        return bytes(buf)

    # ── 底层收发 ──────────────────────────────────────────────

    def _send(self, servo_id: int, cmd: int, params: bytes = b"") -> None:
        """发送指令帧（不加锁，调用方需持锁）"""
        assert self._ser and self._ser.is_open, "串口未连接"
        frame = self._build_cmd(servo_id, cmd, params)
        self._ser.write(frame)

    def _recv(self, timeout: float = 0.15) -> Optional[bytes]:
        """接收响应帧（不加锁，调用方需持锁）
        
        Returns:
            完整的响应帧字节，或 None（超时）
        """
        assert self._ser, "串口未连接"
        deadline = time.time() + timeout
        buf = bytearray()
        while time.time() < deadline:
            if self._ser.in_waiting > 0:
                buf.extend(self._ser.read(self._ser.in_waiting))
                # 尝试解析帧
                for i in range(len(buf) - 1):
                    if buf[i] == 0x55 and buf[i + 1] == 0x55 and len(buf) >= i + 4:
                        pkt_len = buf[i + 3]
                        total = 3 + pkt_len  # 帧头(2) + ID(1) + LEN 范围内的字节
                        if len(buf) >= i + total:
                            return bytes(buf[i : i + total])
            time.sleep(0.005)
        return None

    def _send_recv(self, servo_id: int, cmd: int, params: bytes = b"") -> Optional[bytes]:
        """发送指令并等待响应（线程安全）"""
        with self._lock:
            self._ser.flushInput()
            self._send(servo_id, cmd, params)
            return self._recv()

    def _send_only(self, servo_id: int, cmd: int, params: bytes = b"") -> None:
        """仅发送指令，不等待响应（线程安全）"""
        with self._lock:
            self._send(servo_id, cmd, params)

    # ── 核心操作 ──────────────────────────────────────────────

    def move(self, servo_id: int, position: int, time_ms: int = 1000) -> None:
        """移动舵机到指定位置
        
        Args:
            servo_id: 舵机 ID (0~253)
            position: 目标位置 (0~1000, 对应 0°~240°)
            time_ms: 移动时间 (0~30000 ms)
        """
        position = max(0, min(1000, position))
        time_ms = max(0, min(30000, time_ms))
        params = position.to_bytes(2, "little") + time_ms.to_bytes(2, "little")
        self._send_only(servo_id, self.CMD_MOVE_TIME_WRITE, params)

    def move_multiple(self, moves: list[tuple[int, int]], time_ms: int = 1000) -> None:
        """同步移动多个舵机
        
        使用快速连续发送实现近似同步。
        
        Args:
            moves: [(servo_id, position), ...] 列表
            time_ms: 移动时间 (所有舵机共用)
        """
        with self._lock:
            for servo_id, position in moves:
                position = max(0, min(1000, position))
                t = max(0, min(30000, time_ms))
                params = position.to_bytes(2, "little") + t.to_bytes(2, "little")
                self._send(servo_id, self.CMD_MOVE_TIME_WRITE, params)

    def read_position(self, servo_id: int) -> Optional[int]:
        """读取舵机当前位置
        
        Returns:
            位置值 (0~1000) 或 None（通信失败）
        """
        resp = self._send_recv(servo_id, self.CMD_POS_READ)
        if resp and len(resp) >= 8:
            pos = resp[5] | (resp[6] << 8)
            return pos - 65536 if pos > 32767 else pos
        return None

    def read_voltage(self, servo_id: int) -> Optional[int]:
        """读取舵机输入电压
        
        Returns:
            电压值 (mV) 或 None
        """
        resp = self._send_recv(servo_id, self.CMD_VIN_READ)
        if resp and len(resp) >= 8:
            return resp[5] | (resp[6] << 8)
        return None

    def read_temperature(self, servo_id: int) -> Optional[int]:
        """读取舵机温度
        
        Returns:
            温度 (°C) 或 None
        """
        resp = self._send_recv(servo_id, self.CMD_TEMP_READ)
        if resp and len(resp) >= 7:
            return resp[5]
        return None

    def read_id(self, servo_id: int = BROADCAST_ID) -> Optional[int]:
        """读取舵机 ID
        
        Args:
            servo_id: 目标 ID (默认 254 广播)
            
        Returns:
            舵机 ID 或 None
        """
        resp = self._send_recv(servo_id, self.CMD_ID_READ)
        if resp and len(resp) >= 7:
            return resp[5]
        return None

    def write_id(self, old_id: int, new_id: int) -> None:
        """修改舵机 ID
        
        ⚠️ 总线上必须仅连接一个舵机！
        
        Args:
            old_id: 当前 ID
            new_id: 新 ID (0~253)
        """
        new_id = max(0, min(253, new_id))
        self._send_only(old_id, self.CMD_ID_WRITE, bytes([new_id]))
        time.sleep(0.1)  # 等待 Flash 写入

    def unload(self, servo_id: int) -> None:
        """卸载舵机（释放扭力，舵机轴可自由旋转）"""
        self._send_only(servo_id, self.CMD_LOAD_OR_UNLOAD_WRITE, bytes([0]))

    def load(self, servo_id: int) -> None:
        """加载舵机（锁定扭力）"""
        self._send_only(servo_id, self.CMD_LOAD_OR_UNLOAD_WRITE, bytes([1]))

    def set_led(self, servo_id: int, on: bool = True) -> None:
        """控制舵机 LED"""
        self._send_only(servo_id, self.CMD_LED_CTRL_WRITE, bytes([0 if on else 1]))

    # ── 扫描 ─────────────────────────────────────────────────

    def scan(self, id_range: range = range(1, 21)) -> list[dict]:
        """扫描在线舵机
        
        Args:
            id_range: 扫描的 ID 范围 (默认 1~20)
            
        Returns:
            在线舵机信息列表 [{'id': int, 'position': int, 'voltage': int, 'temperature': int}, ...]
        """
        found = []
        for sid in id_range:
            pos = self.read_position(sid)
            if pos is not None:
                vin = self.read_voltage(sid)
                temp = self.read_temperature(sid)
                found.append({
                    "id": sid,
                    "position": pos,
                    "voltage": vin,
                    "temperature": temp,
                })
        return found
