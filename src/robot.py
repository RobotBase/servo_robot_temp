"""
robot.py — 高层机器人控制

基于 ServoBus + RobotConfig，提供按关节名操作的高层 API。
自动处理方向映射、偏移校准、安全限位。
"""

import time
from typing import Optional

from .servo_bus import ServoBus
from .robot_config import RobotConfig, ROBOT_CONFIG


class Robot:
    """双足机器人下半身控制器"""

    def __init__(
        self,
        port: str,
        config: RobotConfig = ROBOT_CONFIG,
        baud: int = 115200,
    ):
        """
        Args:
            port: 串口名 (如 'COM3')
            config: 机器人配置 (默认使用全局配置)
            baud: 波特率
        """
        self.config = config
        self.bus = ServoBus(port, baud)

    # ── 连接管理 ──────────────────────────────────────────────

    def connect(self) -> None:
        """连接串口"""
        self.bus.connect()
        print(f"[Robot] 已连接 {self.bus._port} @ {self.bus._baud} baud")

    def disconnect(self) -> None:
        """断开串口"""
        self.bus.disconnect()
        print("[Robot] 已断开连接")

    # ── 舵机扫描与诊断 ────────────────────────────────────────

    def scan_servos(self) -> list[dict]:
        """扫描配置中的所有舵机，返回在线状态
        
        Returns:
            [{'id': int, 'name': str, 'position': int, 'voltage': int, 
              'temperature': int, 'online': bool}, ...]
        """
        results = []
        for joint in self.config.all_joints:
            pos = self.bus.read_position(joint.servo_id)
            online = pos is not None
            info = {
                "id": joint.servo_id,
                "name": joint.name,
                "online": online,
                "position": pos,
                "voltage": None,
                "temperature": None,
            }
            if online:
                info["voltage"] = self.bus.read_voltage(joint.servo_id)
                info["temperature"] = self.bus.read_temperature(joint.servo_id)
            results.append(info)
        return results

    # ── 零位控制 ──────────────────────────────────────────────

    def go_home(self, time_ms: int = 1500) -> None:
        """所有关节回零位
        
        Args:
            time_ms: 移动时间 (ms)
        """
        moves = [(j.servo_id, j.home_pos) for j in self.config.all_joints]
        self.bus.move_multiple(moves, time_ms)
        print(f"[Robot] 所有关节回零位 ({time_ms}ms)")

    # ── 关节操作 ──────────────────────────────────────────────

    def set_joint(self, name: str, position: int, time_ms: int = 500) -> None:
        """按关节名设置位置
        
        自动处理方向映射和偏移。
        
        Args:
            name: 关节名称 (如 "left_knee")
            position: 目标位置 (0~1000)
            time_ms: 移动时间 (ms)
        """
        joint = self.config.get_joint(name)
        if joint is None:
            raise ValueError(f"未知关节: {name}")
        actual_pos = joint.to_servo_pos(position)
        self.bus.move(joint.servo_id, actual_pos, time_ms)

    def set_joints(self, positions: dict[str, int], time_ms: int = 500) -> None:
        """批量设置多个关节位置
        
        Args:
            positions: {关节名: 位置值, ...}
            time_ms: 移动时间 (ms)
        """
        moves = []
        for name, pos in positions.items():
            joint = self.config.get_joint(name)
            if joint is None:
                raise ValueError(f"未知关节: {name}")
            actual_pos = joint.to_servo_pos(pos)
            moves.append((joint.servo_id, actual_pos))
        self.bus.move_multiple(moves, time_ms)

    def set_joints_raw(self, positions: dict[str, int], time_ms: int = 500) -> None:
        """批量设置多个关节的原始舵机位置（不做方向/偏移转换）
        
        Args:
            positions: {关节名: 原始位置值, ...}
            time_ms: 移动时间 (ms)
        """
        moves = []
        for name, pos in positions.items():
            joint = self.config.get_joint(name)
            if joint is None:
                raise ValueError(f"未知关节: {name}")
            moves.append((joint.servo_id, joint.clamp(pos)))
        self.bus.move_multiple(moves, time_ms)

    # ── 位置读取 ──────────────────────────────────────────────

    def get_joint_position(self, name: str) -> Optional[int]:
        """读取单个关节当前位置 (原始值)"""
        joint = self.config.get_joint(name)
        if joint is None:
            raise ValueError(f"未知关节: {name}")
        return self.bus.read_position(joint.servo_id)

    def get_all_positions(self) -> dict[str, Optional[int]]:
        """读取所有关节当前位置 (原始值)
        
        Returns:
            {关节名: 位置值 (或 None), ...}
        """
        positions = {}
        for joint in self.config.all_joints:
            positions[joint.name] = self.bus.read_position(joint.servo_id)
        return positions

    # ── 加载/卸载控制 ─────────────────────────────────────────

    def unload_all(self) -> None:
        """卸载所有舵机（释放扭力，可自由手动摆动）"""
        for joint in self.config.all_joints:
            self.bus.unload(joint.servo_id)
            time.sleep(0.01)
        print("[Robot] 所有舵机已卸载")

    def load_all(self) -> None:
        """加载所有舵机（锁定扭力）"""
        for joint in self.config.all_joints:
            self.bus.load(joint.servo_id)
            time.sleep(0.01)
        print("[Robot] 所有舵机已加载")

    def unload_leg(self, leg_name: str) -> None:
        """卸载指定腿的所有舵机
        
        Args:
            leg_name: "left" 或 "right"
        """
        leg = self.config.left_leg if leg_name == "left" else self.config.right_leg
        for joint in leg.joints:
            self.bus.unload(joint.servo_id)
            time.sleep(0.01)
        print(f"[Robot] {leg_name} 腿已卸载")
