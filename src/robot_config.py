"""
robot_config.py — 机器人下半身硬件配置定义

定义关节映射、ID 分配、行程范围、零位、正反向。
所有模块共用 ROBOT_CONFIG 全局实例。
"""

from dataclasses import dataclass, field


@dataclass
class JointConfig:
    """单个关节配置"""

    servo_id: int           # 舵机 ID (0~253)
    name: str               # 关节名称 (如 "left_hip_yaw")
    min_pos: int = 0        # 最小位置限位
    max_pos: int = 1000     # 最大位置限位
    home_pos: int = 500     # 零位（中间位置）
    direction: int = 1      # 方向 (1 = 正向, -1 = 反向/镜像)
    offset: int = 0         # 偏移校准值

    def clamp(self, position: int) -> int:
        """将位置限制在安全范围内"""
        return max(self.min_pos, min(self.max_pos, position))

    def to_servo_pos(self, logical_pos: int) -> int:
        """逻辑位置 → 舵机实际位置（处理方向和偏移）
        
        logical_pos: 以 home_pos 为中心的逻辑位置 (0~1000)
        """
        if self.direction == -1:
            # 反向: 以 home 为中心镜像
            actual = self.home_pos - (logical_pos - self.home_pos) + self.offset
        else:
            actual = logical_pos + self.offset
        return self.clamp(actual)

    def to_logical_pos(self, servo_pos: int) -> int:
        """舵机实际位置 → 逻辑位置（逆运算）"""
        if self.direction == -1:
            logical = self.home_pos - (servo_pos - self.offset - self.home_pos)
        else:
            logical = servo_pos - self.offset
        return logical


@dataclass
class LegConfig:
    """单条腿配置"""

    name: str                       # "left" / "right"
    joints: list[JointConfig] = field(default_factory=list)

    def get_joint(self, joint_name: str) -> JointConfig | None:
        """按关节名查找"""
        for j in self.joints:
            if j.name == joint_name:
                return j
        return None

    @property
    def servo_ids(self) -> list[int]:
        """该腿所有舵机 ID"""
        return [j.servo_id for j in self.joints]


@dataclass
class RobotConfig:
    """完整机器人下半身配置"""

    name: str = "servo_biped"
    left_leg: LegConfig = field(default_factory=lambda: LegConfig("left"))
    right_leg: LegConfig = field(default_factory=lambda: LegConfig("right"))

    @property
    def all_joints(self) -> list[JointConfig]:
        """所有关节列表"""
        return self.left_leg.joints + self.right_leg.joints

    def get_joint(self, name: str) -> JointConfig | None:
        """按名称查找关节"""
        for j in self.all_joints:
            if j.name == name:
                return j
        return None

    @property
    def all_servo_ids(self) -> list[int]:
        """所有舵机 ID"""
        return [j.servo_id for j in self.all_joints]

    @property
    def joint_names(self) -> list[str]:
        """所有关节名称"""
        return [j.name for j in self.all_joints]


# ┌──────────────────────────────────────────────────────────────┐
# │                    默认机器人配置                              │
# │  左腿 ID 1-5 | 右腿 ID 6-10                                  │
# │  每条腿: 髋侧摆 → 髋前摆 → 膝关节 → 踝前摆 → 踝侧摆          │
# └──────────────────────────────────────────────────────────────┘

ROBOT_CONFIG = RobotConfig(
    name="servo_biped_v1",
    left_leg=LegConfig(
        name="left",
        joints=[
            JointConfig(servo_id=6,  name="left_hip_yaw",     home_pos=500, direction=1),
            JointConfig(servo_id=7,  name="left_hip_pitch",   home_pos=500, direction=1),
            JointConfig(servo_id=8,  name="left_knee",        home_pos=500, direction=1),
            JointConfig(servo_id=9,  name="left_ankle_pitch", home_pos=500, direction=1),
            JointConfig(servo_id=10, name="left_ankle_roll",  home_pos=500, direction=1),
        ],
    ),
    right_leg=LegConfig(
        name="right",
        joints=[
            JointConfig(servo_id=1,  name="right_hip_yaw",     home_pos=500, direction=1),
            JointConfig(servo_id=2,  name="right_hip_pitch",   home_pos=500, direction=1),
            JointConfig(servo_id=3,  name="right_knee",        home_pos=500, direction=1),
            JointConfig(servo_id=4,  name="right_ankle_pitch", home_pos=500, direction=1),
            JointConfig(servo_id=5,  name="right_ankle_roll",  home_pos=500, direction=1),
        ],
    ),
)
