"""
walk.py — 基于 footCont 逆运动学的开环步行控制

移植自 servo_robot_learn 项目的 footCont() 逆运动学，
通过 CPG 生成足端轨迹，IK 计算关节角度，驱动舵机行走。

用法:
    python tools/walk.py --port COM11
    python tools/walk.py --port COM11 --steps 10 --period 1.2
    python tools/walk.py --port COM11 --mode stand
    python tools/walk.py --port COM11 --mode test
    python tools/walk.py --port COM11 --mode set_zero
"""

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.robot import Robot


# ╔══════════════════════════════════════════════════════════════╗
# ║                 机器人物理参数 (mm)                           ║
# ╚══════════════════════════════════════════════════════════════╝

class RobotDimensions:
    """机器人腿部连杆尺寸"""
    THIGH_LEN = 65.0          # 大腿长 (K0→H) mm
    SHIN_LEN = 65.0           # 小腿长 (H→A0) mm
    LEG_LEN = 130.0           # 大腿+小腿 = 130mm
    HIP_OFFSET = 40.0         # K1→K0 偏移 mm
    ANKLE_OFFSET = 24.5       # A0→A1 偏移 mm
    LATERAL_OFFSET = 64.5     # 总横向偏移 = 40+24.5 mm
    HEIGHT = 190.0            # 站立时踝→髋高度 mm
    MAX_LEG = 194.5           # 最大腿长 = 40+65+65+24.5 mm
    HIP_SPACING = 21.5        # 髋关节中心到身体中线距离 mm


# ╔══════════════════════════════════════════════════════════════╗
# ║          零位 — 站立时各关节原始舵机位置                        ║
# ╚══════════════════════════════════════════════════════════════╝

DEFAULT_ZERO = {
    "right_hip_yaw": 446,
    "right_hip_pitch": 809,
    "right_knee": 462,
    "right_ankle_pitch": 431,
    "right_ankle_roll": 560,
    "left_hip_yaw": 380,
    "left_hip_pitch": 475,
    "left_knee": 465,
    "left_ankle_pitch": 812,
    "left_ankle_roll": 635,
}


def load_zero_position(filepath: str = None) -> dict[str, int]:
    """从 JSON 文件加载零位"""
    if filepath and os.path.exists(filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        if data.get("keyframes"):
            zero = data["keyframes"][0]["positions"]
            print(f"零位已从 {filepath} 加载")
            return zero
    print("使用默认零位")
    return DEFAULT_ZERO.copy()


def save_zero_position(robot: Robot, filepath: str) -> dict[str, int]:
    positions = robot.get_all_positions()
    missing = [name for name, pos in positions.items() if pos is None]
    if missing:
        raise RuntimeError(f"以下关节读取失败，无法保存零位: {', '.join(missing)}")

    zero = {name: int(pos) for name, pos in positions.items() if pos is not None}
    data = {
        "name": "零位",
        "description": "walk.py set_zero 生成",
        "created_at": datetime.now().astimezone().isoformat(),
        "frame_count": 1,
        "total_duration_ms": 0,
        "keyframes": [
            {
                "timestamp_ms": 0,
                "positions": zero,
            }
        ],
    }
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print(f"新零位已保存: {filepath}")
    return zero


# ╔══════════════════════════════════════════════════════════════╗
# ║             footCont 逆运动学 (移植自 robot_ctrl.c)           ║
# ╚══════════════════════════════════════════════════════════════╝

def foot_cont(x: float, y: float, h: float, dim: RobotDimensions = RobotDimensions):
    """逆运动学: 足端位置 → 5个关节角度 (弧度)

    坐标系: X=前后(前+), Y=左右(右+), Z=上下
    
    Args:
        x: 足端前后偏移 (mm), 前为正
        y: 足端左右偏移 (mm), 右为正
        h: 踝Roll轴到髋Roll轴的垂直高度 (mm)
        dim: 机器人尺寸参数

    Returns:
        (K0, K1, H, A0, A1) 各关节角度 (弧度)
        K0 = 髋关节 pitch
        K1 = 髋关节 roll (yaw)
        H  = 膝关节
        A0 = 踝关节 pitch
        A1 = 踝关节 roll
    """
    # Step 1: 计算 K0→A0 直线距离
    # 正视图: K1→A1 距离 = sqrt(y² + h²)
    # 减去横向偏移 (K1→K0=40mm, A0→A1=24.5mm): K0→A0面 = sqrt(y²+h²) - 64.5
    # 侧视图加入前后: k = sqrt(x² + (K0→A0面)²)
    inner = math.sqrt(y * y + h * h) - dim.LATERAL_OFFSET
    if inner < 0:
        inner = 0  # 安全限制
    k = math.sqrt(x * x + inner * inner)

    # 安全限制: k 不超过大腿+小腿长度
    if k > dim.LEG_LEN - 1:  # 留 1mm 余量避免奇点
        k = dim.LEG_LEN - 1

    # Step 2: 解算关节角度
    # 膝弯曲角 (余弦定理: 大腿=小腿=65mm, 合计=130mm)
    k0 = math.acos(k / dim.LEG_LEN)  # 膝弯曲半角 (rad)

    # 脚前后摆角
    x0 = math.asin(max(-1.0, min(1.0, x / k))) if k > 0.001 else 0.0

    # 横向角
    k1 = math.atan2(y, h) if h > 0.001 else 0.0

    # Step 3: 输出各关节角 (rad)
    K0 = k0 + x0   # 髋关节 pitch = 膝弯曲角 + 摆腿角
    H = k0 * 2.0    # 膝关节 = 2 × 膝弯曲角 (对称展开)
    A0 = k0 - x0    # 踝关节 pitch = 膝弯曲角 - 摆腿角 (保持脚掌水平)
    K1 = k1          # 髋关节 roll
    A1 = -k1         # 踝关节 roll = -髋roll (保持脚掌水平)

    return K0, K1, H, A0, A1


# ╔══════════════════════════════════════════════════════════════╗
# ║            角度→舵机原始值转换                                 ║
# ╚══════════════════════════════════════════════════════════════╝

# LX 舵机: 0~1000 对应 0°~240°, 即 1 unit = 0.24°
SERVO_UNITS_PER_DEG = 1.0 / 0.24  # ≈ 4.167 units/degree

# 站立时的关节角度 (由 footCont(0, 0, HEIGHT) 计算)
_stand_angles = foot_cont(0, 0, RobotDimensions.HEIGHT)
STAND_K0, STAND_K1, STAND_H, STAND_A0, STAND_A1 = _stand_angles

print(f"[IK] 站立角度 (度): K0={math.degrees(STAND_K0):.1f}  "
      f"H={math.degrees(STAND_H):.1f}  A0={math.degrees(STAND_A0):.1f}  "
      f"K1={math.degrees(STAND_K1):.1f}  A1={math.degrees(STAND_A1):.1f}")

# 关节映射: IK输出 → 舵机原始值
# direction: IK角度增大时，舵机值应该增大(+1)还是减小(-1)
# 从手动示教数据推断:
#   right_hip_pitch(K0):  值增大 → 前摆  → dir=+1
#   left_hip_pitch(K0): 值减小 → 前摆  → dir=-1
#   right_knee(H):       值减小 → 弯曲   (462→396)  → dir=-1
#   left_knee(H):      值增大 → 弯曲   (465→558)  → dir=+1
#   right_ankle_pitch(A0): 值减小 → 补偿  → dir=-1
#   left_ankle_pitch(A0): 值增大 → 补偿  → dir=+1

JOINT_MAP = {
    "right": {
        # (零位key, IK分量索引, 方向)
        "hip_yaw":     ("right_hip_yaw",     1, -1),   # K1 (roll)
        "hip_pitch":   ("right_hip_pitch",    0, +1),   # K0 ← 已修正
        "knee":        ("right_knee",         2, -1),   # H
        "ankle_pitch": ("right_ankle_pitch",  3, -1),   # A0 ← 已修正
        "ankle_roll":  ("right_ankle_roll",   4, -1),   # A1
    },
    "left": {
        "hip_yaw":     ("left_hip_yaw",     1, +1),   # K1 (roll)
        "hip_pitch":   ("left_hip_pitch",   0, -1),   # K0 ← 已修正
        "knee":        ("left_knee",        2, +1),   # H
        "ankle_pitch": ("left_ankle_pitch", 3, +1),   # A0 ← 已修正
        "ankle_roll":  ("left_ankle_roll",  4, +1),   # A1
    },
}

# 站立时 5 个分量的角度值 (用于计算 delta)
STAND_ANGLES = [STAND_K0, STAND_K1, STAND_H, STAND_A0, STAND_A1]


def ik_to_servo(
    ik_angles: tuple[float, ...],
    side: str,
    zero: dict[str, int],
    ankle_pitch_limit_deg: float | None = None,
) -> dict[str, int]:
    """将 IK 角度转换为舵机原始位置值

    Args:
        ik_angles: footCont 输出 (K0, K1, H, A0, A1) 弧度
        side: "left" 或 "right"
        zero: 零位 dict

    Returns:
        {关节全名: 原始舵机值} dict
    """
    result = {}
    for joint_short, (zero_key, ik_idx, direction) in JOINT_MAP[side].items():
        # 计算相对于站立姿态的角度差 (弧度)
        delta_rad = ik_angles[ik_idx] - STAND_ANGLES[ik_idx]

        # 弧度 → 度 → 舵机单位
        delta_servo = math.degrees(delta_rad) * SERVO_UNITS_PER_DEG
        if ankle_pitch_limit_deg is not None and joint_short == "ankle_pitch":
            max_delta_servo = ankle_pitch_limit_deg * SERVO_UNITS_PER_DEG
            delta_servo = max(-max_delta_servo, min(max_delta_servo, delta_servo))

        # 应用方向和零位
        joint_name = f"{side}_{joint_short}"
        raw_value = zero[zero_key] + direction * delta_servo
        result[joint_name] = int(max(0, min(1000, raw_value)))

    return result


# ╔══════════════════════════════════════════════════════════════╗
# ║              CPG 步态轨迹生成                                 ║
# ╚══════════════════════════════════════════════════════════════╝

class GaitParams:
    """步态参数"""
    def __init__(self):
        self.period: float = 1.2        # 完整步态周期 (秒)
        self.dt: float = 0.2           # 控制周期 (秒), 50Hz
        self.stride: float = 20.0       # 步幅 (mm), 前后各 ±stride/2
        self.lift_height: float = 26.0  # 抬脚高度 (mm)
        self.sway: float = 11.0         # 横向摆动幅度 (mm)
        self.stance_width: float = 8.0
        self.left_stride_scale: float = 1.18
        self.action_scale: float = 1.0
        self.ankle_pitch_limit_deg: float = 45.0
        self.pre_shift_ratio: float = 0.30
        self.crouch_depth: float = 8.0
        self.weight_shift: float = 10.0
        self.landing_ankle_relax: float = 0.35
        self.yaw_trim: float = 2.0      # 偏航修正(mm): 正值更偏左, 用于纠正向右偏
        self.height: float = RobotDimensions.HEIGHT  # 站立高度


def smoothstep(x: float) -> float:
    u = max(0.0, min(1.0, x))
    return u * u * (3.0 - 2.0 * u)


@dataclass
class ForwardPhase:
    swing_leg: str
    phase: float
    pre_shift_ratio: float
    cycle_index: int


@dataclass
class FootTarget:
    x: float
    y: float
    h: float
    landing_progress: float


@dataclass
class ForwardFramePlan:
    phase: ForwardPhase
    left: FootTarget
    right: FootTarget


def plan_forward_phase(t: float, params: GaitParams) -> ForwardPhase:
    gait_phase = (t % params.period) / params.period
    cycle_index = int(t / params.period) if params.period > 0 else 0
    if gait_phase < 0.5:
        swing_leg = "right"
        phase = gait_phase / 0.5
    else:
        swing_leg = "left"
        phase = (gait_phase - 0.5) / 0.5
    pre_shift_ratio = max(0.05, min(0.7, params.pre_shift_ratio))
    return ForwardPhase(swing_leg=swing_leg, phase=phase, pre_shift_ratio=pre_shift_ratio, cycle_index=cycle_index)


def leg_trajectory(leg: str, phase_info: ForwardPhase, params: GaitParams) -> FootTarget:
    stride = params.stride * params.action_scale
    half_stride = stride / 2.0
    lift_height = params.lift_height * params.action_scale
    pre = phase_info.pre_shift_ratio
    phase = phase_info.phase
    swing_leg = phase_info.swing_leg
    crouch = params.crouch_depth * math.sin(math.pi * phase)
    shift_sign = 1.0 if (swing_leg == "left" and leg == "right") or (swing_leg == "right" and leg == "left") else -1.0
    shift_gain = smoothstep(phase / pre) if phase < pre else 1.0
    y = params.stance_width + shift_sign * params.weight_shift * shift_gain
    x = 0.0
    lift = 0.0
    landing_progress = 0.0

    if leg == swing_leg and phase >= pre:
        swing_phase = (phase - pre) / (1.0 - pre)
        x = -half_stride * math.cos(math.pi * swing_phase)
        lift = lift_height * (math.sin(math.pi * swing_phase) ** 1.5)
        landing_progress = smoothstep((swing_phase - 0.75) / 0.25)
    elif leg != swing_leg:
        if phase < pre:
            x = -half_stride
        else:
            support_phase = (phase - pre) / (1.0 - pre)
            x = -half_stride * math.cos(math.pi * support_phase)

    if leg == "left":
        x *= params.left_stride_scale

    h = params.height - crouch + lift
    return FootTarget(x=x, y=y, h=h, landing_progress=landing_progress)


def build_forward_frame_plan(t: float, params: GaitParams) -> ForwardFramePlan:
    phase_info = plan_forward_phase(t, params)
    left = leg_trajectory("left", phase_info, params)
    right = leg_trajectory("right", phase_info, params)
    return ForwardFramePlan(phase=phase_info, left=left, right=right)


def generate_frame(
    t: float,
    zero: dict[str, int],
    params: GaitParams,
) -> dict[str, int]:
    """生成一帧完整的舵机位置

    Args:
        t: 当前时间 (秒)
        zero: 零位
        params: 步态参数

    Returns:
        {关节名: 原始舵机值}
    """
    plan = build_forward_frame_plan(t, params)
    lx, ly, lh = plan.left.x, plan.left.y, plan.left.h
    rx, ry, rh = plan.right.x, plan.right.y, plan.right.h

    if params.yaw_trim != 0.0:
        lx -= params.yaw_trim
        rx += params.yaw_trim

    # 逆运动学: 足端 → 关节角
    left_angles = foot_cont(lx, ly, lh)
    right_angles = foot_cont(rx, -ry, rh)  # 右腿 y 取反 (镜像)

    # 关节角 → 舵机值
    frame = {}
    left_limit = params.ankle_pitch_limit_deg * (1.0 - params.landing_ankle_relax * plan.left.landing_progress)
    right_limit = params.ankle_pitch_limit_deg * (1.0 - params.landing_ankle_relax * plan.right.landing_progress)
    frame.update(ik_to_servo(left_angles, "left", zero, left_limit))
    frame.update(ik_to_servo(right_angles, "right", zero, right_limit))

    return frame


# ╔══════════════════════════════════════════════════════════════╗
# ║                       主程序                                  ║
# ╚══════════════════════════════════════════════════════════════╝

def do_stand(robot: Robot, zero: dict[str, int]):
    """站立"""
    print("\n进入站立姿态...")
    robot.set_joints_raw(zero, 1500)
    time.sleep(2.0)
    print("站立就绪")


def do_test(robot: Robot, zero: dict[str, int]):
    """IK 测试: 在站立基础上小幅前后摆腿, 验证方向"""
    print("\nIK 验证测试")
    do_stand(robot, zero)
    time.sleep(1)

    dim = RobotDimensions
    test_cases = [
        ("站立",     0,  0,  dim.HEIGHT),
        ("左脚前伸", 20,  0,  dim.HEIGHT),
        ("左脚后摆", -20, 0,  dim.HEIGHT),
        ("左脚外展", 0,  10, dim.HEIGHT),
        ("左脚抬高", 0,  0,  dim.HEIGHT + 10),
    ]

    for name, x, y, h in test_cases:
        print(f"\n  测试: {name}  (x={x}, y={y}, h={h})")
        angles = foot_cont(x, y, h)
        K0, K1, H, A0, A1 = angles
        print(f"    IK输出 (度): K0={math.degrees(K0):.1f}  K1={math.degrees(K1):.1f}  "
              f"H={math.degrees(H):.1f}  A0={math.degrees(A0):.1f}  A1={math.degrees(A1):.1f}")

        frame = zero.copy()
        left_servos = ik_to_servo(angles, "left", zero)
        frame.update(left_servos)
        print(f"    舵机值: hip_pitch={left_servos['left_hip_pitch']}  "
              f"knee={left_servos['left_knee']}  "
              f"ankle_pitch={left_servos['left_ankle_pitch']}")

        robot.set_joints_raw(frame, 800)
        time.sleep(1.2)

    print("\n  回到站立...")
    robot.set_joints_raw(zero, 800)
    time.sleep(1)
    print("IK 测试完成")


def do_walk(robot: Robot, zero: dict[str, int], params: GaitParams, steps: int = 6):
    """开环步行"""
    total_time = steps * params.period / 2.0
    print(f"\n开始行走: {steps} 步, 周期 {params.period}s, 预计 {total_time:.1f}s")
    print(f"   步幅: {params.stride}mm, 抬脚: {params.lift_height}mm, 侧摆: {params.sway}mm")
    print(f"   双脚外扩: {params.stance_width}mm")
    print(f"   左腿步幅系数: {params.left_stride_scale:.2f}, 动作放大: {params.action_scale:.2f}x")
    print(f"   踝前后角度限制: ±{params.ankle_pitch_limit_deg:.1f}°")
    print(f"   偏航修正: {params.yaw_trim}mm")
    print(f"   预移重心占比: {params.pre_shift_ratio:.2f}, 下蹲深度: {params.crouch_depth:.1f}mm")
    print(f"   重心横移: {params.weight_shift:.1f}mm, 落地缓冲: {params.landing_ankle_relax:.2f}")
    print("   正向流程: phase_plan -> leg_target -> yaw_trim -> ik -> servo")
    print(f"   控制频率: {1/params.dt:.0f}Hz\n")

    do_stand(robot, zero)

    move_time_ms = int(params.dt * 1000)
    t = 0.0
    start = time.time()

    try:
        while t < total_time:
            loop_start = time.time()

            frame = generate_frame(t, zero, params)
            robot.set_joints_raw(frame, move_time_ms)

            # 每 0.5s 打印状态
            if int(t / 0.5) != int((t - params.dt) / 0.5):
                step_num = int(t / (params.period / 2.0)) + 1
                print(f"  步 {step_num}/{steps}  t={t:.2f}s  "
                      f"L_hp={frame.get('left_hip_pitch',0)}  "
                      f"R_hp={frame.get('right_hip_pitch',0)}  "
                      f"L_kn={frame.get('left_knee',0)}  "
                      f"R_kn={frame.get('right_knee',0)}")

            t += params.dt
            elapsed = time.time() - loop_start
            if params.dt - elapsed > 0:
                time.sleep(params.dt - elapsed)

    except KeyboardInterrupt:
        print("\n\n中断!")

    print("\n回到站立...")
    robot.set_joints_raw(zero, 1000)
    time.sleep(1.5)
    print(f"行走完成 (耗时 {time.time()-start:.1f}s)")


def main():
    parser = argparse.ArgumentParser(description="footCont IK 开环步行")
    parser.add_argument("--port", default="COM11", help="串口")
    parser.add_argument("--mode", default="walk", choices=["walk", "stand", "test", "set_zero"])
    parser.add_argument("--steps", type=int, default=10, help="步数")
    parser.add_argument("--period", type=float, default=1.2, help="步态周期 (s)")
    parser.add_argument("--stride", type=float, default=20, help="步幅 (mm)")
    parser.add_argument("--lift", type=float, default=26, help="抬脚高度 (mm)")
    parser.add_argument("--sway", type=float, default=11, help="侧摆 (mm)")
    parser.add_argument("--stance-width", type=float, default=8.0, help="双脚外扩距离 (mm)")
    parser.add_argument("--left-stride-scale", type=float, default=1.18,
                        help="左腿步幅系数(>1增大左腿幅度)")
    parser.add_argument("--action-scale", type=float, default=1.0,
                        help="动作整体放大倍率(>1 放大 stride/lift/sway)")
    parser.add_argument("--ankle-pitch-limit-deg", type=float, default=45.0,
                        help="踝前后角度变化限制(度)")
    parser.add_argument("--pre-shift-ratio", type=float, default=0.30,
                        help="抬腿前重心转移占半步比例")
    parser.add_argument("--crouch-depth", type=float, default=8.0,
                        help="迈步下蹲深度(mm)")
    parser.add_argument("--weight-shift", type=float, default=10.0,
                        help="重心横向转移(mm)")
    parser.add_argument("--landing-ankle-relax", type=float, default=0.35,
                        help="落地踝缓冲比例(0~1)")
    parser.add_argument("--yaw-trim", type=float, default=2.0,
                        help="偏航修正(mm)，正值会让机器人更偏左(用于修正向右偏)")
    parser.add_argument("--height", type=float, default=190, help="站立高度 (mm)")
    parser.add_argument("--zero-file", default=None, help="零位文件")
    args = parser.parse_args()

    default_zero_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "motions", "零位.json")
    )
    zf = args.zero_file
    if zf is None:
        for c in [
            os.path.join(os.path.dirname(__file__), "..", "零位.json"),
            os.path.join(os.path.dirname(__file__), "..", "motions", "零位.json"),
        ]:
            if os.path.exists(c):
                zf = c
                break
    load_zf = zf
    save_zf = os.path.abspath(args.zero_file) if args.zero_file else (os.path.abspath(zf) if zf else default_zero_path)
    zero = load_zero_position(load_zf)

    # 更新尺寸 (如果用户指定了不同高度)
    RobotDimensions.HEIGHT = args.height

    # 步态参数
    params = GaitParams()
    params.period = args.period
    params.stride = args.stride
    params.lift_height = args.lift
    params.sway = args.sway
    params.stance_width = args.stance_width
    params.left_stride_scale = args.left_stride_scale
    params.action_scale = args.action_scale
    params.ankle_pitch_limit_deg = args.ankle_pitch_limit_deg
    params.pre_shift_ratio = args.pre_shift_ratio
    params.crouch_depth = args.crouch_depth
    params.weight_shift = args.weight_shift
    params.landing_ankle_relax = args.landing_ankle_relax
    params.yaw_trim = args.yaw_trim
    params.height = args.height

    robot = Robot(port=args.port)
    robot.connect()

    try:
        if args.mode == "stand":
            do_stand(robot, zero)
            print("按 Ctrl+C 退出...")
            while True:
                time.sleep(1)
        elif args.mode == "test":
            do_test(robot, zero)
        elif args.mode == "set_zero":
            print("\n读取当前姿态并保存为零位...")
            save_zero_position(robot, save_zf)
            print("零位重设完成")
        elif args.mode == "walk":
            do_walk(robot, zero, params, steps=args.steps)
    except KeyboardInterrupt:
        print("\n中断! 回到站立...")
        robot.set_joints_raw(zero, 1000)
        time.sleep(1.5)
    finally:
        robot.disconnect()
        print("已断开")


if __name__ == "__main__":
    main()
