"""
walk.py — 基于 footCont 逆运动学的开环步行控制

移植自 servo_robot_learn 项目的 footCont() 逆运动学，
通过 CPG 生成足端轨迹，IK 计算关节角度，驱动舵机行走。

用法:
    python tools/walk.py --port COM3
    python tools/walk.py --port COM3 --steps 10 --period 1.2
    python tools/walk.py --port COM3 --mode stand
    python tools/walk.py --port COM3 --mode test
"""

import argparse
import json
import math
import os
import sys
import time

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
    "left_hip_yaw": 446,
    "left_hip_pitch": 809,
    "left_knee": 462,
    "left_ankle_pitch": 431,
    "left_ankle_roll": 560,
    "right_hip_yaw": 380,
    "right_hip_pitch": 475,
    "right_knee": 465,
    "right_ankle_pitch": 812,
    "right_ankle_roll": 635,
}


def load_zero_position(filepath: str = None) -> dict[str, int]:
    """从 JSON 文件加载零位"""
    if filepath and os.path.exists(filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        if data.get("keyframes"):
            zero = data["keyframes"][0]["positions"]
            print(f"✅ 零位已从 {filepath} 加载")
            return zero
    print("⚠️ 使用默认零位")
    return DEFAULT_ZERO.copy()


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
#   left_hip_pitch(K0):  站立809, 前摆620 → 前摆时K0增大但值减小 → dir=-1
#   right_hip_pitch(K0): 站立475, 前摆669 → 前摆时K0增大且值增大 → dir=+1
#   left_knee(H):       站立462, 弯曲396 → 弯曲时H增大但值减小  → dir=-1
#   right_knee(H):      站立465, 弯曲558 → 弯曲时H增大且值增大  → dir=+1
#   left_ankle_pitch(A0): 站立431, A0增大→值增大 → dir=+1
#   right_ankle_pitch(A0): 站立812, A0增大→值减小 → dir=-1

JOINT_MAP = {
    "left": {
        # (零位key, IK分量索引, 方向)
        "hip_yaw":     ("left_hip_yaw",     1, -1),   # K1 (roll)
        "hip_pitch":   ("left_hip_pitch",    0, -1),   # K0
        "knee":        ("left_knee",         2, -1),   # H
        "ankle_pitch": ("left_ankle_pitch",  3, +1),   # A0
        "ankle_roll":  ("left_ankle_roll",   4, -1),   # A1
    },
    "right": {
        "hip_yaw":     ("right_hip_yaw",     1, +1),   # K1 (roll)
        "hip_pitch":   ("right_hip_pitch",   0, +1),   # K0
        "knee":        ("right_knee",        2, +1),   # H
        "ankle_pitch": ("right_ankle_pitch", 3, -1),   # A0
        "ankle_roll":  ("right_ankle_roll",  4, +1),   # A1
    },
}

# 站立时 5 个分量的角度值 (用于计算 delta)
STAND_ANGLES = [STAND_K0, STAND_K1, STAND_H, STAND_A0, STAND_A1]


def ik_to_servo(
    ik_angles: tuple[float, ...],
    side: str,
    zero: dict[str, int],
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
        self.dt: float = 0.02           # 控制周期 (秒), 50Hz
        self.stride: float = 20.0       # 步幅 (mm), 前后各 ±stride/2
        self.lift_height: float = 15.0  # 抬脚高度 (mm)
        self.sway: float = 8.0          # 横向摆动幅度 (mm)
        self.height: float = RobotDimensions.HEIGHT  # 站立高度


def foot_trajectory(phase: float, params: GaitParams):
    """计算单脚足端轨迹

    phase (0~1):
        0.0~0.5 = 支撑期 (脚在地面, 向后推)
        0.5~1.0 = 摆动期 (脚抬起, 向前摆)

    Returns:
        (x, y, h): 足端位置 (mm)
        x = 前后 (前为正)
        y = 横向偏移 (正=向身体外侧)
        h = 垂直高度 (踝→髋)
    """
    two_pi = 2.0 * math.pi
    half_stride = params.stride / 2.0

    # ── 前后方向 (x) ──
    # 支撑期: 从前到后 (脚在地面推身体前进)
    # 摆动期: 从后到前 (脚抬起向前摆)
    # 用正弦曲线平滑: 支撑期前半→后半, 摆动期后半→前半
    x = -half_stride * math.sin(two_pi * phase)

    # ── 横向方向 (y) ──
    # 支撑期: 身体重心向支撑腿侧移
    y = params.sway * math.sin(two_pi * phase)

    # ── 高度方向 (h) ──
    # 支撑期: 恒定高度
    # 摆动期: 正弦抬脚
    if 0.5 <= phase < 1.0:
        swing_phase = (phase - 0.5) / 0.5  # 0~1 in swing
        lift = params.lift_height * math.sin(math.pi * swing_phase)
    else:
        lift = 0.0

    h = params.height + lift  # 抬脚时 h 增大 (脚离地更远)

    return x, y, h


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
    half_period = params.period / 2.0

    # 左右腿相位 (0~1), 错开半周期
    left_phase = (t % half_period) / half_period
    right_phase = ((t + half_period / 2.0) % half_period) / half_period

    # 计算足端轨迹
    lx, ly, lh = foot_trajectory(left_phase, params)
    rx, ry, rh = foot_trajectory(right_phase, params)

    # 逆运动学: 足端 → 关节角
    left_angles = foot_cont(lx, ly, lh)
    right_angles = foot_cont(rx, -ry, rh)  # 右腿 y 取反 (镜像)

    # 关节角 → 舵机值
    frame = {}
    frame.update(ik_to_servo(left_angles, "left", zero))
    frame.update(ik_to_servo(right_angles, "right", zero))

    return frame


# ╔══════════════════════════════════════════════════════════════╗
# ║                       主程序                                  ║
# ╚══════════════════════════════════════════════════════════════╝

def do_stand(robot: Robot, zero: dict[str, int]):
    """站立"""
    print("\n🧍 进入站立姿态...")
    robot.set_joints_raw(zero, 1500)
    time.sleep(2.0)
    print("✅ 站立就绪")


def do_test(robot: Robot, zero: dict[str, int]):
    """IK 测试: 在站立基础上小幅前后摆腿, 验证方向"""
    print("\n🔧 IK 验证测试")
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
    print("✅ IK 测试完成")


def do_walk(robot: Robot, zero: dict[str, int], params: GaitParams, steps: int = 6):
    """开环步行"""
    total_time = steps * params.period / 2.0
    print(f"\n🚶 开始行走: {steps} 步, 周期 {params.period}s, 预计 {total_time:.1f}s")
    print(f"   步幅: {params.stride}mm, 抬脚: {params.lift_height}mm, 侧摆: {params.sway}mm")
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
        print("\n\n⏹ 中断!")

    print("\n🧍 回到站立...")
    robot.set_joints_raw(zero, 1000)
    time.sleep(1.5)
    print(f"✅ 行走完成 (耗时 {time.time()-start:.1f}s)")


def main():
    parser = argparse.ArgumentParser(description="footCont IK 开环步行")
    parser.add_argument("--port", default="COM3", help="串口")
    parser.add_argument("--mode", default="walk", choices=["walk", "stand", "test"])
    parser.add_argument("--steps", type=int, default=6, help="步数")
    parser.add_argument("--period", type=float, default=1.2, help="步态周期 (s)")
    parser.add_argument("--stride", type=float, default=20, help="步幅 (mm)")
    parser.add_argument("--lift", type=float, default=15, help="抬脚高度 (mm)")
    parser.add_argument("--sway", type=float, default=8, help="侧摆 (mm)")
    parser.add_argument("--height", type=float, default=190, help="站立高度 (mm)")
    parser.add_argument("--zero-file", default=None, help="零位文件")
    args = parser.parse_args()

    # 查找零位文件
    zf = args.zero_file
    if zf is None:
        for c in [
            os.path.join(os.path.dirname(__file__), "..", "零位.json"),
            os.path.join(os.path.dirname(__file__), "..", "motions", "零位.json"),
        ]:
            if os.path.exists(c):
                zf = c
                break
    zero = load_zero_position(zf)

    # 更新尺寸 (如果用户指定了不同高度)
    RobotDimensions.HEIGHT = args.height

    # 步态参数
    params = GaitParams()
    params.period = args.period
    params.stride = args.stride
    params.lift_height = args.lift
    params.sway = args.sway
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
        elif args.mode == "walk":
            do_walk(robot, zero, params, steps=args.steps)
    except KeyboardInterrupt:
        print("\n⏹ 中断! 回到站立...")
        robot.set_joints_raw(zero, 1000)
        time.sleep(1.5)
    finally:
        robot.disconnect()
        print("👋 已断开")


if __name__ == "__main__":
    main()
