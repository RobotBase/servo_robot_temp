"""
test_walk.py — 双足机器人步行完整测试脚本
══════════════════════════════════════════

🎯 用途：远程交给现场测试人员，按步骤逐项验证机器人硬件和步行功能。
📋 每一步都有详细说明，测试人员只需按提示操作并反馈结果。

使用方法:
    python tools/test_walk.py --port COM3

⚠️  安全注意:
    - 全程用手扶住机器人，防止倒地损坏舵机
    - 确保电池电量充足 (>7.0V)
    - 任何异常立刻 Ctrl+C 停止
"""

import argparse
import json
import math
import os
import sys
import time
import traceback
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.robot import Robot
from src.robot_config import ROBOT_CONFIG


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  配置区 — 根据实际情况修改
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# 机器人腿部尺寸 (mm)
THIGH_LEN = 65.0          # 大腿长
SHIN_LEN = 65.0           # 小腿长
LEG_LEN = THIGH_LEN + SHIN_LEN  # 130mm
LATERAL_OFFSET = 64.5     # K1→K0 + A0→A1 横向偏移
HEIGHT = 190.0            # 站立高度 (踝→髋)

# 舵机参数
SERVO_UNITS_PER_DEG = 1.0 / 0.24  # ≈ 4.167

# 零位 (站立时各舵机原始值)
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

# 关节方向映射 (IK角度增大时舵机值的变化方向)
# ⚠️ 这是需要测试验证的核心参数!
JOINT_MAP = {
    "left": {
        "hip_yaw":     ("left_hip_yaw",     1, -1),
        "hip_pitch":   ("left_hip_pitch",    0, -1),
        "knee":        ("left_knee",         2, -1),
        "ankle_pitch": ("left_ankle_pitch",  3, +1),
        "ankle_roll":  ("left_ankle_roll",   4, -1),
    },
    "right": {
        "hip_yaw":     ("right_hip_yaw",     1, +1),
        "hip_pitch":   ("right_hip_pitch",   0, +1),
        "knee":        ("right_knee",        2, +1),
        "ankle_pitch": ("right_ankle_pitch", 3, -1),
        "ankle_roll":  ("right_ankle_roll",  4, +1),
    },
}


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  工具函数
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class TestLog:
    """测试日志记录器"""

    def __init__(self):
        self.entries: list[dict] = []
        self.start_time = datetime.now()

    def log(self, step: str, result: str, details: str = ""):
        entry = {
            "time": datetime.now().isoformat(),
            "step": step,
            "result": result,
            "details": details,
        }
        self.entries.append(entry)
        icon = "✅" if result == "PASS" else "❌" if result == "FAIL" else "⚠️"
        print(f"    {icon} [{result}] {details}" if details else f"    {icon} [{result}]")

    def save(self, filepath: str):
        report = {
            "test_date": self.start_time.isoformat(),
            "total_steps": len(self.entries),
            "passed": sum(1 for e in self.entries if e["result"] == "PASS"),
            "failed": sum(1 for e in self.entries if e["result"] == "FAIL"),
            "skipped": sum(1 for e in self.entries if e["result"] == "SKIP"),
            "entries": self.entries,
        }
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print(f"\n📄 测试报告已保存: {filepath}")

    def summary(self):
        passed = sum(1 for e in self.entries if e["result"] == "PASS")
        failed = sum(1 for e in self.entries if e["result"] == "FAIL")
        skipped = sum(1 for e in self.entries if e["result"] == "SKIP")
        total = len(self.entries)
        print(f"\n{'='*60}")
        print(f"  测试总结: {passed}✅ / {failed}❌ / {skipped}⏭️  (共 {total} 项)")
        print(f"{'='*60}")


def load_zero(filepath: str = None) -> dict[str, int]:
    if filepath and os.path.exists(filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        if data.get("keyframes"):
            return data["keyframes"][0]["positions"]
    return DEFAULT_ZERO.copy()


def wait_for_user(prompt: str = "按 Enter 继续...") -> str:
    """等待用户输入"""
    try:
        return input(f"    👉 {prompt} ").strip()
    except EOFError:
        return ""


def ask_yes_no(question: str) -> bool:
    """询问 yes/no"""
    while True:
        answer = wait_for_user(f"{question} (y/n): ").lower()
        if answer in ("y", "yes", "是"):
            return True
        if answer in ("n", "no", "否", ""):
            return False
        print("    请输入 y 或 n")


def ask_choice(question: str, choices: list[str]) -> str:
    """选择题"""
    for i, c in enumerate(choices):
        print(f"      {i+1}. {c}")
    while True:
        answer = wait_for_user(f"{question} (输入编号): ")
        try:
            idx = int(answer) - 1
            if 0 <= idx < len(choices):
                return choices[idx]
        except ValueError:
            pass
        print(f"    请输入 1~{len(choices)}")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  逆运动学 (移植自 robot_ctrl.c)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def foot_cont(x: float, y: float, h: float) -> tuple[float, ...]:
    """逆运动学: 足端位置(mm) → 5个关节角度(弧度)

    Args:
        x: 前后 (前+) mm
        y: 左右 (外侧+) mm
        h: 踝→髋高度 mm

    Returns:
        (K0, K1, H, A0, A1) 弧度
    """
    inner = math.sqrt(y * y + h * h) - LATERAL_OFFSET
    if inner < 0:
        inner = 0
    k = math.sqrt(x * x + inner * inner)
    if k > LEG_LEN - 1:
        k = LEG_LEN - 1

    k0 = math.acos(k / LEG_LEN)
    x0 = math.asin(max(-1.0, min(1.0, x / k))) if k > 0.001 else 0.0
    k1 = math.atan2(y, h) if h > 0.001 else 0.0

    K0 = k0 + x0
    H = k0 * 2.0
    A0 = k0 - x0
    K1 = k1
    A1 = -k1
    return K0, K1, H, A0, A1


# 站立角度 (基准)
STAND_ANGLES = list(foot_cont(0, 0, HEIGHT))


def ik_to_servo(
    ik_angles: tuple[float, ...],
    side: str,
    zero: dict[str, int],
) -> dict[str, int]:
    """IK 角度 → 舵机原始值"""
    result = {}
    for joint_short, (zero_key, ik_idx, direction) in JOINT_MAP[side].items():
        delta_rad = ik_angles[ik_idx] - STAND_ANGLES[ik_idx]
        delta_servo = math.degrees(delta_rad) * SERVO_UNITS_PER_DEG
        joint_name = f"{side}_{joint_short}"
        raw = zero[zero_key] + direction * delta_servo
        result[joint_name] = int(max(0, min(1000, raw)))
    return result


def foot_trajectory(phase: float, stride: float, lift: float,
                    sway: float, h: float):
    """CPG 足端轨迹: phase(0~1) → (x, y, h)"""
    two_pi = 2.0 * math.pi
    x = -(stride / 2.0) * math.sin(two_pi * phase)
    y = sway * math.sin(two_pi * phase)
    if 0.5 <= phase < 1.0:
        swing = (phase - 0.5) / 0.5
        lift_h = lift * math.sin(math.pi * swing)
    else:
        lift_h = 0.0
    return x, y, h + lift_h


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  测试步骤
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def step_0_environment(log: TestLog):
    """Step 0: 环境检查"""
    print("\n" + "="*60)
    print("  STEP 0: 环境检查")
    print("="*60)

    # Python 版本
    v = sys.version
    print(f"  Python: {v}")
    log.log("环境-Python", "PASS", f"Python {sys.version_info.major}.{sys.version_info.minor}")

    # pyserial
    try:
        import serial
        print(f"  pyserial: {serial.__version__}")
        log.log("环境-pyserial", "PASS", f"v{serial.__version__}")
    except ImportError:
        print("  ❌ pyserial 未安装! 请运行: pip install pyserial")
        log.log("环境-pyserial", "FAIL", "未安装")
        return False

    # 列出串口
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    print(f"  可用串口: {[p.device for p in ports]}")
    if not ports:
        print("  ❌ 没有找到任何串口!")
        log.log("环境-串口", "FAIL", "无可用串口")
        return False
    log.log("环境-串口", "PASS", str([p.device for p in ports]))

    return True


def step_1_connect(robot: Robot, log: TestLog) -> bool:
    """Step 1: 连接舵机总线"""
    print("\n" + "="*60)
    print("  STEP 1: 连接舵机总线")
    print("="*60)
    print(f"  串口: {robot.bus._port}")
    print()
    print("  📋 确认清单:")
    print("     □ USB 串口线已连接到电脑")
    print("     □ 串口线已连接到舵机总线板/舵机")
    print("     □ 电池/电源已接好 (7.4V)")
    print("     □ 电源开关已打开")
    print()

    wait_for_user("确认以上都已就绪后按 Enter")

    try:
        robot.connect()
        log.log("连接", "PASS", f"已连接 {robot.bus._port}")
        return True
    except Exception as e:
        print(f"  ❌ 连接失败: {e}")
        log.log("连接", "FAIL", str(e))
        print()
        print("  🔧 排查方法:")
        print("     1. 检查串口号是否正确 (设备管理器查看)")
        print("     2. 关闭其他可能占用串口的软件")
        print("     3. 拔插 USB 线重试")
        return False


def step_2_scan(robot: Robot, log: TestLog) -> bool:
    """Step 2: 扫描舵机"""
    print("\n" + "="*60)
    print("  STEP 2: 扫描舵机")
    print("="*60)
    print("  预期: 应发现 10 个舵机 (ID 1~10)")
    print("  正在扫描...")

    results = robot.scan_servos()

    online = [r for r in results if r["online"]]
    offline = [r for r in results if not r["online"]]

    print(f"\n  在线: {len(online)}/10")
    print(f"  {'ID':>4}  {'关节名':<22}  {'位置':>5}  {'电压':>6}  {'温度':>4}")
    print(f"  {'─'*4}  {'─'*22}  {'─'*5}  {'─'*6}  {'─'*4}")

    for r in results:
        if r["online"]:
            vin = f"{r['voltage']/1000:.1f}V" if r["voltage"] else "N/A"
            temp = f"{r['temperature']}°C" if r["temperature"] else "N/A"
            print(f"  {r['id']:>4}  {r['name']:<22}  {r['position']:>5}  {vin:>6}  {temp:>4}")
        else:
            print(f"  {r['id']:>4}  {r['name']:<22}  ── 离线 ──")

    if len(online) == 10:
        log.log("扫描", "PASS", "10/10 在线")
        # 检查电压
        voltages = [r["voltage"] for r in online if r["voltage"]]
        if voltages:
            min_v = min(voltages) / 1000
            if min_v < 6.5:
                print(f"\n  ⚠️ 最低电压 {min_v:.1f}V, 偏低! 建议充电")
                log.log("电压", "WARN", f"最低 {min_v:.1f}V")
            else:
                log.log("电压", "PASS", f"最低 {min_v:.1f}V")
        return True
    elif len(online) >= 8:
        log.log("扫描", "WARN", f"{len(online)}/10 在线, 缺少: {[r['name'] for r in offline]}")
        print(f"\n  ⚠️ 有舵机离线: {[r['name'] for r in offline]}")
        return ask_yes_no("部分舵机离线, 是否继续测试?")
    else:
        log.log("扫描", "FAIL", f"仅 {len(online)}/10 在线")
        print("\n  ❌ 大量舵机离线!")
        print("  🔧 排查方法:")
        print("     1. 检查电源是否接通")
        print("     2. 检查舵机信号线连接")
        print("     3. 确认舵机 ID 是否为 1~10")
        return False


def step_3_zero_position(robot: Robot, zero: dict[str, int], log: TestLog) -> bool:
    """Step 3: 零位测试"""
    print("\n" + "="*60)
    print("  STEP 3: 零位测试 (站立)")
    print("="*60)
    print("  ⚠️  用手扶住机器人! 舵机将移动到站立姿态")
    print()
    print("  零位值:")
    for name, val in zero.items():
        print(f"    {name:<22} = {val}")
    print()

    wait_for_user("用手扶住机器人后按 Enter")

    print("  发送零位命令 (1.5秒到达)...")
    robot.set_joints_raw(zero, 1500)
    time.sleep(2.0)
    print("  ✅ 命令已发送")
    print()

    result = ask_choice("机器人姿态是否正常?", [
        "正常 — 双腿直立, 身体正直",
        "基本正常 — 大体直立, 轻微偏差",
        "异常 — 姿态明显不对 (腿弯曲/扭曲/趴下)",
    ])

    if "正常" in result and "异常" not in result:
        log.log("零位", "PASS", result)
        return True
    else:
        log.log("零位", "FAIL", result)
        if "异常" in result:
            print("\n  📝 请记录当前姿态的具体问题 (哪个关节不对):")
            detail = wait_for_user("描述问题: ")
            log.log("零位-详情", "FAIL", detail)

            # 读取实际位置用于排查
            print("\n  正在读取各关节实际位置...")
            actual = robot.get_all_positions()
            print(f"  {'关节名':<22}  {'零位':>5}  {'实际':>5}  {'差值':>5}")
            print(f"  {'─'*22}  {'─'*5}  {'─'*5}  {'─'*5}")
            for name, zero_val in zero.items():
                act = actual.get(name, None)
                if act is not None:
                    diff = act - zero_val
                    marker = " ⚠️" if abs(diff) > 30 else ""
                    print(f"  {name:<22}  {zero_val:>5}  {act:>5}  {diff:>+5}{marker}")
                else:
                    print(f"  {name:<22}  {zero_val:>5}  {'N/A':>5}")
            log.log("零位-实际位置", "INFO", json.dumps(actual))

        return ask_yes_no("是否继续下一步测试?")


def step_4_single_joint(robot: Robot, zero: dict[str, int], log: TestLog) -> bool:
    """Step 4: 逐关节方向验证"""
    print("\n" + "="*60)
    print("  STEP 4: 逐关节方向验证")
    print("="*60)
    print("  目的: 验证每个关节的运动方向是否符合预期")
    print("  方法: 每次只动一个关节, 观察是否正确")
    print("  ⚠️  全程扶住机器人!")
    print()

    # 测试项目
    tests = [
        {
            "name": "左髋前后摆 (left_hip_pitch)",
            "joint": "left_hip_pitch",
            "expect_pos": "正方向: 机器人左腿应该向前迈",
            "expect_neg": "负方向: 机器人左腿应该向后摆",
            "amount": 60,
        },
        {
            "name": "左膝弯曲 (left_knee)",
            "joint": "left_knee",
            "expect_pos": "正方向: 左膝应该弯曲(蹲下)",
            "expect_neg": "负方向: 左膝应该伸直",
            "amount": 50,
        },
        {
            "name": "左踝前后 (left_ankle_pitch)",
            "joint": "left_ankle_pitch",
            "expect_pos": "正方向: 左脚尖应该翘起",
            "expect_neg": "负方向: 左脚尖应该压下",
            "amount": 50,
        },
        {
            "name": "左髋侧摆 (left_hip_yaw)",
            "joint": "left_hip_yaw",
            "expect_pos": "正方向: 左腿应该向外侧展开",
            "expect_neg": "负方向: 左腿应该向内侧收拢",
            "amount": 30,
        },
        {
            "name": "左踝侧摆 (left_ankle_roll)",
            "joint": "left_ankle_roll",
            "expect_pos": "正方向: 左脚应该向外侧翻",
            "expect_neg": "负方向: 左脚应该向内侧翻",
            "amount": 30,
        },
        {
            "name": "右髋前后摆 (right_hip_pitch)",
            "joint": "right_hip_pitch",
            "expect_pos": "正方向: 机器人右腿应该向前迈",
            "expect_neg": "负方向: 机器人右腿应该向后摆",
            "amount": 60,
        },
        {
            "name": "右膝弯曲 (right_knee)",
            "joint": "right_knee",
            "expect_pos": "正方向: 右膝应该弯曲",
            "expect_neg": "负方向: 右膝应该伸直",
            "amount": 50,
        },
        {
            "name": "右踝前后 (right_ankle_pitch)",
            "joint": "right_ankle_pitch",
            "expect_pos": "正方向: 右脚尖应该翘起",
            "expect_neg": "负方向: 右脚尖应该压下",
            "amount": 50,
        },
        {
            "name": "右髋侧摆 (right_hip_yaw)",
            "joint": "right_hip_yaw",
            "expect_pos": "正方向: 右腿应该向外侧展开",
            "expect_neg": "负方向: 右腿应该向内侧收拢",
            "amount": 30,
        },
        {
            "name": "右踝侧摆 (right_ankle_roll)",
            "joint": "right_ankle_roll",
            "expect_pos": "正方向: 右脚应该向外侧翻",
            "expect_neg": "负方向: 右脚应该向内侧翻",
            "amount": 30,
        },
    ]

    # 获取 JOINT_MAP 方向表
    dir_lookup = {}
    for side, joints in JOINT_MAP.items():
        for jshort, (zkey, idx, direction) in joints.items():
            dir_lookup[zkey] = direction

    direction_corrections = {}  # 记录需要反转的关节

    for i, test in enumerate(tests):
        print(f"\n  ── 测试 {i+1}/{len(tests)}: {test['name']} ──")
        print(f"    {test['expect_pos']}")
        print(f"    {test['expect_neg']}")
        print()

        wait_for_user("准备好后按 Enter (发送正方向)")

        # 正方向测试
        direction = dir_lookup.get(test["joint"], 1)
        frame_pos = zero.copy()
        frame_pos[test["joint"]] = zero[test["joint"]] + direction * test["amount"]
        frame_pos[test["joint"]] = max(0, min(1000, frame_pos[test["joint"]]))

        print(f"    → 发送正方向: {test['joint']} = {frame_pos[test['joint']]} "
              f"(零位 {zero[test['joint']]} {"+" if direction>0 else ""}{direction * test['amount']})")
        robot.set_joints_raw(frame_pos, 600)
        time.sleep(1.0)

        # 负方向
        frame_neg = zero.copy()
        frame_neg[test["joint"]] = zero[test["joint"]] - direction * test["amount"]
        frame_neg[test["joint"]] = max(0, min(1000, frame_neg[test["joint"]]))

        print(f"    → 发送负方向: {test['joint']} = {frame_neg[test['joint']]} "
              f"(零位 {zero[test['joint']]} {"+" if -direction>0 else ""}{-direction * test['amount']})")
        robot.set_joints_raw(frame_neg, 600)
        time.sleep(1.0)

        # 回零
        robot.set_joints_raw(zero, 500)
        time.sleep(0.5)

        result = ask_choice("运动方向是否正确?", [
            "正确 — 动作符合描述",
            "反了 — 正负方向和描述相反",
            "不确定 — 看不清/没反应",
        ])

        if "正确" in result:
            log.log(f"方向-{test['joint']}", "PASS", "方向正确")
        elif "反了" in result:
            log.log(f"方向-{test['joint']}", "FAIL", "方向反了! 需要修改 JOINT_MAP")
            direction_corrections[test["joint"]] = "REVERSE"
        else:
            log.log(f"方向-{test['joint']}", "WARN", result)

    # 汇总方向调整
    if direction_corrections:
        print("\n  ⚠️ 以下关节方向需要反转:")
        for joint, action in direction_corrections.items():
            print(f"    - {joint}: {action}")
        log.log("方向汇总", "FAIL",
                f"需要反转: {list(direction_corrections.keys())}")
        print("\n  📝 请把这个列表反馈给开发者, 他会修改 JOINT_MAP 配置")
        return ask_yes_no("是否跳过方向问题继续测试步行?")
    else:
        print("\n  ✅ 所有关节方向正确!")
        log.log("方向汇总", "PASS", "全部10个关节方向正确")
        return True


def step_5_ik_test(robot: Robot, zero: dict[str, int], log: TestLog) -> bool:
    """Step 5: 逆运动学联合测试"""
    print("\n" + "="*60)
    print("  STEP 5: 逆运动学联合测试")
    print("="*60)
    print("  目的: 验证多关节同时运动时, IK 计算是否正确")
    print("  ⚠️  扶住机器人!")
    print()

    wait_for_user("准备好后按 Enter")

    tests = [
        ("站立 (0, 0, 190)",           0,   0,  HEIGHT),
        ("左脚前伸 20mm",              20,   0,  HEIGHT),
        ("左脚后摆 20mm",             -20,   0,  HEIGHT),
        ("左脚外展 10mm",               0,  10,  HEIGHT),
        ("左脚抬高 15mm (脚离地更远)",   0,   0,  HEIGHT + 15),
        ("左脚弯曲蹲下 (高度-20mm)",     0,   0,  HEIGHT - 20),
        ("站立复位",                     0,   0,  HEIGHT),
    ]

    for name, x, y, h in tests:
        angles = foot_cont(x, y, h)
        K0, K1, H, A0, A1 = [math.degrees(a) for a in angles]
        print(f"\n    {name}")
        print(f"    IK: K0={K0:.1f}° K1={K1:.1f}° H={H:.1f}° A0={A0:.1f}° A1={A1:.1f}°")

        frame = zero.copy()
        left_servos = ik_to_servo(angles, "left", zero)
        frame.update(left_servos)

        robot.set_joints_raw(frame, 800)
        time.sleep(1.0)

    result = ask_choice("左腿 IK 运动是否合理?", [
        "合理 — 动作平滑, 方向正确, 脚掌基本保持水平",
        "部分合理 — 大方向对但有轻微问题",
        "不合理 — 动作混乱/方向错误",
    ])

    if "不合理" not in result:
        log.log("IK测试", "PASS", result)
    else:
        log.log("IK测试", "FAIL", result)
        detail = wait_for_user("描述问题: ")
        log.log("IK测试-详情", "FAIL", detail)

    # 回到零位
    robot.set_joints_raw(zero, 800)
    time.sleep(1.0)

    return "不合理" not in result


def step_6_mini_walk(robot: Robot, zero: dict[str, int], log: TestLog) -> bool:
    """Step 6: 小幅度步行测试"""
    print("\n" + "="*60)
    print("  STEP 6: 小幅度步行测试 (最保守参数)")
    print("="*60)
    print("  参数: 步幅=10mm, 抬脚=8mm, 侧摆=5mm, 周期=1.5s")
    print("  步数: 4步")
    print("  ⚠️  全程四根手指托住机器人腰部!")
    print()

    wait_for_user("扶好机器人后按 Enter 开始")

    stride, lift, sway, period, dt = 10, 8, 5, 1.5, 0.02
    steps = 4
    half_period = period / 2.0
    total_time = steps * half_period
    move_ms = int(dt * 1000)
    t = 0.0

    print(f"  🚶 开始 (约 {total_time:.0f}秒)...")

    try:
        # 先站立
        robot.set_joints_raw(zero, 1000)
        time.sleep(1.5)

        while t < total_time:
            loop_start = time.time()
            lp = (t % half_period) / half_period
            rp = ((t + half_period / 2) % half_period) / half_period

            lx, ly, lh = foot_trajectory(lp, stride, lift, sway, HEIGHT)
            rx, ry, rh = foot_trajectory(rp, stride, lift, sway, HEIGHT)

            la = foot_cont(lx, ly, lh)
            ra = foot_cont(rx, -ry, rh)

            frame = {}
            frame.update(ik_to_servo(la, "left", zero))
            frame.update(ik_to_servo(ra, "right", zero))
            robot.set_joints_raw(frame, move_ms)

            t += dt
            elapsed = time.time() - loop_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\n    ⏹ 中断!")

    robot.set_joints_raw(zero, 1000)
    time.sleep(1.5)
    print("  完成!")

    result = ask_choice("小幅步行效果如何?", [
        "看起来像在走 — 有交替抬脚和摆动",
        "有动作但不像走路 — 抖动/方向混乱",
        "几乎没动 / 完全不对",
    ])

    log.log("小幅步行", "PASS" if "像在走" in result else "FAIL", result)

    if "完全不对" in result:
        detail = wait_for_user("描述具体现象: ")
        log.log("小幅步行-详情", "FAIL", detail)

    return "像在走" in result


def step_7_full_walk(robot: Robot, zero: dict[str, int], log: TestLog):
    """Step 7: 正常参数步行"""
    print("\n" + "="*60)
    print("  STEP 7: 正常参数步行测试")
    print("="*60)
    print("  参数: 步幅=20mm, 抬脚=15mm, 侧摆=8mm, 周期=1.2s")
    print("  步数: 6步")
    print("  ⚠️  全程保护机器人!")
    print()

    if not ask_yes_no("是否进行正常参数步行?"):
        log.log("正常步行", "SKIP", "用户跳过")
        return

    wait_for_user("扶好机器人后按 Enter 开始")

    stride, lift, sway, period, dt = 20, 15, 8, 1.2, 0.02
    steps = 6
    half_period = period / 2.0
    total_time = steps * half_period
    move_ms = int(dt * 1000)
    t = 0.0

    print(f"  🚶 开始 (约 {total_time:.0f}秒)...")

    try:
        robot.set_joints_raw(zero, 1000)
        time.sleep(1.5)

        while t < total_time:
            loop_start = time.time()
            lp = (t % half_period) / half_period
            rp = ((t + half_period / 2) % half_period) / half_period

            lx, ly, lh = foot_trajectory(lp, stride, lift, sway, HEIGHT)
            rx, ry, rh = foot_trajectory(rp, stride, lift, sway, HEIGHT)

            la = foot_cont(lx, ly, lh)
            ra = foot_cont(rx, -ry, rh)

            frame = {}
            frame.update(ik_to_servo(la, "left", zero))
            frame.update(ik_to_servo(ra, "right", zero))
            robot.set_joints_raw(frame, move_ms)

            t += dt
            elapsed = time.time() - loop_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\n    ⏹ 中断!")

    robot.set_joints_raw(zero, 1000)
    time.sleep(1.5)
    print("  完成!")

    result = ask_choice("正常步行效果如何?", [
        "很好 — 明显的步行动作, 基本稳定",
        "一般 — 有步行动作但不稳/偏移",
        "很差 — 无法维持, 需要大力扶住",
    ])
    log.log("正常步行", "PASS" if "很好" in result else "WARN", result)

    # 额外反馈
    if ask_yes_no("是否要尝试调整参数?"):
        print("\n  可调参数:")
        print("    步幅 (stride): 当前20mm, 范围5~40mm")
        print("    抬脚 (lift):   当前15mm, 范围5~30mm")
        print("    侧摆 (sway):   当前8mm,  范围3~15mm")
        print("    周期 (period):  当前1.2s, 范围0.8~2.0s")
        feedback = wait_for_user("你觉得应该怎么调? (描述): ")
        log.log("参数建议", "INFO", feedback)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  主流程
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    parser = argparse.ArgumentParser(description="双足机器人步行完整测试")
    parser.add_argument("--port", default="COM3", help="串口名")
    parser.add_argument("--zero-file", default=None, help="零位文件路径")
    parser.add_argument("--skip-to", type=int, default=0,
                        help="跳过前N步 (用于断点续测)")
    args = parser.parse_args()

    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║          🤖 双足机器人步行测试 — 完整流程                  ║")
    print("╠══════════════════════════════════════════════════════════╣")
    print("║                                                        ║")
    print("║  本测试将逐步验证:                                       ║")
    print("║    Step 0: 环境检查 (Python/串口)                        ║")
    print("║    Step 1: 连接舵机总线                                  ║")
    print("║    Step 2: 扫描 10 个舵机                                ║")
    print("║    Step 3: 零位站立测试                                  ║")
    print("║    Step 4: 逐关节方向验证 (10个关节)                      ║")
    print("║    Step 5: 逆运动学联合测试                              ║")
    print("║    Step 6: 小幅度步行测试                                ║")
    print("║    Step 7: 正常参数步行测试                              ║")
    print("║                                                        ║")
    print("║  ⚠️  安全: 全程用手扶住机器人!                            ║")
    print("║  ⏹  任何时候按 Ctrl+C 可安全停止                         ║")
    print("║                                                        ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()

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
    zero = load_zero(zf)

    log = TestLog()
    robot = Robot(port=args.port)
    report_file = os.path.join(
        os.path.dirname(__file__), "..",
        f"test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )

    try:
        # Step 0
        if args.skip_to <= 0:
            if not step_0_environment(log):
                log.summary()
                log.save(report_file)
                return

        # Step 1
        if args.skip_to <= 1:
            if not step_1_connect(robot, log):
                log.summary()
                log.save(report_file)
                return
        else:
            robot.connect()

        # Step 2
        if args.skip_to <= 2:
            if not step_2_scan(robot, log):
                log.summary()
                log.save(report_file)
                return

        # Step 3
        if args.skip_to <= 3:
            if not step_3_zero_position(robot, zero, log):
                log.summary()
                log.save(report_file)
                return

        # Step 4
        if args.skip_to <= 4:
            if not step_4_single_joint(robot, zero, log):
                log.summary()
                log.save(report_file)
                return

        # Step 5
        if args.skip_to <= 5:
            if not step_5_ik_test(robot, zero, log):
                if not ask_yes_no("IK 测试未通过, 是否继续尝试步行?"):
                    log.summary()
                    log.save(report_file)
                    return

        # Step 6
        if args.skip_to <= 6:
            if not step_6_mini_walk(robot, zero, log):
                if not ask_yes_no("小幅步行不理想, 是否继续尝试正常步行?"):
                    log.summary()
                    log.save(report_file)
                    return

        # Step 7
        step_7_full_walk(robot, zero, log)

    except KeyboardInterrupt:
        print("\n\n  ⏹ 测试中断!")
        log.log("中断", "WARN", "用户 Ctrl+C 中断")
        try:
            robot.set_joints_raw(zero, 1000)
            time.sleep(1.5)
        except Exception:
            pass

    except Exception as e:
        print(f"\n  ❌ 异常: {e}")
        traceback.print_exc()
        log.log("异常", "FAIL", f"{type(e).__name__}: {e}")

    finally:
        try:
            robot.disconnect()
        except Exception:
            pass

        log.summary()
        log.save(report_file)

        print(f"\n  📬 请将以下文件发回给开发者:")
        print(f"     {os.path.abspath(report_file)}")
        print()


if __name__ == "__main__":
    main()
