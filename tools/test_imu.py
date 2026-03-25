"""
test_imu.py — YIS321 IMU 完整测试脚本
═══════════════════════════════════════

🎯 用途：远程交给现场测试人员，逐步验证 IMU 硬件和数据质量。
📋 每一步都有详细说明，测试员只需按提示操作并反馈结果。

使用方法:
    python tools/test_imu.py --port COM5
    python tools/test_imu.py --port COM15 --baud 921600

⚠️ 注意：IMU 串口和舵机串口是不同的！
    - 舵机串口: 一般 COM3, 波特率 115200
    - IMU 串口: 一般 COM15, 波特率 921600
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

from src.yis321_driver import YIS321Driver, IMUData


class TestLog:
    """测试日志"""
    def __init__(self):
        self.entries = []
        self.start_time = datetime.now()
    def log(self, step, result, details=""):
        self.entries.append({"time": datetime.now().isoformat(), "step": step,
                             "result": result, "details": details})
        icon = "PASS" if result == "PASS" else "FAIL" if result == "FAIL" else "WARN"
        print(f"    [{icon}] [{result}] {details}" if details else f"    [{icon}] [{result}]")
    def save(self, filepath):
        report = {"test_date": self.start_time.isoformat(),
                  "total": len(self.entries),
                  "passed": sum(1 for e in self.entries if e["result"] == "PASS"),
                  "failed": sum(1 for e in self.entries if e["result"] == "FAIL"),
                  "entries": self.entries}
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print(f"\n测试报告已保存: {filepath}")
    def summary(self):
        p = sum(1 for e in self.entries if e["result"] == "PASS")
        f = sum(1 for e in self.entries if e["result"] == "FAIL")
        print(f"\n{'='*60}")
        print(f"  测试总结: {p}PASS / {f}FAIL  (共 {len(self.entries)} 项)")
        print(f"{'='*60}")


def wait_input(prompt="按 Enter 继续..."):
    try:
        return input(f"    {prompt} ").strip()
    except EOFError:
        return ""


def ask_yes_no(q):
    while True:
        a = wait_input(f"{q} (y/n): ").lower()
        if a in ("y", "yes", "是"): return True
        if a in ("n", "no", "否", ""): return False


def ask_choice(q, choices):
    for i, c in enumerate(choices):
        print(f"      {i+1}. {c}")
    while True:
        a = wait_input(f"{q} (输入编号): ")
        try:
            idx = int(a) - 1
            if 0 <= idx < len(choices): return choices[idx]
        except ValueError: pass


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  测试步骤
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def step_0_env(log):
    """Step 0: 环境检查"""
    print("\n" + "="*60)
    print("  STEP 0: 环境检查")
    print("="*60)

    try:
        import serial
        print(f"  pyserial: {serial.__version__}")
        log.log("环境-pyserial", "PASS")
    except ImportError:
        print("  ❌ pyserial 未安装! pip install pyserial")
        log.log("环境-pyserial", "FAIL")
        return False

    from serial.tools import list_ports
    ports = list(list_ports.comports())
    print(f"  可用串口: {[(p.device, p.description) for p in ports]}")
    log.log("环境-串口", "PASS", str([p.device for p in ports]))

    if len(ports) < 1:
        print("  ❌ 没有串口!")
        return False
    if len(ports) < 2:
        print("  ⚠️ 只发现1个串口。IMU和舵机需要各一个串口!")
    return True


def step_1_connect(driver, log):
    """Step 1: 连接 IMU"""
    print("\n" + "="*60)
    print("  STEP 1: 连接 YIS321 IMU")
    print("="*60)
    print(f"  串口: {driver.port}  波特率: {driver.baudrate}")
    print()
    print("  确认清单:")
    print("     □ IMU 模块已通过 USB-TTL 连接到电脑")
    print("     □ IMU 模块已供电 (红色电源灯亮)")
    print("     □ 接线: TX→RX, RX→TX, GND→GND")
    print()
    print("  注意: IMU串口 ≠ 舵机串口! 它们是两个不同的 COM 口")
    print()

    wait_input("确认就绪后按 Enter")

    try:
        driver.open()
        log.log("连接", "PASS", f"{driver.port} @ {driver.baudrate}")
        return True
    except Exception as e:
        print(f"  连接失败: {e}")
        log.log("连接", "FAIL", str(e))
        print("\n  排查:")
        print("     1. 确认串口号 (设备管理器中查看)")
        print("     2. 确认波特率 (YIS321 默认 921600)")
        print("     3. 关闭其他占用串口的软件")
        return False


def step_2_first_data(driver, log):
    """Step 2: 首次数据读取"""
    print("\n" + "="*60)
    print("  STEP 2: 首次数据读取")
    print("="*60)
    print("  等待 IMU 数据包 (最多 5 秒)...")

    data = driver.read_blocking(timeout=5.0)

    if data is None:
        print("  5 秒内没有收到数据!")
        log.log("首次读取", "FAIL", "超时")
        print("\n  排查:")
        print("     1. 检查 TX/RX 是否接反")
        print("     2. 确认波特率正确 (试试 --baud 115200 或 --baud 921600)")
        print("     3. 检查 IMU 是否有数据输出 (指示灯是否闪烁)")
        return False

    print(f"  收到数据!")
    print(f"  {data}")
    print(f"\n  帧统计: 成功={driver.frame_count}  错误={driver.error_count}")
    log.log("首次读取", "PASS", f"帧数={driver.frame_count}")
    return True


def step_3_data_rate(driver, log):
    """Step 3: 数据速率测试"""
    print("\n" + "="*60)
    print("  STEP 3: 数据速率测试 (持续2秒)")
    print("="*60)
    print("  IMU 放在桌上不动, 统计帧率和错误率...")
    print()

    wait_input("将 IMU 平放在桌上, 按 Enter 开始")

    start_frames = driver.frame_count
    start_errors = driver.error_count
    start_time = time.time()
    duration = 2.0

    while time.time() - start_time < duration:
        driver.read()
        time.sleep(0.0005)

    elapsed = time.time() - start_time
    frames = driver.frame_count - start_frames
    errors = driver.error_count - start_errors
    fps = frames / elapsed

    print(f"  帧数: {frames}  错误: {errors}  速率: {fps:.1f} Hz")
    print(f"  错误率: {errors/(frames+errors)*100:.1f}%" if frames+errors > 0 else "")

    if fps > 50:
        log.log("帧率", "PASS", f"{fps:.1f} Hz ({frames} frames in {elapsed:.1f}s)")
    elif fps > 10:
        log.log("帧率", "WARN", f"{fps:.1f} Hz (偏低)")
        print("  ⚠️ 帧率偏低, 但可用")
    else:
        log.log("帧率", "FAIL", f"{fps:.1f} Hz")
        print("  ❌ 帧率太低!")
        return False

    if errors > frames * 0.1:
        log.log("错误率", "WARN", f"{errors} errors / {frames} frames")
        print("  ⚠️ 错误率较高, 检查接线")
    else:
        log.log("错误率", "PASS", f"{errors} / {frames}")

    return True


def step_4_static(driver, log):
    """Step 4: 静态精度测试"""
    print("\n" + "="*60)
    print("  STEP 4: 静态精度测试")
    print("="*60)
    print("  IMU 平放桌上3秒, 测量静态漂移")
    print()

    wait_input("确认 IMU 平稳放置后按 Enter")

    samples = []
    start = time.time()
    while time.time() - start < 3.0:
        d = driver.read()
        if d:
            samples.append(d)
        time.sleep(0.001)

    if len(samples) < 10:
        log.log("静态精度", "FAIL", f"仅 {len(samples)} 采样")
        return False

    # 计算平均值和标准差
    n = len(samples)
    avg_pitch = sum(s.euler[0] for s in samples) / n
    avg_roll = sum(s.euler[1] for s in samples) / n
    avg_yaw = sum(s.euler[2] for s in samples) / n

    std_pitch = math.sqrt(sum((s.euler[0] - avg_pitch)**2 for s in samples) / n)
    std_roll = math.sqrt(sum((s.euler[1] - avg_roll)**2 for s in samples) / n)

    avg_gz = sum(s.gyro[2] for s in samples) / n

    print(f"  采样数: {n}")
    print(f"  平均欧拉角:")
    print(f"    Pitch: {math.degrees(avg_pitch):+.2f}°  (抖动: ±{math.degrees(std_pitch):.3f}°)")
    print(f"    Roll:  {math.degrees(avg_roll):+.2f}°  (抖动: ±{math.degrees(std_roll):.3f}°)")
    print(f"    Yaw:   {math.degrees(avg_yaw):+.2f}°")
    print(f"  陀螺仪 Z 轴漂移: {math.degrees(avg_gz):.4f}°/s")

    # 静态时 pitch/roll 应该接近 0 (平放)
    if abs(math.degrees(avg_pitch)) < 5 and abs(math.degrees(avg_roll)) < 5:
        log.log("静态-水平", "PASS",
                f"Pitch={math.degrees(avg_pitch):.1f}° Roll={math.degrees(avg_roll):.1f}°")
    else:
        log.log("静态-水平", "WARN",
                f"Pitch={math.degrees(avg_pitch):.1f}° Roll={math.degrees(avg_roll):.1f}° (偏大, IMU可能没放平)")

    if math.degrees(std_pitch) < 0.5 and math.degrees(std_roll) < 0.5:
        log.log("静态-抖动", "PASS",
                f"σ_pitch={math.degrees(std_pitch):.3f}° σ_roll={math.degrees(std_roll):.3f}°")
    else:
        log.log("静态-抖动", "WARN",
                f"抖动偏大: σ={math.degrees(std_pitch):.3f}°")

    return True


def step_5_tilt(driver, log):
    """Step 5: 倾斜响应测试"""
    print("\n" + "="*60)
    print("  STEP 5: 倾斜响应测试")
    print("="*60)
    print("  目的: 验证 IMU 方向轴是否正确")
    print()

    tests = [
        ("请将 IMU 向前倾斜约 30° (保持3秒)",
         "pitch", "前倾时 Pitch 应有约 30° 变化"),
        ("请将 IMU 向右倾斜约 30° (保持3秒)",
         "roll", "右倾时 Roll 应有约 30° 变化"),
        ("请将 IMU 水平顺时针旋转约 90° (保持3秒)",
         "yaw", "右转时 Yaw 应有约 90° 变化"),
    ]

    for instruction, axis, expect in tests:
        print(f"\n  ── 测试: {axis.upper()} 轴 ──")
        print(f"    {expect}")
        print()

        # 先读基准
        wait_input("先将 IMU 放平, 按 Enter 记录基准")
        base_samples = []
        t0 = time.time()
        while time.time() - t0 < 1.0:
            d = driver.read()
            if d: base_samples.append(d)
            time.sleep(0.001)

        if not base_samples:
            log.log(f"倾斜-{axis}", "FAIL", "无基准数据")
            continue

        base_euler = [sum(s.euler[i] for s in base_samples) / len(base_samples) for i in range(3)]
        print(f"    基准: Pitch={math.degrees(base_euler[0]):.1f}° "
              f"Roll={math.degrees(base_euler[1]):.1f}° Yaw={math.degrees(base_euler[2]):.1f}°")

        # 倾斜测试
        wait_input(f"    {instruction}\n    👉 准备好后按 Enter 开始采集")

        print("    采集中 (3秒)...")
        tilt_samples = []
        t0 = time.time()
        while time.time() - t0 < 3.0:
            d = driver.read()
            if d:
                tilt_samples.append(d)
                # 实时显示
                ed = d.euler_deg
                print(f"\r    Pitch={ed[0]:+7.2f}° Roll={ed[1]:+7.2f}° Yaw={ed[2]:+7.2f}°   ",
                      end="", flush=True)
            time.sleep(0.01)
        print()

        if not tilt_samples:
            log.log(f"倾斜-{axis}", "FAIL", "无倾斜数据")
            continue

        tilt_euler = [sum(s.euler[i] for s in tilt_samples) / len(tilt_samples) for i in range(3)]

        # 计算角度变化
        axis_idx = {"pitch": 0, "roll": 1, "yaw": 2}[axis]
        delta = math.degrees(tilt_euler[axis_idx] - base_euler[axis_idx])
        print(f"    {axis.upper()} 变化: {delta:+.1f}°")

        result = ask_choice(f"    {axis.upper()} 轴是否响应正确?", [
            "正确 — 方向和幅度基本对",
            "方向反了 — 应该正但显示负 (或反过来)",
            "没有反应 — 变化很小",
            "轴搞混了 — 变化出现在其他轴上",
        ])

        if "正确" in result:
            log.log(f"倾斜-{axis}", "PASS", f"delta={delta:+.1f}°")
        else:
            log.log(f"倾斜-{axis}", "FAIL", f"delta={delta:+.1f}° 反馈={result}")

    return True


def step_6_live(driver, log):
    """Step 6: 实时数据流"""
    print("\n" + "="*60)
    print("  STEP 6: 实时数据流显示")
    print("="*60)
    print("  显示实时IMU数据, 可以拿起来转动感受一下")
    print("  按 Ctrl+C 退出此步骤")
    print()

    wait_input("按 Enter 开始")

    start = time.time()
    try:
        while True:
            d = driver.read()
            now = time.time()
            if d:
                ed = d.euler_deg
                gyro = d.gyro
                print(f"\r  [{driver.frame_count:>6}] "
                      f"Pitch={ed[0]:+7.2f}° Roll={ed[1]:+7.2f}° Yaw={ed[2]:+7.2f}° | "
                      f"ωx={gyro[0]:+6.3f} ωy={gyro[1]:+6.3f} ωz={gyro[2]:+6.3f} rad/s  ",
                      end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        elapsed = time.time() - start
        print(f"\n\n  观察了 {elapsed:.0f} 秒")

    log.log("实时流", "PASS", f"观察了 {time.time()-start:.0f}s")

    result = ask_choice("IMU 数据是否感觉正确?", [
        "正确 — 旋转时数据变化符合直觉",
        "基本正确 — 有些小问题",
        "不对 — 数据和动作不匹配",
    ])
    log.log("实时感受", "PASS" if "正确" in result and "不对" not in result else "WARN", result)
    return True


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    parser = argparse.ArgumentParser(description="YIS321 IMU 完整测试")
    parser.add_argument("--port", default="COM15", help="IMU 串口名")
    parser.add_argument("--baud", type=int, default=921600, help="波特率")
    args = parser.parse_args()

    print()
    print("==========================================================")
    print("        YIS321 IMU 完整测试 — 逐步验证                    ")
    print("==========================================================")
    print("                                                        ")
    print("  Step 0: 环境检查                                       ")
    print("  Step 1: 连接 IMU 串口                                  ")
    print("  Step 2: 首次数据读取                                    ")
    print("  Step 3: 帧率和错误率                                    ")
    print("  Step 4: 静态精度 (平放桌上)                             ")
    print("  Step 5: 倾斜响应 (Pitch/Roll/Yaw 方向验证)              ")
    print("  Step 6: 实时数据流                                     ")
    print("                                                        ")
    print("  IMU串口 ≠ 舵机串口! 注意选对端口!                     ")
    print("==========================================================")
    print()

    log = TestLog()
    driver = YIS321Driver(args.port, args.baud)
    report_file = os.path.join(
        os.path.dirname(__file__), "..",
        f"imu_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )

    try:
        if not step_0_env(log): return
        if not step_1_connect(driver, log): return
        if not step_2_first_data(driver, log): return
        step_3_data_rate(driver, log)
        step_4_static(driver, log)
        step_5_tilt(driver, log)
        step_6_live(driver, log)

    except KeyboardInterrupt:
        print("\n\n  测试中断!")
        log.log("中断", "WARN", "Ctrl+C")
    except Exception as e:
        print(f"\n  异常: {e}")
        traceback.print_exc()
        log.log("异常", "FAIL", str(e))
    finally:
        driver.close()
        log.summary()
        log.save(report_file)
        print(f"\n  请把报告文件发回: {os.path.abspath(report_file)}")


if __name__ == "__main__":
    main()
