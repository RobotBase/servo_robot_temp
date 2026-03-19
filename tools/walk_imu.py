"""
walk_imu.py — 闭环 UVC 步行控制 (舵机 + IMU 双串口)
════════════════════════════════════════════════════════

基于 servo_robot_learn 项目的 UVC (上体垂直制御) 算法，
使用 IMU 实时反馈修正步态，实现闭环平衡控制。

架构:
    PC
    ├── COM3 (115200)  → 舵机总线 (LX-15D × 10)
    └── COM5 (460800)  → YIS321 IMU

用法:
    python tools/walk_imu.py --servo-port COM3 --imu-port COM5
    python tools/walk_imu.py --servo-port COM3 --imu-port COM5 --mode balance
    python tools/walk_imu.py --servo-port COM3 --imu-port COM5 --mode walk
"""

import argparse
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.robot import Robot
from src.yis321_driver import YIS321Driver, IMUData


# ╔══════════════════════════════════════════════════════════════╗
# ║                    物理参数 (mm)                              ║
# ╚══════════════════════════════════════════════════════════════╝

THIGH_LEN = 65.0
SHIN_LEN = 65.0
LEG_LEN = 130.0
LATERAL_OFFSET = 64.5
HEIGHT = 190.0

SERVO_UNITS_PER_DEG = 1.0 / 0.24

DEFAULT_ZERO = {
    "left_hip_yaw": 446, "left_hip_pitch": 809, "left_knee": 462,
    "left_ankle_pitch": 431, "left_ankle_roll": 560,
    "right_hip_yaw": 380, "right_hip_pitch": 475, "right_knee": 465,
    "right_ankle_pitch": 812, "right_ankle_roll": 635,
}

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


# ╔══════════════════════════════════════════════════════════════╗
# ║                   逆运动学 (footCont)                        ║
# ╚══════════════════════════════════════════════════════════════╝

def foot_cont(x, y, h):
    inner = math.sqrt(y*y + h*h) - LATERAL_OFFSET
    if inner < 0: inner = 0
    k = math.sqrt(x*x + inner*inner)
    if k > LEG_LEN - 1: k = LEG_LEN - 1
    k0 = math.acos(k / LEG_LEN)
    x0 = math.asin(max(-1, min(1, x/k))) if k > 0.001 else 0
    k1 = math.atan2(y, h) if h > 0.001 else 0
    return (k0+x0, k1, k0*2, k0-x0, -k1)


STAND_ANGLES = list(foot_cont(0, 0, HEIGHT))


def ik_to_servo(ik_angles, side, zero):
    result = {}
    for jshort, (zkey, idx, d) in JOINT_MAP[side].items():
        delta = math.degrees(ik_angles[idx] - STAND_ANGLES[idx]) * SERVO_UNITS_PER_DEG
        result[f"{side}_{jshort}"] = int(max(0, min(1000, zero[zkey] + d * delta)))
    return result


# ╔══════════════════════════════════════════════════════════════╗
# ║                 UVC 上体垂直制御 (核心算法)                    ║
# ║    移植自 robot_ctrl.c 的 uvc() 函数                          ║
# ╚══════════════════════════════════════════════════════════════╝

class UVCController:
    """UVC 闭环平衡控制器"""

    def __init__(self):
        # UVC 积分变量
        self.dxi = 0.0    # 支撑腿前后偏移 (mm)
        self.dyi = 0.0    # 支撑腿横向偏移 (mm)
        self.dxis = 0.0   # 摆动腿前后偏移
        self.dyis = 0.0   # 摆动腿横向偏移
        self.dxib = 0.0   # 前一步前后偏移备份
        self.dyib = 0.0   # 前一步横向偏移备份
        self.autoH = HEIGHT  # 自动调整高度

        # 步态状态
        self.jikuasi = 0  # 支撑腿 (0=左, 1=右)
        self.fwct = 0.0   # 步态周期计数
        self.fwctEnd = 48.0  # 半周期长度
        self.fwctUp = 1.0    # 计数增量
        self.fh = 0.0        # 抬脚高度
        self.fhMax = 15.0    # 最大抬脚高度
        self.sw = 0.0        # 横向摆偏移
        self.swx = 0.0
        self.swy = 0.0
        self.swMax = 12.0    # 最大横向摆幅

        # 着地缓冲
        self.landF = 8       # 前着地周期数
        self.landB = 8       # 后着地周期数

        # UVC 增益
        self.uvc_gain_roll = 0.85   # 横滚增益
        self.uvc_gain_pitch = 0.85  # 俯仰增益
        self.dead_zone = 0.033      # 死区 (rad, ≈2°)

    def reset(self):
        """复位所有状态"""
        self.dxi = self.dyi = 0
        self.dxis = self.dyis = 0
        self.dxib = self.dyib = 0
        self.autoH = HEIGHT
        self.jikuasi = 0
        self.fwct = 0
        self.fh = 0
        self.sw = self.swx = self.swy = 0

    def uvc(self, pitch: float, roll: float):
        """UVC 主控制: 根据 IMU 数据修正腿部位置

        Args:
            pitch: 俯仰角 (rad), 前倾为负
            roll:  横滚角 (rad), 右倾为正
        """
        # 死区处理
        k = math.sqrt(pitch*pitch + roll*roll)
        if k > self.dead_zone:
            scale = (k - self.dead_zone) / k
            pitch *= scale
            roll *= scale
        else:
            pitch = 0
            roll = 0

        # 系数
        rollt = self.uvc_gain_roll * roll
        if self.jikuasi == 0:
            rollt = -rollt
        pitcht = self.uvc_gain_pitch * pitch

        # 仅在单脚支撑期执行 UVC
        if self.fwct > self.landF and self.fwct <= self.fwctEnd - self.landB:
            # ── 横向修正 (Roll) ──
            k = math.atan2(self.dyi - self.sw, self.autoH)
            kl = self.autoH / math.cos(k) if abs(math.cos(k)) > 0.01 else self.autoH
            ks = k + rollt
            k_new = kl * math.sin(ks)
            self.dyi = k_new + self.sw
            self.autoH = kl * math.cos(ks)

            # ── 纵向修正 (Pitch) ──
            k = math.atan2(self.dxi, self.autoH)
            kl = self.autoH / math.cos(k) if abs(math.cos(k)) > 0.01 else self.autoH
            ks = k + pitcht
            self.dxi = kl * math.sin(ks)
            self.autoH = kl * math.cos(ks)

            # 限制
            self.dyi = max(0, min(45, self.dyi))
            self.dxi = max(-45, min(45, self.dxi))

            # 游脚跟随
            self.dyis = self.dyi
            self.dxis = -self.dxi

            # 双脚内侧平行修正
            if self.jikuasi == 0:
                k_r = -self.sw + self.dyi
                k_l = self.sw + self.dyis
            else:
                k_l = -self.sw + self.dyi
                k_r = self.sw + self.dyis
            if k_r + k_l < 0:
                self.dyis -= k_r + k_l

    def uvc_sub(self):
        """UVC 辅助: 支撑腿恢复垂直, 腿长控制"""
        RST_F = 3.0
        if self.fwct <= self.landF:
            # 横向恢复
            k = self.dyi / (11 - self.fwct) if (11 - self.fwct) > 0 else 0
            self.dyi -= k
            self.dyis += k
            # 纵向恢复
            if self.dxi > RST_F:
                self.dxi -= RST_F; self.dxis -= RST_F
            elif self.dxi < -RST_F:
                self.dxi += RST_F; self.dxis += RST_F
            else:
                self.dxis -= self.dxi; self.dxi = 0

        # 限幅
        self.dyis = min(70, self.dyis)
        self.dxis = max(-70, min(70, self.dxis))

        # 腿长恢复
        if HEIGHT > self.autoH:
            self.autoH += (HEIGHT - self.autoH) * 0.07
        else:
            self.autoH = HEIGHT
        self.autoH = max(140, self.autoH)

    def foot_up(self):
        """抬脚轨迹"""
        if self.fwct > self.landF and self.fwct <= self.fwctEnd - self.landB:
            phase = (self.fwct - self.landF) / (self.fwctEnd - self.landF - self.landB)
            self.fh = self.fhMax * math.sin(math.pi * phase)
        else:
            self.fh = 0

    def sw_cont(self):
        """横向摆动"""
        k = self.swMax * math.sin(math.pi * self.fwct / self.fwctEnd)
        self.swy = k
        self.swx = 0

    def counter_cont(self):
        """周期计数器"""
        if self.fwct >= self.fwctEnd:
            self.jikuasi ^= 1
            self.fwct = 0
            self.fh = 0
            # 交换支撑/摆动腿数据
            self.dyi, self.dyis = self.dyis, self.dyi
            self.dyib = self.dyi
            self.dxi, self.dxis = self.dxis, self.dxi
            self.dxib = self.dxi
        else:
            self.fwct += self.fwctUp
            if self.fwct > self.fwctEnd:
                self.fwct = self.fwctEnd

    def compute_feet(self, zero):
        """计算双脚舵机值

        Returns:
            {关节名: 原始舵机值} dict
        """
        # 根据支撑腿选择左右脚参数
        if self.jikuasi == 0:
            # 左脚支撑
            left_x = self.dxi - self.swx
            left_y = self.dyi - self.swy
            left_h = self.autoH
            right_x = self.dxis - self.swx
            right_y = self.dyis + self.swy
            right_h = self.autoH - self.fh
        else:
            # 右脚支撑
            left_x = self.dxis - self.swx
            left_y = self.dyis + self.swy
            left_h = self.autoH - self.fh
            right_x = self.dxi - self.swx
            right_y = self.dyi - self.swy
            right_h = self.autoH

        left_h = max(140, left_h)
        right_h = max(140, right_h)

        la = foot_cont(left_x, left_y, left_h)
        ra = foot_cont(right_x, -right_y, right_h)

        frame = {}
        frame.update(ik_to_servo(la, "left", zero))
        frame.update(ik_to_servo(ra, "right", zero))
        return frame


# ╔══════════════════════════════════════════════════════════════╗
# ║                 IMU 读取线程                                  ║
# ╚══════════════════════════════════════════════════════════════╝

class IMUThread:
    """后台线程持续读取 IMU 数据"""

    def __init__(self, driver: YIS321Driver):
        self.driver = driver
        self.latest: IMUData | None = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def get(self) -> IMUData | None:
        with self._lock:
            return self.latest

    def _run(self):
        while self._running:
            d = self.driver.read()
            if d:
                with self._lock:
                    self.latest = d
            time.sleep(0.001)


# ╔══════════════════════════════════════════════════════════════╗
# ║                    运行模式                                   ║
# ╚══════════════════════════════════════════════════════════════╝

def load_zero(filepath=None):
    if filepath and os.path.exists(filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        if data.get("keyframes"):
            print(f"✅ 零位已加载: {filepath}")
            return data["keyframes"][0]["positions"]
    return DEFAULT_ZERO.copy()


def do_balance(robot, zero, imu_thread, duration=30):
    """静态平衡模式: 站立 + UVC 修正 (不迈步)"""
    print(f"\n⚖️ 静态平衡模式 ({duration}s)")
    print("   机器人站立, 用 IMU 数据保持平衡")
    print("   你可以轻推机器人, 观察它的平衡反应")
    print("   按 Ctrl+C 停止\n")

    uvc = UVCController()
    uvc.fwctEnd = 999  # 不迈步
    dt = 0.02
    t = 0

    robot.set_joints_raw(zero, 1500)
    time.sleep(2)

    try:
        while t < duration:
            loop_start = time.time()

            imu = imu_thread.get()
            if imu:
                # 映射 IMU 轴到 robot_ctrl.c 的约定
                pitch = -imu.euler[0]  # 前倾为负
                roll = imu.euler[1]    # 右倾为正

                uvc.uvc(pitch, roll)
                uvc.uvc_sub()

                frame = uvc.compute_feet(zero)
                robot.set_joints_raw(frame, int(dt * 1000))

                if int(t / 0.5) != int((t - dt) / 0.5):
                    print(f"\r  t={t:.1f}s  pitch={math.degrees(pitch):+.1f}°  "
                          f"roll={math.degrees(roll):+.1f}°  "
                          f"dyi={uvc.dyi:.1f}  dxi={uvc.dxi:.1f}  "
                          f"H={uvc.autoH:.0f}  ", end="", flush=True)

            t += dt
            elapsed = time.time() - loop_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\n\n⏹ 中断!")

    robot.set_joints_raw(zero, 1000)
    time.sleep(1.5)


def do_walk_uvc(robot, zero, imu_thread, steps=6, period=1.2):
    """闭环步行: CPG + UVC"""
    print(f"\n🚶 UVC 闭环步行: {steps}步, 周期{period}s")
    print("   IMU 实时修正平衡!")
    print("   ⚠️ 用手保护机器人!")
    print("   按 Ctrl+C 停止\n")

    uvc = UVCController()
    uvc.fwctEnd = 48
    uvc.fwctUp = 1
    uvc.fhMax = 15
    uvc.swMax = 12
    uvc.landF = 8
    uvc.landB = 8
    dt = 0.02

    half_period = period / 2.0
    total_time = steps * half_period

    robot.set_joints_raw(zero, 1500)
    time.sleep(2)

    # 先检测初始倾斜, 选择支撑腿
    imu = imu_thread.get()
    if imu:
        if imu.euler[1] > 0:  # 右倾
            uvc.jikuasi = 1
        else:
            uvc.jikuasi = 0
    uvc.fwct = 1

    t = 0
    start = time.time()

    try:
        while t < total_time:
            loop_start = time.time()

            imu = imu_thread.get()
            if imu:
                pitch = -imu.euler[0]
                roll = imu.euler[1]

                # 跌倒检测
                if abs(pitch) > 0.35 or abs(roll) > 0.35:
                    print(f"\n\n  ❌ 跌倒检测! pitch={math.degrees(pitch):.0f}° "
                          f"roll={math.degrees(roll):.0f}°")
                    break

                # UVC 闭环
                uvc.uvc(pitch, roll)
                uvc.uvc_sub()
                uvc.foot_up()
                uvc.sw_cont()
                uvc.counter_cont()

                frame = uvc.compute_feet(zero)
                robot.set_joints_raw(frame, int(dt * 1000))

                if int(t / 0.5) != int((t - dt) / 0.5):
                    step_n = int(t / half_period) + 1
                    print(f"  步{step_n}/{steps}  t={t:.1f}s  "
                          f"P={math.degrees(pitch):+.1f}° R={math.degrees(roll):+.1f}°  "
                          f"jiku={uvc.jikuasi}  fwct={uvc.fwct:.0f}  "
                          f"dyi={uvc.dyi:.1f}  dxi={uvc.dxi:.1f}")

            t += dt
            elapsed = time.time() - loop_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\n\n⏹ 中断!")

    robot.set_joints_raw(zero, 1000)
    time.sleep(1.5)
    print(f"✅ 完成 (耗时 {time.time()-start:.1f}s)")


def main():
    parser = argparse.ArgumentParser(description="UVC 闭环步行 (舵机+IMU)")
    parser.add_argument("--servo-port", default="COM3", help="舵机串口")
    parser.add_argument("--imu-port", default="COM5", help="IMU 串口")
    parser.add_argument("--imu-baud", type=int, default=460800, help="IMU 波特率")
    parser.add_argument("--mode", default="walk",
                        choices=["stand", "balance", "walk"],
                        help="模式: stand=站立, balance=静态平衡, walk=步行")
    parser.add_argument("--steps", type=int, default=6)
    parser.add_argument("--period", type=float, default=1.2)
    parser.add_argument("--zero-file", default=None)
    args = parser.parse_args()

    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║       🤖 UVC 闭环步行控制 (舵机 + IMU 双串口)            ║")
    print("╠══════════════════════════════════════════════════════════╣")
    print(f"║  舵机: {args.servo_port:8s} (115200)                           ║")
    print(f"║  IMU:  {args.imu_port:8s} ({args.imu_baud})                         ║")
    print(f"║  模式: {args.mode:8s}                                     ║")
    print("╚══════════════════════════════════════════════════════════╝")

    zf = args.zero_file
    if zf is None:
        for c in [os.path.join(os.path.dirname(__file__), "..", "零位.json"),
                  os.path.join(os.path.dirname(__file__), "..", "motions", "零位.json")]:
            if os.path.exists(c):
                zf = c; break
    zero = load_zero(zf)

    robot = Robot(port=args.servo_port)
    imu_driver = YIS321Driver(args.imu_port, args.imu_baud)

    try:
        # 连接
        print("\n连接舵机...")
        robot.connect()
        print("连接 IMU...")
        imu_driver.open()

        # 等待 IMU 数据
        print("等待 IMU 首帧数据...")
        first = imu_driver.read_blocking(timeout=5)
        if first is None:
            print("❌ IMU 无数据! 检查接线和串口号")
            return
        print(f"✅ IMU 在线: pitch={first.euler_deg[0]:.1f}° "
              f"roll={first.euler_deg[1]:.1f}° yaw={first.euler_deg[2]:.1f}°")

        # 启动 IMU 后台线程
        imu_thread = IMUThread(imu_driver)
        imu_thread.start()

        if args.mode == "stand":
            print("\n🧍 站立模式")
            robot.set_joints_raw(zero, 1500)
            time.sleep(2)
            print("站立中... 按 Ctrl+C 退出")
            while True:
                imu = imu_thread.get()
                if imu:
                    ed = imu.euler_deg
                    print(f"\r  Pitch={ed[0]:+7.2f}° Roll={ed[1]:+7.2f}° Yaw={ed[2]:+7.2f}°  ",
                          end="", flush=True)
                time.sleep(0.1)

        elif args.mode == "balance":
            do_balance(robot, zero, imu_thread, duration=60)

        elif args.mode == "walk":
            do_walk_uvc(robot, zero, imu_thread,
                        steps=args.steps, period=args.period)

    except KeyboardInterrupt:
        print("\n\n⏹ 停止!")
        try:
            robot.set_joints_raw(zero, 1000)
            time.sleep(1.5)
        except: pass
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback; traceback.print_exc()
    finally:
        try:
            if 'imu_thread' in dir():
                imu_thread.stop()
            imu_driver.close()
            robot.disconnect()
        except: pass
        print("👋 已断开")


if __name__ == "__main__":
    main()
