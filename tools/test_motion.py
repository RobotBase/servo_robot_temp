#!/usr/bin/env python3
"""
test_motion.py — 基础运动测试工具

用法:
    python tools/test_motion.py --port COM3
    python tools/test_motion.py --port COM3 --test home
"""

import argparse
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.robot import Robot
from src.robot_config import ROBOT_CONFIG


def test_home(robot):
    print("\n🏠 测试: 所有关节回零位")
    robot.go_home(time_ms=2000)
    time.sleep(2.5)
    positions = robot.get_all_positions()
    print("\n当前位置:")
    for name, pos in positions.items():
        joint = robot.config.get_joint(name)
        if pos is not None:
            diff = abs(pos - joint.home_pos)
            s = "✅" if diff < 20 else "⚠️"
            print(f"  {s} {name:25s}  目标={joint.home_pos:4d}  实际={pos:4d}  偏差={diff}")
        else:
            print(f"  ❌ {name:25s}  读取失败")


def test_sweep(robot):
    print("\n🔄 测试: 各关节正反向扫描")
    robot.go_home(time_ms=1500)
    time.sleep(2)
    R, T = 150, 800
    for joint in robot.config.all_joints:
        print(f"\n  测试 {joint.name} (ID={joint.servo_id}):")
        fwd = min(joint.home_pos + R, joint.max_pos)
        print(f"    → 正向: {joint.home_pos} → {fwd}")
        robot.bus.move(joint.servo_id, fwd, T)
        time.sleep(T / 1000 + 0.3)
        robot.bus.move(joint.servo_id, joint.home_pos, T)
        time.sleep(T / 1000 + 0.3)
        rev = max(joint.home_pos - R, joint.min_pos)
        print(f"    ← 反向: {joint.home_pos} → {rev}")
        robot.bus.move(joint.servo_id, rev, T)
        time.sleep(T / 1000 + 0.3)
        robot.bus.move(joint.servo_id, joint.home_pos, T)
        time.sleep(T / 1000 + 0.3)
        print(f"    ✅ 完成")
    print("\n所有关节测试完成")


def test_interactive(robot):
    print("\n🎮 交互式测试")
    joints = robot.config.all_joints
    for i, j in enumerate(joints):
        print(f"  [{i+1:2d}] {j.name} (ID={j.servo_id})")
    print("\n  <编号> <位置> — 移动  |  home — 回零  |  read — 读取  |  quit — 退出\n")
    while True:
        try:
            cmd = input(">>> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue
        if cmd.lower() in ("quit", "q"):
            break
        elif cmd.lower() == "home":
            robot.go_home(1500)
        elif cmd.lower() == "read":
            for name, pos in robot.get_all_positions().items():
                print(f"  {name:25s} = {pos if pos is not None else 'N/A'}")
        else:
            parts = cmd.split()
            if len(parts) == 2:
                try:
                    idx, pos = int(parts[0]) - 1, int(parts[1])
                    if 0 <= idx < len(joints):
                        j = joints[idx]
                        pos = max(j.min_pos, min(j.max_pos, pos))
                        robot.bus.move(j.servo_id, pos, 500)
                        print(f"  {j.name} → {pos}")
                    else:
                        print(f"  ❌ 编号无效 (1~{len(joints)})")
                except ValueError:
                    print("  ❌ 格式: <编号> <位置>")
            else:
                print("  ❌ 未知命令")


def main():
    parser = argparse.ArgumentParser(description="舵机运动测试")
    parser.add_argument("--port", required=True, help="串口名")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--test", choices=["home", "sweep", "interactive", "all"], default="all")
    args = parser.parse_args()

    robot = Robot(args.port, ROBOT_CONFIG, args.baud)
    try:
        robot.connect()
        print("\n📡 扫描舵机...")
        results = robot.scan_servos()
        online = [r for r in results if r["online"]]
        print(f"  在线: {len(online)}/{len(results)}")
        for r in online:
            print(f"    ✅ ID={r['id']:2d} {r['name']}")
        if not online:
            print("  ⚠️ 无在线舵机")
            return
        if args.test in ("home", "all"):
            test_home(robot)
        if args.test in ("sweep", "all"):
            test_sweep(robot)
        if args.test in ("interactive", "all"):
            test_interactive(robot)
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        sys.exit(1)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
