#!/usr/bin/env python3
"""
teach.py — 试教录制交互工具

交互式终端界面，用于录制和回放机器人动作。

用法:
    python tools/teach.py --port COM3
"""

import argparse
import sys
import os
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.robot import Robot
from src.robot_config import ROBOT_CONFIG
from src.recorder.motion_data import MotionClip
from src.recorder.recorder import MotionRecorder
from src.recorder.player import MotionPlayer

MOTIONS_DIR = Path(__file__).parent.parent / "motions"


def print_help():
    print("""
╔══════════════════════════════════════════════════╗
║          🎬 舵机机器人试教录制工具                 ║
╠══════════════════════════════════════════════════╣
║  录制命令:                                        ║
║    record <名称>     开始新录制                    ║
║    c / capture       捕获当前帧                    ║
║    undo              撤销上一帧                    ║
║    finish            结束录制并保存                 ║
║    cancel            取消录制                      ║
║                                                    ║
║  回放命令:                                        ║
║    play <文件> [速度] 回放动作 (速度默认1.0)        ║
║    loop <文件> [速度] [次数] 循环回放               ║
║                                                    ║
║  其他:                                            ║
║    list               列出已保存的动作              ║
║    home               所有关节回零位                ║
║    unload             卸载所有舵机                  ║
║    load               加载所有舵机                  ║
║    read               读取当前位置                  ║
║    help               显示帮助                     ║
║    quit               退出                         ║
╚══════════════════════════════════════════════════╝
""")


def list_motions():
    files = sorted(MOTIONS_DIR.glob("*.json"))
    if not files:
        print("  (无已保存的动作文件)")
        return
    print(f"\n📂 已保存的动作 ({MOTIONS_DIR}):")
    for f in files:
        try:
            clip = MotionClip.load(f)
            print(f"  📄 {f.name:30s}  {clip.frame_count} 帧  {clip.total_duration_ms}ms")
        except Exception:
            print(f"  ⚠️ {f.name:30s}  (读取失败)")


def main():
    parser = argparse.ArgumentParser(description="试教录制工具")
    parser.add_argument("--port", required=True, help="串口名")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    robot = Robot(args.port, ROBOT_CONFIG, args.baud)
    robot.connect()

    recorder = MotionRecorder(robot)
    player = MotionPlayer(robot)

    print_help()

    try:
        while True:
            prompt = "🔴 录制中 >>> " if recorder.is_recording else ">>> "
            try:
                line = input(prompt).strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            # ── 录制命令 ──
            if cmd == "record":
                if len(parts) < 2:
                    print("  用法: record <名称>")
                    continue
                name = parts[1]
                desc = " ".join(parts[2:]) if len(parts) > 2 else ""
                try:
                    recorder.start(name, desc)
                except RuntimeError as e:
                    print(f"  ❌ {e}")

            elif cmd in ("c", "capture"):
                try:
                    recorder.capture()
                except RuntimeError as e:
                    print(f"  ❌ {e}")

            elif cmd == "undo":
                try:
                    recorder.undo()
                except RuntimeError as e:
                    print(f"  ❌ {e}")

            elif cmd == "finish":
                try:
                    clip = recorder.finish()
                    filepath = MOTIONS_DIR / f"{clip.name}.json"
                    clip.save(filepath)
                except RuntimeError as e:
                    print(f"  ❌ {e}")

            elif cmd == "cancel":
                recorder.cancel()

            # ── 回放命令 ──
            elif cmd == "play":
                if len(parts) < 2:
                    print("  用法: play <文件名> [速度]")
                    continue
                fname = parts[1]
                speed = float(parts[2]) if len(parts) > 2 else 1.0
                fpath = MOTIONS_DIR / fname
                if not fpath.suffix:
                    fpath = fpath.with_suffix(".json")
                if not fpath.exists():
                    print(f"  ❌ 文件不存在: {fpath}")
                    continue
                clip = MotionClip.load(fpath)
                player.play(clip, speed)

            elif cmd == "loop":
                if len(parts) < 2:
                    print("  用法: loop <文件名> [速度] [次数]")
                    continue
                fname = parts[1]
                speed = float(parts[2]) if len(parts) > 2 else 1.0
                count = int(parts[3]) if len(parts) > 3 else 3
                fpath = MOTIONS_DIR / fname
                if not fpath.suffix:
                    fpath = fpath.with_suffix(".json")
                if not fpath.exists():
                    print(f"  ❌ 文件不存在: {fpath}")
                    continue
                clip = MotionClip.load(fpath)
                player.play_loop(clip, speed, count)

            # ── 其他命令 ──
            elif cmd == "list":
                list_motions()
            elif cmd == "home":
                robot.go_home(1500)
            elif cmd == "unload":
                robot.unload_all()
            elif cmd == "load":
                robot.load_all()
            elif cmd == "read":
                for name, pos in robot.get_all_positions().items():
                    print(f"  {name:25s} = {pos if pos is not None else 'N/A'}")
            elif cmd == "help":
                print_help()
            elif cmd in ("quit", "q", "exit"):
                break
            else:
                print(f"  ❌ 未知命令: {cmd}  (输入 help 查看帮助)")

    finally:
        if recorder.is_recording:
            recorder.cancel()
        robot.disconnect()


if __name__ == "__main__":
    main()
