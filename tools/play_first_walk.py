#!/usr/bin/env python3
"""
play_first_walk.py — 播放第一阶段里程碑步行动作

加载 motions/first_walk.json（手动示教录制的步行关键帧），
通过串口驱动机器人回放动作。这是项目第一个可用的步行演示。

用法:
    python tools/play_first_walk.py --port COM11
    python tools/play_first_walk.py --port COM11 --repeat 3 --speed 0.5
"""

import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.robot import Robot
from src.recorder.motion_data import MotionClip
from src.recorder.player import MotionPlayer

MOTION_FILE = os.path.join(os.path.dirname(__file__), "..", "motions", "first_walk.json")


def main():
    parser = argparse.ArgumentParser(
        description="播放第一阶段里程碑步行动作 (motions/first_walk.json)"
    )
    parser.add_argument("--port", default="COM11", help="串口 (默认 COM11)")
    parser.add_argument("--repeat", type=int, default=1, help="重复次数 (默认 1)")
    parser.add_argument("--speed", type=float, default=1.0, help="播放速度倍率 (默认 1.0)")
    parser.add_argument("--frames", type=int, default=None,
                        help="只播放前 N 帧 (默认全部)")
    args = parser.parse_args()

    # 加载动作文件
    motion_path = Path(MOTION_FILE)
    if not motion_path.exists():
        print(f"❌ 文件不存在: {motion_path}")
        return

    clip = MotionClip.load(motion_path)

    # 可选: 截取前 N 帧
    if args.frames and args.frames < clip.frame_count:
        clip = MotionClip(
            name=clip.name,
            description=clip.description,
            created_at=clip.created_at,
            keyframes=clip.keyframes[: args.frames],
        )
        print(f"📋 截取前 {args.frames} 帧")

    print(f"\n🤖 第一阶段步行动作回放")
    print(f"   文件: {motion_path.name}")
    print(f"   帧数: {clip.frame_count}")
    print(f"   时长: {clip.total_duration_ms}ms")
    print(f"   速度: {args.speed}x")
    print(f"   重复: {args.repeat} 次")

    # 连接机器人并播放
    robot = Robot(port=args.port)
    player = MotionPlayer(robot)

    try:
        robot.connect()
        print(f"\n📡 已连接: {args.port}")

        # 先回到第一帧姿态（站立）
        first_positions = clip.keyframes[0].positions
        print("\n⏳ 进入起始姿态...")
        robot.set_joints_raw(first_positions, 1500)
        time.sleep(2.0)

        # 播放动作
        player.play_loop(clip, speed=args.speed, count=args.repeat)

        # 回到起始姿态
        print("\n⏳ 回到起始姿态...")
        robot.set_joints_raw(first_positions, 1500)
        time.sleep(2.0)

        print("✅ 完成")

    except KeyboardInterrupt:
        print("\n⏹ 手动中断")
        player.stop()
    except Exception as e:
        print(f"\n❌ 错误: {e}")
    finally:
        try:
            robot.disconnect()
            print("📡 已断开连接")
        except Exception:
            pass


if __name__ == "__main__":
    main()
