#!/usr/bin/env python3
"""
play_motion.py — 播放并优化动作

加载指定的动作文件，优化帧时间使动作更流畅，然后重复播放指定次数。

用法:
    python tools/play_motion.py --file motions/1.json --repeat 3 --smooth
"""

import argparse
import os
import sys
from pathlib import Path

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.robot import Robot
from src.recorder.motion_data import MotionClip
from src.recorder.player import MotionPlayer

def optimize_timestamps(clip: MotionClip, target_duration_ms: int = 10000) -> MotionClip:
    """优化动作的时间戳，使帧间隔更均匀
    
    Args:
        clip: 原始动作数据
        target_duration_ms: 目标总时长 (默认10000ms = 10秒)
    
    Returns:
        优化时间戳后的新动作
    """
    if clip.frame_count < 2:
        return clip
    
    # 计算每帧的目标时间间隔 (ms)
    target_interval = target_duration_ms / (clip.frame_count - 1)
    
    # 创建新的关键帧列表，保持位置不变但重新分配时间戳
    optimized_keyframes = []
    for i, kf in enumerate(clip.keyframes):
        new_timestamp = int(i * target_interval)
        optimized_keyframes.append(
            type(kf)(
                timestamp_ms=new_timestamp,
                positions=kf.positions.copy()
            )
        )
    
    # 创建优化后的动作
    optimized_clip = MotionClip(
        name=f"{clip.name}_optimized",
        description=f"{clip.description} (optimized for {target_duration_ms}ms total duration)",
        created_at=clip.created_at,
        keyframes=optimized_keyframes
    )
    
    print(f"\n优化时间戳完成:")
    print(f"   原始时长: {clip.total_duration_ms}ms")
    print(f"   优化后时长: {optimized_clip.total_duration_ms}ms")
    print(f"   目标总时长: {target_duration_ms}ms")
    print(f"   每帧间隔: {target_interval:.1f}ms")
    
    return optimized_clip

def main():
    parser = argparse.ArgumentParser(description="播放并优化动作")
    parser.add_argument("--file", default="motions/1.json", help="动作文件路径")
    parser.add_argument("--port", default="COM11", help="串口")
    parser.add_argument("--repeat", type=int, default=3, help="重复次数")
    parser.add_argument("--smooth", action="store_true", help="优化时间戳")
    parser.add_argument("--duration", type=int, default=10000, help="优化后的目标总时长 (ms)")
    parser.add_argument("--speed", type=float, default=1.0, help="播放速度倍率")
    args = parser.parse_args()
    
    # 加载动作文件
    motion_file = Path(args.file)
    if not motion_file.exists():
        print(f"文件不存在: {motion_file}")
        return
    
    clip = MotionClip.load(motion_file)
    
    # 优化时间戳
    if args.smooth:
        clip = optimize_timestamps(clip, args.duration)
    
    # 初始化机器人和播放器
    robot = Robot(port=args.port)
    player = MotionPlayer(robot)
    
    try:
        robot.connect()
        print(f"\n机器人已连接: {args.port}")
        
        # 循环播放指定次数
        print(f"\n准备播放: {clip.name}")
        print(f"   重复次数: {args.repeat}")
        print(f"   播放速度: {args.speed}x")
        print(f"   总帧数: {clip.frame_count}")
        print(f"   总时长: {clip.total_duration_ms}ms")
        
        player.play_loop(clip, speed=args.speed, count=args.repeat)
        
    except KeyboardInterrupt:
        print("\n手动中断")
        player.stop()
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        try:
            robot.disconnect()
            print("\n已断开连接")
        except:
            pass

if __name__ == "__main__":
    main()
