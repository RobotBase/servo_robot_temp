"""
player.py — 动作回放器

回放 MotionClip 动作数据，支持变速控制。
遍历关键帧序列，按时间间隔驱动舵机同步运动。
"""

import time
import threading
from typing import Optional

from ..robot import Robot
from .motion_data import MotionClip


class MotionPlayer:
    """动作回放器"""

    def __init__(self, robot: Robot):
        self.robot = robot
        self._playing = False
        self._stop_event = threading.Event()

    @property
    def is_playing(self) -> bool:
        return self._playing

    def play(self, clip: MotionClip, speed: float = 1.0) -> None:
        """回放动作
        
        Args:
            clip: 动作数据
            speed: 速度倍率 (1.0=原速, 2.0=两倍速, 0.5=半速)
        """
        if clip.frame_count < 2:
            print("⚠️ 动作至少需要 2 帧才能回放")
            return

        if speed <= 0:
            raise ValueError("speed 必须大于 0")

        self._playing = True
        self._stop_event.clear()

        print(f"\n▶️ 回放: {clip.name} (速度: {speed}x, {clip.frame_count} 帧)")
        print(f"   原始时长: {clip.total_duration_ms}ms → "
              f"实际时长: {int(clip.total_duration_ms / speed)}ms\n")

        try:
            for i in range(clip.frame_count):
                if self._stop_event.is_set():
                    print("\n⏹ 回放已停止")
                    break

                kf = clip.keyframes[i]

                # 计算当前帧到下一帧之间的移动时间
                if i < clip.frame_count - 1:
                    next_kf = clip.keyframes[i + 1]
                    move_time_ms = next_kf.timestamp_ms - kf.timestamp_ms
                else:
                    # 最后一帧，使用与前一帧相同的间隔
                    if i > 0:
                        prev_kf = clip.keyframes[i - 1]
                        move_time_ms = kf.timestamp_ms - prev_kf.timestamp_ms
                    else:
                        move_time_ms = 500

                # 按速度缩放时间
                scaled_time_ms = max(50, int(move_time_ms / speed))

                # 驱动所有舵机
                self.robot.set_joints_raw(kf.positions, scaled_time_ms)

                print(f"  帧 {i+1}/{clip.frame_count}  t={kf.timestamp_ms}ms  "
                      f"move={scaled_time_ms}ms")

                # 等待移动完成
                if i < clip.frame_count - 1:
                    wait_time = scaled_time_ms / 1000.0
                    if self._stop_event.wait(timeout=wait_time):
                        print("\n⏹ 回放已停止")
                        break

        finally:
            self._playing = False

        if not self._stop_event.is_set():
            print(f"\n✅ 回放完成: {clip.name}\n")

    def play_loop(
        self,
        clip: MotionClip,
        speed: float = 1.0,
        count: int = 0,
    ) -> None:
        """循环回放动作
        
        Args:
            clip: 动作数据
            speed: 速度倍率
            count: 循环次数 (0=无限循环，直到 stop())
        """
        iteration = 0
        while not self._stop_event.is_set():
            iteration += 1
            if count > 0 and iteration > count:
                break
            print(f"\n🔁 循环 #{iteration}" + (f"/{count}" if count > 0 else ""))
            self.play(clip, speed)

    def stop(self) -> None:
        """停止回放"""
        self._stop_event.set()

    def play_async(self, clip: MotionClip, speed: float = 1.0) -> threading.Thread:
        """在后台线程中回放
        
        Returns:
            回放线程
        """
        thread = threading.Thread(
            target=self.play,
            args=(clip, speed),
            daemon=True,
        )
        thread.start()
        return thread
