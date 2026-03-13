"""
recorder.py — 动作录制器

通过卸载舵机让用户手动摆姿势，逐帧捕获关节位置，
生成 MotionClip 动作数据。

典型工作流：
    1. recorder.start("walk_v1")
    2. 手动摆姿势 → recorder.capture()   (重复 N 次)
    3. recorder.finish() → MotionClip
    4. clip.save("motions/walk_v1.json")
"""

import time
from typing import Optional

from ..robot import Robot
from .motion_data import Keyframe, MotionClip


class MotionRecorder:
    """动作录制器"""

    def __init__(self, robot: Robot):
        self.robot = robot
        self._recording = False
        self._clip: Optional[MotionClip] = None
        self._start_time: float = 0
        self._frame_interval_ms: int = 500  # 帧之间的默认间隔

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def current_clip(self) -> Optional[MotionClip]:
        return self._clip

    def start(self, name: str, description: str = "") -> None:
        """开始新的录制会话
        
        会卸载所有舵机，让来用户可以手动摆姿势。
        
        Args:
            name: 动作名称
            description: 动作描述
        """
        if self._recording:
            raise RuntimeError("录制已在进行中，请先 finish() 或 cancel()")

        self._clip = MotionClip(name=name, description=description)
        self._recording = True
        self._start_time = time.time()

        # 卸载所有舵机，让用户可以手动摆动
        self.robot.unload_all()
        print(f"\n{'='*50}")
        print(f"🎬 开始录制: {name}")
        print(f"{'='*50}")
        print("所有舵机已卸载，请手动摆姿势")
        print("使用 capture() 捕获当前帧\n")

    def capture(self) -> Keyframe:
        """捕获当前所有关节位置为一帧
        
        Returns:
            捕获的 Keyframe
        """
        if not self._recording or self._clip is None:
            raise RuntimeError("当前没有进行录制，请先 start()")

        # 计算时间戳
        if len(self._clip.keyframes) == 0:
            timestamp_ms = 0
        else:
            # 使用实际经过的时间
            elapsed = time.time() - self._start_time
            timestamp_ms = int(elapsed * 1000)

        # 读取所有关节位置
        positions = self.robot.get_all_positions()

        # 过滤掉读取失败的关节
        valid_positions = {}
        failed = []
        for name, pos in positions.items():
            if pos is not None:
                valid_positions[name] = pos
            else:
                failed.append(name)

        if failed:
            print(f"⚠️ 以下关节读取失败: {', '.join(failed)}")

        kf = Keyframe(timestamp_ms=timestamp_ms, positions=valid_positions)
        self._clip.keyframes.append(kf)

        frame_num = len(self._clip.keyframes)
        print(f"📸 帧 #{frame_num} 已捕获 (t={timestamp_ms}ms, {len(valid_positions)} 个关节)")

        # 显示关节位置
        for name, pos in sorted(valid_positions.items()):
            print(f"    {name:25s} → {pos}")

        return kf

    def undo(self) -> Optional[Keyframe]:
        """撤销最后一帧
        
        Returns:
            被撤销的 Keyframe, 或 None（没有帧可撤销）
        """
        if not self._recording or self._clip is None:
            raise RuntimeError("当前没有进行录制")

        if not self._clip.keyframes:
            print("⚠️ 没有帧可以撤销")
            return None

        removed = self._clip.keyframes.pop()
        print(f"↩️ 帧 #{len(self._clip.keyframes) + 1} 已撤销")
        return removed

    def finish(self) -> MotionClip:
        """结束录制，返回完整的 MotionClip
        
        Returns:
            录制完成的 MotionClip
        """
        if not self._recording or self._clip is None:
            raise RuntimeError("当前没有进行录制")

        clip = self._clip
        self._recording = False
        self._clip = None

        # 加载所有舵机
        self.robot.load_all()

        print(f"\n{'='*50}")
        print(f"✅ 录制完成: {clip.name}")
        print(f"   帧数: {clip.frame_count}")
        print(f"   总时长: {clip.total_duration_ms}ms")
        print(f"{'='*50}\n")

        return clip

    def cancel(self) -> None:
        """取消当前录制"""
        if self._recording:
            self.robot.load_all()
            self._recording = False
            self._clip = None
            print("❌ 录制已取消")
