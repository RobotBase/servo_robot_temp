"""
motion_data.py — 动作数据模型

定义关键帧和动作片段的数据结构，
支持 JSON 序列化/反序列化。
"""

import json
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import Optional


@dataclass
class Keyframe:
    """单个关键帧"""

    timestamp_ms: int                   # 从录制开始的时间戳 (ms)
    positions: dict[str, int]           # 关节名 → 原始位置值

    def to_dict(self) -> dict:
        return {
            "timestamp_ms": self.timestamp_ms,
            "positions": dict(self.positions),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Keyframe":
        return cls(
            timestamp_ms=d["timestamp_ms"],
            positions=d["positions"],
        )


@dataclass
class MotionClip:
    """动作片段（一组关键帧序列）"""

    name: str                           # 动作名称
    description: str = ""               # 描述
    created_at: str = ""                # ISO 时间戳
    keyframes: list[Keyframe] = field(default_factory=list)

    def __post_init__(self):
        if not self.created_at:
            # 默认使用 UTC+8 时间
            tz = timezone(timedelta(hours=8))
            self.created_at = datetime.now(tz).isoformat()

    @property
    def frame_count(self) -> int:
        return len(self.keyframes)

    @property
    def total_duration_ms(self) -> int:
        if not self.keyframes:
            return 0
        return self.keyframes[-1].timestamp_ms

    @property
    def joint_names(self) -> list[str]:
        """获取此动作涉及的关节名列表"""
        if self.keyframes:
            return list(self.keyframes[0].positions.keys())
        return []

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "description": self.description,
            "created_at": self.created_at,
            "frame_count": self.frame_count,
            "total_duration_ms": self.total_duration_ms,
            "keyframes": [kf.to_dict() for kf in self.keyframes],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "MotionClip":
        return cls(
            name=d["name"],
            description=d.get("description", ""),
            created_at=d.get("created_at", ""),
            keyframes=[Keyframe.from_dict(kf) for kf in d.get("keyframes", [])],
        )

    def save(self, filepath: str | Path) -> None:
        """保存为 JSON 文件"""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, ensure_ascii=False, indent=2)
        print(f"[MotionClip] 已保存: {filepath} ({self.frame_count} 帧, {self.total_duration_ms}ms)")

    @classmethod
    def load(cls, filepath: str | Path) -> "MotionClip":
        """从 JSON 文件加载"""
        filepath = Path(filepath)
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        clip = cls.from_dict(data)
        print(f"[MotionClip] 已加载: {filepath} ({clip.frame_count} 帧, {clip.total_duration_ms}ms)")
        return clip
