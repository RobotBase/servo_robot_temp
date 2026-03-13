# 🤖 Servo Robot — 双足舵机机器人下半身运动控制

[English](#english) | [中文](#中文)

---

<a name="中文"></a>
## 中文

### 项目简介

双足舵机机器人**下半身运动控制系统**。基于 LOBOT LX 系列串行总线舵机（LX-16A / LX-15D / LX-224），通过 USB 串口直连实现对机器人下半身 10 个关节舵机的精确控制。

**核心功能：**
- 🎮 **舵机驱动** — 完整的硬件配置、正反向定义、扫描诊断
- 🎬 **试教录制** — 关键帧动作录制、回放、变速控制

### 硬件需求

| 组件 | 规格 |
|------|------|
| 舵机 | LOBOT LX-16A / LX-15D / LX-224 × 10 |
| 电源 | 7.4V 2S 锂电池（6.0~8.4V） |
| 串口 | USB 转串口模块（CH343/CH340/CP2102） |
| 结构 | 双足下半身，每腿 5 个关节 |

### 机械结构

```
            ┌─────────┐
            │  骨 盆   │
            └─┬─────┬─┘
    左腿       │     │       右腿
  ┌─ID:1──┐   │     │   ┌─ID:6──┐
  │髋侧摆  │   │     │   │髋侧摆  │
  ├─ID:2──┤   │     │   ├─ID:7──┤
  │髋前摆  │   │     │   │髋前摆  │
  ├─ID:3──┤         ├─ID:8──┤
  │膝关节  │         │膝关节  │
  ├─ID:4──┤         ├─ID:9──┤
  │踝前摆  │         │踝前摆  │
  ├─ID:5──┤         ├─ID:10─┤
  │踝侧摆  │         │踝侧摆  │
  └───────┘         └───────┘
```

### 快速上手

```bash
# 1. 安装依赖
pip install -r requirements.txt

# 2. 扫描舵机（确认硬件连接）
python tools/scan.py --port COM3

# 3. 运动测试（各关节正反向验证）
python tools/test_motion.py --port COM3

# 4. 试教录制
python tools/teach.py --port COM3
```

### 项目结构

```
servo_robot_temp/
├── README.md                    # 项目说明
├── requirements.txt             # Python 依赖
├── src/                         # 核心库
│   ├── servo_bus.py             # 舵机总线驱动 (LX 协议)
│   ├── robot_config.py          # 机器人硬件配置
│   ├── robot.py                 # 高层机器人控制
│   └── recorder/                # 试教录制子系统
│       ├── motion_data.py       # 动作数据模型
│       ├── recorder.py          # 录制器
│       └── player.py            # 回放器
├── tools/                       # CLI 工具
│   ├── scan.py                  # 扫描诊断
│   ├── test_motion.py           # 运动测试
│   └── teach.py                 # 试教录制交互工具
├── motions/                     # 录制的动作文件 (.json)
└── .agents/skills/              # Agent 技能参考
```

### 开发路线图

| 阶段 | 内容 | 状态 |
|------|------|------|
| **Phase 1** | 舵机驱动与定义 — 硬件配置、协议通信、诊断扫描 | 🚧 进行中 |
| **Phase 2** | 试教录制系统 — 关键帧录制、回放、变速控制 | ⏳ 待开始 |

---

<a name="english"></a>
## English

### Overview

Lower-body motion control system for a **bipedal servo robot**. Built on LOBOT LX-series serial bus servos (LX-16A / LX-15D / LX-224), providing precise control of 10 joint servos via USB serial connection.

**Core Features:**
- 🎮 **Servo Driving** — Hardware config, direction mapping, scan & diagnostics
- 🎬 **Teach & Record** — Keyframe motion recording, playback, speed scaling

### Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Servos | LOBOT LX-16A / LX-15D / LX-224 × 10 |
| Power | 7.4V 2S LiPo battery (6.0~8.4V) |
| Serial | USB-to-Serial adapter (CH343/CH340/CP2102) |
| Structure | Bipedal lower body, 5 joints per leg |

### Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Scan servos (verify hardware connection)
python tools/scan.py --port COM3

# 3. Motion test (verify each joint direction)
python tools/test_motion.py --port COM3

# 4. Teach & record
python tools/teach.py --port COM3
```

### Development Roadmap

| Phase | Description | Status |
|-------|-------------|--------|
| **Phase 1** | Servo driving & definition — hardware config, protocol comm, diagnostics | 🚧 In Progress |
| **Phase 2** | Teach & record system — keyframe recording, playback, speed control | ⏳ Pending |

### License

MIT
