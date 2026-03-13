---
name: LOBOT Serial Bus Servo Control
description: Complete protocol reference and control guide for LOBOT LX-series serial bus servos (LX-16A, LX-15D, LX-224). Covers LX direct protocol (115200 baud) and controller board protocol (9600 baud) with frame formats, all 36 commands, checksum algorithm, wiring, safety, and code examples.
---

# LOBOT 串行总线舵机完整控制指南

> 本技能文件提供 LOBOT LX 系列总线舵机的**完整协议规范、控制方法和安全规范**，可在任何项目中复用。

---

## 1. 硬件概览

### 1.1 支持的舵机型号

| 型号 | 工作电压 | 建议电压 | 位置范围 | 角度范围 | 特点 |
|------|---------|---------|---------|---------|------|
| **LX-16A** | 6.0~8.4V | 7.4V (2S 锂电) | 0~1000 | 0°~240° | 标准总线舵机 |
| **LX-15D** | 5.0~8.4V | 7.4V (2S 锂电) | 0~1000 | 0°~240° | 低压版本 |
| **LX-224** | 6.0~8.4V | 7.4V (2S 锂电) | 0~1000 | 0°~240° | 高性能版本 |

### 1.2 物理参数

- **位置精度**: 0~1000 (对应 0°~240° 角度范围)
- **位置 → 角度换算**: `角度 = 位置值 × 0.24°`，即 `500 = 120°` (中位)
- **移动时间**: 0~30000ms (控制移动速度)
- **ID 范围**: 0~253 (可用 ID)，254 为广播地址
- **通信方式**: 半双工 UART 串行总线 (单线通信)
- **菊花链连接**: 多个舵机共用一条总线，通过 ID 区分

### 1.3 接线与硬件连接

```
USB 转串口模块 (CH343/CH340/CP2102/FTDI)
    ├── TX  ──→  舵机信号线 (白/黄)
    ├── RX  ←──  舵机信号线 (白/黄)  [注: 半双工，TX/RX 通过电平转换板合并]
    └── GND ──→  舵机 GND (棕/黑)

电源 (7.4V 2S 锂电池)
    ├── V+  ──→  舵机 VCC (红)
    └── GND ──→  舵机 GND (棕/黑)  [与串口共地]
```

> **⚠️ 安全**: 先接信号线，再上电。断电后再拔线。禁止热插拔！

---

## 2. 通信协议

本舵机支持两种通信协议。核心区别:

| 特性 | LX 直连协议 | 控制器板协议 |
|------|------------|-------------|
| **波特率** | **115200** | **9600** |
| **用途** | 直接控制单个舵机 | 通过控制器板管理多舵机 |
| **有无校验和** | ✅ 有 | ❌ 无 |
| **帧头** | `0x55 0x55` | `0x55 0x55` |
| **适用场景** | USB 转串口直连 | 通过 LOBOT 控制器板 |
| **串口设置** | 8N1 (8数据位, 无校验, 1停止位) | 8N1 |

---

### 2.1 LX 直连协议 (115200 baud) — 主要协议

#### 帧格式

```
┌────────┬────────┬──────┬──────┬─────┬────────┬──────────┐
│ 0x55   │ 0x55   │  ID  │ LEN  │ CMD │ PARAMS │ CHECKSUM │
│ 帧头1  │ 帧头2  │舵机ID│ 长度 │ 指令│  参数  │  校验和  │
└────────┴────────┴──────┴──────┴─────┴────────┴──────────┘
                   ↑──────── LEN 计算范围 ────────↑
```

- **帧头**: 固定 `0x55 0x55` (2 字节)
- **ID**: 目标舵机 ID (0~253, 254=广播)
- **LEN**: `参数字节数 + 3` (包含 ID, LEN, CMD 本身，不含帧头)
- **CMD**: 指令编号
- **PARAMS**: 指令参数 (可变长度)
- **CHECKSUM**: `(~(ID + LEN + CMD + 所有参数字节)) & 0xFF`

#### 校验和算法

```python
def lx_checksum(servo_id, length, cmd, params=b''):
    s = servo_id + length + cmd
    for b in params:
        s += b
    return (~s) & 0xFF
```

```javascript
function lxChecksum(id, len, cmd, params = []) {
    let s = id + len + cmd;
    for (const b of params) s += b;
    return (~s) & 0xFF;
}
```

#### 构建指令函数

```python
def lx_build_cmd(servo_id, cmd, params=b''):
    length = len(params) + 3
    checksum = lx_checksum(servo_id, length, cmd, params)
    buf = bytearray([0x55, 0x55, servo_id, length, cmd])
    buf.extend(params)
    buf.append(checksum)
    return buf
```

```javascript
function lxBuildCmd(servoId, cmd, params = []) {
    const len = params.length + 3;
    const cs = lxChecksum(servoId, len, cmd, params);
    return new Uint8Array([0x55, 0x55, servoId, len, cmd, ...params, cs]);
}
```

#### 解析响应

响应帧也以 `0x55 0x55` 开头，结构与发送帧相同：

```
接收: 0x55 0x55 [ID] [LEN] [CMD] [PARAMS...] [CHECKSUM]
```

- **位置/电压**: 参数为 2 字节 Little-Endian 有符号 16 位 → `resp[5] | (resp[6] << 8)`
- **温度/ID**: 参数为 1 字节 → `resp[5]`
- **遇到大于 32767 的值**: `value -= 65536` (处理有符号数)

---

### 2.2 LX 直连协议 — 全部指令详解

#### 2.2.1 基本移动指令

| CMD | 名称 | 方向 | 参数 |
|-----|------|------|------|
| **1** | `SERVO_MOVE_TIME_WRITE` | → 发送 | `pos_L, pos_H, time_L, time_H` |
| **2** | `SERVO_MOVE_TIME_READ` | ← 读取 | 无参数，返回当前目标位置和时间 |

**移动指令 (CMD=1) 详解**:
```
参数顺序: [位置低字节, 位置高字节, 时间低字节, 时间高字节]
位置: 0~1000 (对应 0°~240°)
时间: 0~30000 ms (控制移动速度)
```

**示例 — 移动到位置 500, 耗时 1000ms**:
```python
position = 500
move_time = 1000
params = bytearray()
params.extend(position.to_bytes(2, 'little'))  # [0xF4, 0x01]
params.extend(move_time.to_bytes(2, 'little'))  # [0xE8, 0x03]
lx_send_cmd(servo_id, 1, params)
```

```javascript
const position = 500, moveTime = 1000;
const params = [
    position & 0xFF, (position >> 8) & 0xFF,     // 位置: 低字节, 高字节
    moveTime & 0xFF, (moveTime >> 8) & 0xFF,     // 时间: 低字节, 高字节
];
serialWrite(lxBuildCmd(servoId, 1, params));
```

> ⚠️ **参数顺序很重要**: 位置在前，时间在后！

#### 2.2.2 ID 管理

| CMD | 名称 | 方向 | 参数 |说明 |
|-----|------|------|------|-----|
| **13** | `ID_WRITE` | → 发送 | `new_id` (1 字节) | 修改舵机 ID (0~253) |
| **14** | `ID_READ` | → 发送 | 无 | 读取舵机 ID，返回 `resp[5]` |

**修改 ID 注意事项**:
- ⚠️ **总线上必须仅连接一个舵机**，否则所有舵机同时被修改
- 修改后等待 100~500ms 让舵机保存
- 修改后使用广播地址 (254) 验证

```python
# 修改 ID: 旧ID → 新ID
lx_send_cmd(old_id, 13, bytes([new_id]))
time.sleep(0.1)  # 等待保存

# 验证：广播读取
lx_send_cmd(254, 14)  # 广播读取当前ID
resp = read_response()
current_id = resp[5]
```

#### 2.2.3 角度偏移/校准

| CMD | 名称 | 方向 | 参数 | 说明 |
|-----|------|------|------|------|
| **17** | `ANGLE_OFFSET_ADJUST` | → 发送 | `offset` (int8, -125~125) | 临时调整中位偏移 |
| **18** | `ANGLE_OFFSET_WRITE` | → 发送 | 无 | 将当前偏移永久写入Flash |
| **19** | `ANGLE_OFFSET_READ` | ← 读取 | 无 | 读取当前偏移值 |

**用途**: 当舵机机械安装后物理中位与期望不一致时，通过偏移量微调。

#### 2.2.4 角度限位

| CMD | 名称 | 方向 | 参数 | 说明 |
|-----|------|------|------|------|
| **20** | `ANGLE_LIMIT_WRITE` | → 发送 | `min_L, min_H, max_L, max_H` | 设置角度限位范围 |
| **21** | `ANGLE_LIMIT_READ` | ← 读取 | 无 | 读取角度限位设置 |

**用途**: 限制舵机的最大运动范围，防止机械结构碰撞。

#### 2.2.5 电压限位保护

| CMD | 名称 | 方向 | 参数 | 说明 |
|-----|------|------|------|------|
| **22** | `VIN_LIMIT_WRITE` | → 发送 | `min_L, min_H, max_L, max_H` | 设置电压保护范围 (mV) |
| **23** | `VIN_LIMIT_READ` | ← 读取 | 无 | 读取电压限位设置 |

**用途**: 超出设定电压范围时舵机自动卸载保护。

#### 2.2.6 温度限位保护

| CMD | 名称 | 方向 | 参数 | 说明 |
|-----|------|------|------|------|
| **24** | `TEMP_MAX_LIMIT_WRITE` | → 发送 | `max_temp` (1 字节, °C) | 设置最高温度保护阈值 |
| **25** | `TEMP_MAX_LIMIT_READ` | ← 读取 | 无 | 读取最高温度限值 |

#### 2.2.7 传感器读取

| CMD | 名称 | 方向 | 返回 | 说明 |
|-----|------|------|------|------|
| **26** | `TEMP_READ` | ← 读取 | 1 字节 (°C) | 读取舵机内部温度 |
| **27** | `VIN_READ` | ← 读取 | 2 字节 LE (mV) | 读取输入电压 |
| **28** | `POS_READ` | ← 读取 | 2 字节 LE (有符号) | 读取当前实际位置 |

**读取位置 (CMD=28)**:
```python
lx_send_cmd(servo_id, 28)
resp = read_response()
pos = resp[5] | (resp[6] << 8)
if pos > 32767:
    pos -= 65536  # 处理有符号数
```

**读取电压 (CMD=27)**:
```python
lx_send_cmd(servo_id, 27)
resp = read_response()
vin_mv = resp[5] | (resp[6] << 8)  # 单位: mV
vin_v = vin_mv / 1000  # 转换为 V
```

**读取温度 (CMD=26)**:
```python
lx_send_cmd(servo_id, 26)
resp = read_response()
temp_c = resp[5]  # 单位: °C
```

#### 2.2.8 舵机/电机模式切换

| CMD | 名称 | 方向 | 参数 | 说明 |
|-----|------|------|------|------|
| **29** | `SERVO_OR_MOTOR_MODE_WRITE` | → 发送 | `mode, speed_L, speed_H` | 0=舵机模式, 1=电机模式 |
| **30** | `SERVO_OR_MOTOR_MODE_READ` | ← 读取 | 无 | 读取当前运行模式 |

**电机模式**: 当设为电机模式时，舵机变为持续旋转电机，speed 控制转速和方向（-1000~1000）。

#### 2.2.9 加载/卸载控制

| CMD | 名称 | 方向 | 参数 | 说明 |
|-----|------|------|------|------|
| **31** | `LOAD_OR_UNLOAD_WRITE` | → 发送 | `state` (1 字节) | 0=卸载(释放扭力), 1=加载(锁定) |
| **32** | `LOAD_OR_UNLOAD_READ` | ← 读取 | 无 | 读取当前加载状态 |

**卸载**后舵机轴可自由旋转，再次发送移动指令会自动重新加载。

#### 2.2.10 LED 控制

| CMD | 名称 | 方向 | 参数 | 说明 |
|-----|------|------|------|------|
| **33** | `LED_CTRL_WRITE` | → 发送 | `state` (1 字节) | 0=LED开, 1=LED关 |
| **34** | `LED_CTRL_READ` | ← 读取 | 无 | 读取 LED 状态 |

#### 2.2.11 LED 报警设置

| CMD | 名称 | 方向 | 参数 | 说明 |
|-----|------|------|------|------|
| **35** | `LED_ERROR_WRITE` | → 发送 | `error_mask` (1 字节) | 设置报警条件 |
| **36** | `LED_ERROR_READ` | ← 读取 | 无 | 读取报警设置 |

**报警掩码 (bit 位)**:
- `bit 0`: 过温报警
- `bit 1`: 过压报警
- `bit 2`: 堵转报警

---

### 2.3 控制器板协议 (9600 baud)

用于通过 LOBOT 控制器板批量管理舵机。帧格式**无校验和**：

```
┌────────┬────────┬──────┬─────┬────────┐
│ 0x55   │ 0x55   │ LEN  │ CMD │ PARAMS │
└────────┴────────┴──────┴─────┴────────┘
```

- **帧头**: 固定 `0x55 0x55`
- **LEN**: 从 LEN 开始到包尾的总字节数（含 LEN 本身）
- **CMD**: 指令编号
- **PARAMS**: 指令参数

#### 全部控制器板指令

| CMD | 名称 | 参数 | 说明 |
|-----|------|------|------|
| **3** | 多舵机移动 | `count, time_L, time_H, (id, pos_L, pos_H)*N` | 同时控制 N 个舵机 |
| **6** | 运行动作组 | `group_id, times_L, times_H` | times=0 为无限循环 |
| **7** | 停止动作组 | 无 | 立即停止当前动作组 |
| **11** | 设置速度 | `group_id, speed_L, speed_H` | 百分比速度 (100=原速) |
| **15** | 读电池电压 | 无 | 返回 mV (2 字节 LE) |
| **20** | 卸载舵机 | `count, id1, id2, ...` | 释放指定舵机扭力 |
| **21** | 读多轴位置 | `count, id1, id2, ...` | 批量读取位置 |

#### 多舵机移动 (CMD=3) 详解

```
LEN = 舵机数量 × 3 + 5
PARAMS = [count, time_L, time_H, (servo_id, pos_L, pos_H) × count]
```

```python
# 示例：同时移动舵机 1→位置200, 舵机 2→位置800, 时间1000ms
buf = bytearray([0x55, 0x55])
buf.append(2 * 3 + 5)     # LEN = 11
buf.append(3)              # CMD = CMD_SERVO_MOVE
buf.append(2)              # 舵机数量
buf.extend((1000).to_bytes(2, 'little'))  # 时间 1000ms
buf.append(1)              # 舵机 1
buf.extend((200).to_bytes(2, 'little'))   # 位置 200
buf.append(2)              # 舵机 2
buf.extend((800).to_bytes(2, 'little'))   # 位置 800
serial.write(buf)
```

#### PWM 舵机支持

控制器板同时支持 PWM 舵机 (如 SG90)：
- PWM 舵机位置范围: **500~2500** (对应脉宽 μs)
- 总线舵机位置范围: **0~1000**
- 使用相同的 CMD=3 指令，只是位置值范围不同

---

## 3. 通信时序与串口管理

### 3.1 串口基本配置

```
波特率: 115200 (LX 直连) / 9600 (控制器板)
数据位: 8
校验位: 无 (None)
停止位: 1
流控: 无
```

### 3.2 命令响应时序

- **发送后等待**: 100~200ms 超时等待响应
- **命令间隔**: 建议 10~50ms
- **轮询间隔**: 建议 500ms (2Hz) 轮询传感器数据
- **广播指令**: ID=254 时不等待响应 (除 ID_READ 外)

### 3.3 串口互斥锁 (关键!)

半双工总线同一时刻只能有一个命令在执行，必须实现互斥锁：

```javascript
// 使用 Promise 链实现互斥
let serialLock = Promise.resolve();

function withSerialLock(fn) {
    const prev = serialLock;
    let resolve;
    serialLock = new Promise(r => resolve = r);
    return prev.then(() => fn().finally(resolve));
}

// 所有串口操作都通过互斥锁
async function safeRead(servoId) {
    return withSerialLock(async () => {
        clearRxBuffer();
        await serialWrite(lxBuildCmd(servoId, 28));  // 读位置
        return await waitForResponse(200);
    });
}
```

### 3.4 接收缓冲区管理

```javascript
// 持续追加接收数据
let rxBuffer = new Uint8Array(0);

function appendToBuffer(newData) {
    const merged = new Uint8Array(rxBuffer.length + newData.length);
    merged.set(rxBuffer);
    merged.set(newData, rxBuffer.length);
    rxBuffer = merged;
}

// 从缓冲区解析帧
function parseFrame() {
    for (let i = 0; i < rxBuffer.length - 1; i++) {
        if (rxBuffer[i] === 0x55 && rxBuffer[i + 1] === 0x55) {
            if (i + 4 > rxBuffer.length) return null;  // 不完整
            const pktLen = rxBuffer[i + 3];
            const totalLen = 3 + pktLen;
            if (i + totalLen > rxBuffer.length) return null;
            const packet = rxBuffer.slice(i, i + totalLen);
            rxBuffer = rxBuffer.slice(i + totalLen);  // 消费已解析数据
            return packet;
        }
    }
    return null;
}
```

---

## 4. 舵机扫描策略

### 4.1 广播 + 逐 ID 扫描

最佳扫描策略是组合使用：

1. **广播读 ID** (CMD=14, ID=254): 如果总线上只有一个舵机，可直接获得 ID
2. **逐 ID 读位置** (CMD=28, ID=1~20): 用读位置命令探测每个 ID 是否在线

```python
found = []

# 步骤 1: 广播
broadcast_id = lx_read_id(254)

# 步骤 2: 逐 ID 扫描 (默认 1~20)
for sid in range(1, 21):
    pos = lx_read_position(sid)
    if pos is not None:
        vin = lx_read_vin(sid)
        temp = lx_read_temp(sid)
        found.append({'id': sid, 'pos': pos, 'vin': vin, 'temp': temp})

# 步骤 3: 广播发现的 ID 若超出 1~20，补充扫描
if broadcast_id and broadcast_id not in [s['id'] for s in found]:
    # 追加扫描...
```

### 4.2 多波特率扫描

当不确定连接方式时，可尝试多种波特率：

```python
bauds_to_try = [115200, 9600]
for baud in bauds_to_try:
    ctrl = ServoController(port, baud=baud)
    ctrl.connect()
    result = ctrl.lx_read_id(254)
    if result is not None:
        print(f"Found servo at {baud} baud, ID={result}")
        break
    ctrl.disconnect()
```

---

## 5. 安全规范

### 5.1 电压安全

| 等级 | 条件 | 措施 |
|------|------|------|
| ✅ 正常 | 6.0~8.4V | 正常运行 |
| ⚠️ 警告 | <6.0V 或 >8.4V | 舵机自动保护卸载 |
| ❌ 危险 | >9V | **舵机烧毁風險，严禁！** |

### 5.2 温度安全

| 温度 | 状态 | 建议 |
|------|------|------|
| 20~45°C | 正常 | — |
| 45~60°C | 偏高 | 降低负载或增加休息间隔 |
| >60°C | 过热 | 立即停止运行，等待冷却 |

### 5.3 操作安全清单

- [ ] **接线顺序**: 先接信号线 → 再通电 → 断电后 → 再拔线
- [ ] **修改 ID**: 总线上仅连接一个舵机
- [ ] **电压监控**: 持续监控电压，避免过放/过充
- [ ] **温度监控**: 高负载场景定期检查温度
- [ ] **机械限位**: 设置合理的角度限位，防止结构碰撞
- [ ] **共地**: USB 串口模块与舵机电源必须共地
- [ ] **电源独立**: 舵机供电使用独立电源，不要从串口模块取电

---

## 6. 常用操作代码示例

### 6.1 Python 完整示例

```python
import serial
import time

class LobotServo:
    """LOBOT LX 系列总线舵机控制器"""

    BROADCAST_ID = 254

    def __init__(self, port, baud=115200, timeout=0.1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.1)

    def close(self):
        self.ser.close()

    def _checksum(self, sid, length, cmd, params):
        s = sid + length + cmd + sum(params)
        return (~s) & 0xFF

    def _send(self, sid, cmd, params=b''):
        length = len(params) + 3
        cs = self._checksum(sid, length, cmd, params)
        buf = bytes([0x55, 0x55, sid, length, cmd]) + bytes(params) + bytes([cs])
        self.ser.write(buf)

    def _recv(self, timeout=0.15):
        deadline = time.time() + timeout
        buf = bytearray()
        while time.time() < deadline:
            if self.ser.in_waiting > 0:
                buf.extend(self.ser.read(self.ser.in_waiting))
                for i in range(len(buf) - 1):
                    if buf[i] == 0x55 and buf[i+1] == 0x55 and len(buf) >= i + 4:
                        pkt_len = buf[i+3]
                        total = 3 + pkt_len
                        if len(buf) >= i + total:
                            return bytes(buf[i:i+total])
            time.sleep(0.005)
        return None

    # --- 核心操作 ---
    def move(self, sid, position, time_ms=1000):
        """移动舵机到指定位置"""
        position = max(0, min(1000, position))
        time_ms = max(0, min(30000, time_ms))
        params = position.to_bytes(2, 'little') + time_ms.to_bytes(2, 'little')
        self._send(sid, 1, params)

    def read_position(self, sid):
        """读取当前位置 (0~1000)"""
        self.ser.flushInput()
        self._send(sid, 28)
        resp = self._recv()
        if resp and len(resp) >= 8:
            pos = resp[5] | (resp[6] << 8)
            return pos - 65536 if pos > 32767 else pos
        return None

    def read_voltage(self, sid):
        """读取输入电压 (mV)"""
        self.ser.flushInput()
        self._send(sid, 27)
        resp = self._recv()
        if resp and len(resp) >= 8:
            return resp[5] | (resp[6] << 8)
        return None

    def read_temperature(self, sid):
        """读取温度 (°C)"""
        self.ser.flushInput()
        self._send(sid, 26)
        resp = self._recv()
        if resp and len(resp) >= 7:
            return resp[5]
        return None

    def read_id(self, sid=254):
        """读取舵机 ID"""
        self.ser.flushInput()
        self._send(sid, 14)
        resp = self._recv()
        if resp and len(resp) >= 7:
            return resp[5]
        return None

    def write_id(self, old_id, new_id):
        """修改舵机 ID (0~253, 总线上仅一个舵机!)"""
        self._send(old_id, 13, bytes([max(0, min(253, new_id))]))
        time.sleep(0.1)

    def unload(self, sid):
        """卸载舵机 (释放扭力)"""
        self._send(sid, 31, bytes([0]))

    def load(self, sid):
        """加载舵机 (锁定扭力)"""
        self._send(sid, 31, bytes([1]))

    def set_led(self, sid, on=True):
        """控制舵机 LED"""
        self._send(sid, 33, bytes([0 if on else 1]))

    def set_motor_mode(self, sid, speed=0):
        """切换为电机模式 (持续旋转)"""
        speed = max(-1000, min(1000, speed))
        params = bytes([1, 0]) + speed.to_bytes(2, 'little', signed=True)
        self._send(sid, 29, params)

    def set_servo_mode(self, sid):
        """切换回舵机模式"""
        self._send(sid, 29, bytes([0, 0, 0, 0]))
```

### 6.2 JavaScript (Web Serial API) 示例

```javascript
// 连接串口
const port = await navigator.serial.requestPort();
await port.open({ baudRate: 115200 });

// 发送指令
async function serialWrite(data) {
    const writer = port.writable.getWriter();
    try {
        await writer.write(data instanceof Uint8Array ? data : new Uint8Array(data));
    } finally {
        writer.releaseLock();
    }
}

// 移动舵机
function lxBuildCmd(servoId, cmd, params = []) {
    const len = params.length + 3;
    let s = servoId + len + cmd;
    for (const b of params) s += b;
    const cs = (~s) & 0xFF;
    return new Uint8Array([0x55, 0x55, servoId, len, cmd, ...params, cs]);
}

// 移动到位置 500, 时间 1000ms
const pos = 500, time = 1000;
const params = [pos & 0xFF, (pos >> 8) & 0xFF, time & 0xFF, (time >> 8) & 0xFF];
await serialWrite(lxBuildCmd(1, 1, params));
```

---

## 7. 常见问题排查

| 问题 | 可能原因 | 解决方法 |
|------|---------|---------|
| 连接后无法发现舵机 | 1. 舵机未供电 | 检查电源连接，6~8.4V |
| | 2. 波特率不匹配 | LX 直连用 115200，控制器板用 9600 |
| | 3. TX/RX 接反 | 检查接线 |
| | 4. ID 超出扫描范围 | 默认扫描 1~20，可扩大范围 |
| 舵机不转动 | 1. 位置已在目标位置 | 换一个目标位置 |
| | 2. 舵机已卸载 | 发送移动指令自动加载 |
| | 3. 电压过低 | 检查电池电量 |
| 舵机抖动 | 1. 电源不足 | 使用更大功率电源 |
| | 2. 信号干扰 | 缩短线缆，增加屏蔽 |
| 通信偶尔失败 | 1. 半双工冲突 | 确保互斥锁生效 |
| | 2. 缓冲区残留数据 | 发送前清空接收缓冲区 |
| 读取到负数位置 | 有符号 16 位 | `if val > 32767: val -= 65536` |

---

## 8. 快速参考卡片

### LX 指令速查 (CMD 编号)

```
移动类:   1=移动  2=读移动目标
ID 类:    13=写ID  14=读ID
校准类:   17=偏移调整  18=偏移写入  19=偏移读取
限位类:   20=角度限位写  21=角度限位读
          22=电压限位写  23=电压限位读
          24=温度限位写  25=温度限位读
传感器:   26=读温度  27=读电压  28=读位置
模式类:   29=舵机/电机模式写  30=舵机/电机模式读
加载类:   31=加载/卸载写  32=加载/卸载读
LED 类:   33=LED控制写  34=LED控制读
报警类:   35=LED报警写  36=LED报警读
广播ID:   254
```

### 控制器板指令速查

```
3=多舵机移动  6=运行动作组  7=停止动作组
11=设置速度   15=读电池电压  20=卸载舵机  21=读多轴位置
```

### 关键数值范围

```
位置:     0~1000 (总线舵机) / 500~2500 (PWM 舵机)
角度:     0°~240°
时间:     0~30000 ms
ID:       0~253 (254=广播)
电压:     6000~8400 mV (建议 7400 mV)
温度:     正常 20~45°C, 上限 60°C
速度%:    10~500 (100=原速)
动作组号: 0~255
```

---

## 9. 参考资源

- **项目源码**: [WebSerial-ServoTest](file:///c:/GitHub/WebSerial-ServoTest)
- **Python CLI 工具**: [servo_test.py](file:///c:/GitHub/WebSerial-ServoTest/docs/servo_test.py)
- **控制器手册 (PDF)**: `docs/01 Bus Servo Controller User Manual.pdf`
- **通信协议 (PDF)**: `docs/Bus Servo Controller Communication Protocol.pdf`
- **Jetson 开发文档 (PDF)**: `docs/Jetson Development.pdf`
- **示例源码**: `docs/02 Source Code/` (Python 控制示例)
