#!/usr/bin/env python3
"""
scan.py — 舵机扫描诊断工具

扫描所有在线舵机，显示 ID、位置、电压、温度。
用于验证硬件连接和舵机状态。

用法:
    python tools/scan.py --port COM3
    python tools/scan.py --port COM3 --range 1 20
"""

import argparse
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.servo_bus import ServoBus


def main():
    parser = argparse.ArgumentParser(description="扫描在线舵机")
    parser.add_argument("--port", required=True, help="串口名 (如 COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="波特率 (默认 115200)")
    parser.add_argument("--range", nargs=2, type=int, default=[1, 20],
                        metavar=("START", "END"), help="扫描 ID 范围 (默认 1 20)")
    args = parser.parse_args()

    start_id, end_id = args.range

    print(f"🔍 舵机扫描工具")
    print(f"   串口: {args.port} @ {args.baud} baud")
    print(f"   范围: ID {start_id} ~ {end_id}")
    print(f"{'='*60}")

    bus = ServoBus(args.port, args.baud)
    try:
        bus.connect()
        print(f"   串口已打开\n")

        # 先尝试广播读 ID
        print("📡 广播探测 (ID=254)...")
        broadcast_id = bus.read_id(ServoBus.BROADCAST_ID)
        if broadcast_id is not None:
            print(f"   广播返回 ID: {broadcast_id}")
        else:
            print(f"   广播无响应 (总线上可能有多个舵机)")
        print()

        # 逐 ID 扫描
        print(f"📋 逐 ID 扫描 ({start_id}~{end_id})...")
        print(f"{'─'*60}")
        print(f"{'ID':>4s}  {'位置':>6s}  {'电压':>8s}  {'温度':>6s}  状态")
        print(f"{'─'*60}")

        found = []
        for sid in range(start_id, end_id + 1):
            pos = bus.read_position(sid)
            if pos is not None:
                vin = bus.read_voltage(sid)
                temp = bus.read_temperature(sid)

                # 电压状态判断
                vin_str = f"{vin}mV" if vin else "N/A"
                if vin and vin < 6000:
                    vin_status = "⚠️低压"
                elif vin and vin > 8400:
                    vin_status = "⚠️过压"
                else:
                    vin_status = ""

                # 温度状态判断
                temp_str = f"{temp}°C" if temp else "N/A"
                if temp and temp > 60:
                    temp_status = "⚠️过热"
                elif temp and temp > 45:
                    temp_status = "⚠️偏高"
                else:
                    temp_status = ""

                status = " ".join(filter(None, ["✅ 在线", vin_status, temp_status]))
                print(f"{sid:>4d}  {pos:>6d}  {vin_str:>8s}  {temp_str:>6s}  {status}")
                found.append(sid)
            else:
                # 不显示离线的 ID，避免刷屏
                pass

        print(f"{'─'*60}")
        print(f"\n📊 扫描结果: 发现 {len(found)}/{end_id - start_id + 1} 个在线舵机")
        if found:
            print(f"   在线 ID: {found}")
        else:
            print("   ⚠️ 未发现任何舵机！请检查:")
            print("     1. 舵机是否已供电 (6~8.4V)")
            print("     2. TX/RX 接线是否正确")
            print("     3. 波特率是否匹配 (直连 115200 / 控制器板 9600)")
            print("     4. 舵机 ID 是否在扫描范围内")

    except Exception as e:
        print(f"\n❌ 错误: {e}")
        sys.exit(1)
    finally:
        bus.disconnect()
        print(f"\n串口已关闭")


if __name__ == "__main__":
    main()
