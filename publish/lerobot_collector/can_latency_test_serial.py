#!/usr/bin/env python3
"""
CAN 总线串行 vs 并行读取延迟对比测试
======================================

测试 4 路 CAN 在 串行(单线程) 和 并行(4线程) 模式下的读取延迟差异。

串行模式: 主线程依次读取 can1 → can2 → can3 → can4, 测每路耗时 + 总周期
并行模式: 4 线程同时读取, 测每路耗时 + 线程间抖动

用法:
    python can_latency_serial_test.py                           # 默认: 串行+并行对比, 30s
    python can_latency_serial_test.py --serial-only             # 仅串行
    python can_latency_serial_test.py --parallel-only           # 仅并行
    python can_latency_serial_test.py --duration 60             # 测试 60 秒
    python can_latency_serial_test.py --ports can0 can2 can4 can6
"""

import sys
import os
import time
import threading
import argparse
import signal
import numpy as np
from collections import deque
from datetime import datetime
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ── 尝试导入 xarm_can ──────────────────────────────────────────────────
_HAS_XARM_CAN = False
try:
    sys.path.append(r"/home/nvidia/x_air_sdk/publish/lerobot_collector/lib")
# 直接使用编译好的 xarm_can C++ 扩展
    import xarm_can as oa
    _HAS_XARM_CAN = True
except Exception as e:
    print(e)

# ── 回退方案 ───────────────────────────────────────────────────────────
_HAS_SOCKETCAN = (sys.platform == "linux")


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║                           CAN 读取器                                   ║
# ╚═════════════════════════════════════════════════════════════════════════╝

class CANReader:
    """统一 CAN 读取器 — 封装 xarm_can (与采集程序一致)"""

    MOTOR_TYPES = None
    SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
    RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

    def __init__(self, can_if: str):
        self.can_if = can_if
        self._arm = None

    # ── connect ───────────────────────────────────────────────────────
    def connect(self) -> bool:
        if not _HAS_XARM_CAN:
            print(f"  ❌ {self.can_if}: xarm_can 不可用")
            return False
        try:
            if CANReader.MOTOR_TYPES is None:
                CANReader.MOTOR_TYPES = [
                    oa.MotorType.DM8009, oa.MotorType.DM8009,
                    oa.MotorType.DM4340, oa.MotorType.DM4340,
                    oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310,
                ]
            self._arm = oa.XArm(self.can_if, True)
            self._arm.init_arm_motors(self.MOTOR_TYPES, self.SEND_IDS, self.RECV_IDS)
            try:
                self._arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)
            except Exception:
                try:
                    self._arm.init_gripper_motor(oa.MotorType.DM4340, 0x08, 0x18)
                except Exception:
                    pass
            self._arm.recv_all()
            self._arm.refresh_all()
            self._arm.recv_all()
            print(f"  ✅ {self.can_if} 连接成功")
            return True
        except Exception as e:
            print(f"  ❌ {self.can_if} 连接失败: {e}")
            return False

    # ── read_once ─────────────────────────────────────────────────────
    def read_once(self) -> Tuple[bool, float]:
        """返回 (ok, elapsed_seconds)"""
        t0 = time.perf_counter()
        try:
            self._arm.refresh_all()
            self._arm.recv_all()
            return True, time.perf_counter() - t0
        except Exception:
            return False, time.perf_counter() - t0

    def disconnect(self):
        self._arm = None


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║                           统计收集器                                   ║
# ╚═════════════════════════════════════════════════════════════════════════╝

@dataclass
class Stats:
    """延迟统计"""
    label: str
    latencies: deque = field(default_factory=lambda: deque(maxlen=50000))
    errors: int = 0
    reads: int = 0

    def record(self, elapsed_s: float):
        self.latencies.append(elapsed_s)
        self.reads += 1

    def snapshot(self) -> dict:
        if not self.latencies:
            return {"label": self.label, "count": 0}
        a = np.array(self.latencies) * 1000.0
        return {
            "label": self.label,
            "count": len(a),
            "err": self.errors,
            "min": float(np.min(a)),
            "max": float(np.max(a)),
            "avg": float(np.mean(a)),
            "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)),
            "p99": float(np.percentile(a, 99)),
            "std": float(np.std(a)),
        }

    def reset(self):
        self.latencies.clear()
        self.errors = 0


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║                          输出格式化                                    ║
# ╚═════════════════════════════════════════════════════════════════════════╝

HEADER_FMT = "{:<10} {:>7} {:>5} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}"
ROW_FMT    = "{:<10} {:>7d} {:>5d} {:>8.3f} {:>8.3f} {:>8.3f} {:>8.3f} {:>8.3f} {:>8.3f} {:>8.3f}"


def print_table(title: str, snapshots: List[dict]):
    print(f"\n{'─' * 100}")
    print(f"  {title}")
    print(f"{'─' * 100}")
    print(HEADER_FMT.format(
        "Port", "Count", "Err", "Min", "Max", "Avg", "P50", "P95", "P99", "Std"
    ))
    print(f"{'─' * 100}")
    for s in snapshots:
        if s.get("count", 0) == 0:
            print(f"{s['label']:<10} {'—':>7} {'—':>5}")
            continue
        print(ROW_FMT.format(
            s["label"], s["count"], s["err"],
            s["min"], s["max"], s["avg"], s["p50"], s["p95"], s["p99"], s["std"]
        ))
    print(f"{'─' * 100}\n")


def print_comparison(serial_snaps: List[dict], parallel_snaps: List[dict]):
    """串行 vs 并行对比表"""
    # 建立 port → (serial, parallel) 映射
    ser = {s["label"]: s for s in serial_snaps if "cycle" not in s["label"] and s.get("count", 0) > 0}
    par = {s["label"]: s for s in parallel_snaps if s.get("count", 0) > 0}

    print(f"\n{'█' * 100}")
    print(f"  🏁  串行 vs 并行 延迟对比 (avg / P99, 单位: ms)")
    print(f"{'█' * 100}")
    print(f"{'Port':<10} {'串行 Avg':>10} {'并行 Avg':>10} {'差异':>10}  │  {'串行 P99':>10} {'并行 P99':>10} {'差异':>10}")
    print(f"{'─' * 100}")

    all_ports = sorted(set(list(ser.keys()) + list(par.keys())))
    for port in all_ports:
        s_avg = ser[port]["avg"] if port in ser else float("nan")
        p_avg = par[port]["avg"] if port in par else float("nan")
        s_p99 = ser[port]["p99"] if port in ser else float("nan")
        p_p99 = par[port]["p99"] if port in par else float("nan")

        def _diff(a, b):
            if np.isnan(a) or np.isnan(b):
                return "—"
            ratio = (b - a) / a * 100 if a != 0 else 0
            arrow = "↑" if ratio > 5 else ("↓" if ratio < -5 else "≈")
            return f"{b - a:+.3f} ({ratio:+.0f}%) {arrow}"

        print(
            f"{port:<10} {s_avg:>10.3f} {p_avg:>10.3f} {_diff(s_avg, p_avg):>18}  │  "
            f"{s_p99:>10.3f} {p_p99:>10.3f} {_diff(s_p99, p_p99):>18}"
        )

    # 总周期对比
    ser_cycle = next((s for s in serial_snaps if "cycle" in s["label"]), None)
    if ser_cycle and ser_cycle.get("count", 0) > 0:
        print(f"{'─' * 100}")
        print(f"  串行总周期 (4路依次读完) — Avg: {ser_cycle['avg']:.3f} ms, P99: {ser_cycle['p99']:.3f} ms")

    print(f"{'█' * 100}\n")


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║                         测试核心逻辑                                   ║
# ╚═════════════════════════════════════════════════════════════════════════╝

def run_serial_test(
    readers: Dict[str, CANReader],
    duration_s: float,
    interval_s: float,
    stats_interval_s: float,
    stop_event: threading.Event,
) -> Dict[str, Stats]:
    """串行模式: 主线程依次读取 can1 → can2 → can3 → can4"""

    stats = {port: Stats(label=port) for port in readers.keys()}
    stats["_cycle"] = Stats(label="cycle(4路合计)")

    print(f"  🔵 串行模式运行中... (单线程依次读 {len(readers)} 路 CAN)")
    t_start = time.perf_counter()
    last_print = t_start

    while not stop_event.is_set() and (time.perf_counter() - t_start) < duration_s:
        cycle_t0 = time.perf_counter()

        for port, reader in readers.items():
            ok, elapsed = reader.read_once()
            stats[port].record(elapsed)
            if not ok:
                stats[port].errors += 1

            if interval_s > 0:
                time.sleep(interval_s)

        cycle_elapsed = time.perf_counter() - cycle_t0
        stats["_cycle"].record(cycle_elapsed)

        # 定期打印
        now = time.perf_counter()
        if now - last_print >= stats_interval_s:
            last_print = now
            elapsed_total = now - t_start
            snaps = [stats[p].snapshot() for p in readers.keys()]
            snaps.append(stats["_cycle"].snapshot())
            print_table(
                f"🔵 串行模式 [{elapsed_total:.0f}s / {duration_s:.0f}s]",
                snaps,
            )

    return stats


def run_parallel_test(
    readers: Dict[str, CANReader],
    duration_s: float,
    interval_s: float,
    stats_interval_s: float,
    stop_event: threading.Event,
) -> Dict[str, Stats]:
    """并行模式: 每个 CAN 口一个独立线程"""

    stats = {port: Stats(label=port) for port in readers.keys()}

    def _reader_thread(port: str, reader: CANReader):
        while not stop_event.is_set():
            ok, elapsed = reader.read_once()
            stats[port].record(elapsed)
            if not ok:
                stats[port].errors += 1
            if interval_s > 0:
                time.sleep(interval_s)

    print(f"  🟠 并行模式运行中... ({len(readers)} 线程同时读)")

    threads = []
    for port, reader in readers.items():
        t = threading.Thread(target=_reader_thread, args=(port, reader), daemon=True)
        t.start()
        threads.append(t)

    t_start = time.perf_counter()
    last_print = t_start

    while not stop_event.is_set() and (time.perf_counter() - t_start) < duration_s:
        time.sleep(stats_interval_s)
        elapsed_total = time.perf_counter() - t_start
        snaps = [stats[p].snapshot() for p in readers.keys()]
        print_table(
            f"🟠 并行模式 [{elapsed_total:.0f}s / {duration_s:.0f}s]",
            snaps,
        )

    # 等待线程结束
    stop_event.set()
    for t in threads:
        t.join(timeout=2.0)

    return stats


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║                              main                                      ║
# ╚═════════════════════════════════════════════════════════════════════════╝

def main():
    parser = argparse.ArgumentParser(
        description="CAN 总线 串行 vs 并行 读取延迟对比测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python can_latency_serial_test.py                       # 串行 + 并行对比 (默认)
  python can_latency_serial_test.py --serial-only         # 仅串行测试
  python can_latency_serial_test.py --parallel-only       # 仅并行测试
  python can_latency_serial_test.py --duration 120        # 跑 2 分钟
  python can_latency_serial_test.py --ports can0 can2 can4 can6
        """,
    )
    parser.add_argument("--ports", nargs="+",
                        default=["can0", "can1", "can2", "can3"],
                        help="CAN 接口列表 (默认: can1 can2 can3 can4)")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="每种模式的测试时长 (秒, 默认 30)")
    parser.add_argument("--interval", type=float, default=0.0,
                        help="每次读取后 sleep 间隔 (秒, 默认 0 = 全速)")
    parser.add_argument("--stats-interval", type=float, default=3.0,
                        help="统计打印间隔 (秒, 默认 3)")
    parser.add_argument("--serial-only", action="store_true",
                        help="仅运行串行测试")
    parser.add_argument("--parallel-only", action="store_true",
                        help="仅运行并行测试")

    args = parser.parse_args()

    # 决定运行哪些模式
    run_serial = not args.parallel_only
    run_parallel = not args.serial_only
    compare_mode = run_serial and run_parallel

    print(f"\n{'═' * 60}")
    print(f"  CAN 延迟测试 — ")
    print(f"  CAN 接口:  {', '.join(args.ports)}")
    print(f"  每模式时长: {args.duration:.0f}s")
    print(f"  模式:       ", end="")
    if compare_mode:
        print("串行 → 并行 对比")
    elif run_serial:
        print("仅串行")
    else:
        print("仅并行")
    print(f"{'═' * 60}\n")

    # ── 连接 CAN ─────────────────────────────────────────────────────
    print("🔌 初始化 CAN 连接...")
    readers: Dict[str, CANReader] = {}
    for port in args.ports:
        r = CANReader(port)
        if r.connect():
            readers[port] = r
        else:
            print(f"  ⚠️  跳过 {port}")

    if not readers:
        print("❌ 没有可用 CAN 接口, 退出.")
        sys.exit(1)

    print(f"  ✅ {len(readers)}/{len(args.ports)} 路 CAN 就绪\n")

    # ── 信号处理 ─────────────────────────────────────────────────────
    stop_event = threading.Event()

    def _on_signal(sig, frame):
        print("\n⚠️  收到中断信号, 停止测试...")
        stop_event.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    serial_stats = None
    parallel_stats = None

    try:
        # ── 串行测试 ─────────────────────────────────────────────────
        if run_serial:
            serial_stop = threading.Event()
            serial_stats = run_serial_test(
                readers, args.duration, args.interval,
                args.stats_interval, serial_stop,
            )
            serial_stop.set()

            if compare_mode:
                print(f"\n{'·' * 60}")
                print(f"  串行完成, 冷却 2 秒后开始并行测试...")
                print(f"{'·' * 60}\n")
                time.sleep(2.0)

        # ── 并行测试 ─────────────────────────────────────────────────
        if run_parallel:
            parallel_stop = threading.Event()
            parallel_stats = run_parallel_test(
                readers, args.duration, args.interval,
                args.stats_interval, parallel_stop,
            )
            parallel_stop.set()

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()

    # ── 最终输出 ─────────────────────────────────────────────────────
    if serial_stats:
        snaps = [serial_stats[p].snapshot() for p in readers.keys()]
        cycle_snap = serial_stats["_cycle"].snapshot()
        print_table("🔵 串行模式 — 最终统计", snaps + [cycle_snap])

    if parallel_stats:
        snaps = [parallel_stats[p].snapshot() for p in readers.keys()]
        print_table("🟠 并行模式 — 最终统计", snaps)

    if compare_mode and serial_stats and parallel_stats:
        ser_snaps = [serial_stats[p].snapshot() for p in readers.keys()]
        ser_snaps.append(serial_stats["_cycle"].snapshot())
        par_snaps = [parallel_stats[p].snapshot() for p in readers.keys()]
        print_comparison(ser_snaps, par_snaps)

    # ── 清理 ─────────────────────────────────────────────────────────
    for reader in readers.values():
        try:
            reader.disconnect()
        except Exception:
            pass

    print("✅ 测试完成.\n")


if __name__ == "__main__":
    main()

