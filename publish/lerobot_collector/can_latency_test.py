#!/usr/bin/env python3
"""
CAN 总线 4路并发读取延迟测试
==============================

同时启动 4 个线程分别读取 4 个 CAN 口 (can1~can4)，测量每路 CAN 的读取延迟。

延迟定义:
    - read_latency: 单次 refresh_all() + recv_all() 的耗时 (即一次完整读周期的 wall-clock 时间)
    - inter_read_gap: 两次成功读取之间的间隔

输出:
    - 实时打印每路 CAN 的延迟 (ms)
    - 每 2 秒汇总统计: min / max / avg / p50 / p95 / p99 / std

用法:
    python can_latency_test.py                                    # 默认 can1~can4
    python can_latency_test.py --ports can0 can2 can4 can6        # 自定义 CAN 口
    python can_latency_test.py --duration 60 --interval 0.01      # 跑 60 秒, 10ms 间隔
    python can_latency_test.py --raw                              # 使用原生 SocketCAN (不依赖 xarm_can)

依赖 (二选一):
    - xarm_can (默认, 与采集程序一致)
    - python-can + socketcan (--raw 模式, 仅 Linux)
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

# ==============================================================================
# 尝试导入 xarm_can (与采集脚本使用相同的 CAN 协议栈)
# ==============================================================================
_HAS_XARM_CAN = False
try:
    sys.path.append(r"/home/nvidia/x_air_sdk/publish/lerobot_collector/lib")
# 直接使用编译好的 xarm_can C++ 扩展
    import xarm_can as oa
    _HAS_XARM_CAN = True
except ImportError:
    print("fail to import can_xarm")

# ==============================================================================
# 尝试导入 python-can (备选方案)
# ==============================================================================
_HAS_PYTHON_CAN = False
try:
    import can
    _HAS_PYTHON_CAN = True
except ImportError:
    pass

# ==============================================================================
# 尝试导入原生 socketcan
# ==============================================================================
_HAS_SOCKETCAN = False
try:
    import socket
    import struct
    import fcntl
    # 检查是否在 Linux 上
    if sys.platform == 'linux':
        _HAS_SOCKETCAN = True
except ImportError:
    pass


# ==============================================================================
# 数据结构
# ==============================================================================
@dataclass
class CANStats:
    """单路 CAN 的延迟统计"""
    port: str
    latencies: deque = field(default_factory=lambda: deque(maxlen=10000))
    errors: int = 0
    total_reads: int = 0
    start_time: float = 0.0

    def record(self, latency_s: float):
        self.latencies.append(latency_s)
        self.total_reads += 1

    def record_error(self):
        self.errors += 1

    def summary(self) -> dict:
        if not self.latencies:
            return {"port": self.port, "count": 0}
        arr = np.array(self.latencies) * 1000  # 转 ms
        return {
            "port": self.port,
            "count": len(arr),
            "errors": self.errors,
            "min_ms": float(np.min(arr)),
            "max_ms": float(np.max(arr)),
            "avg_ms": float(np.mean(arr)),
            "p50_ms": float(np.percentile(arr, 50)),
            "p95_ms": float(np.percentile(arr, 95)),
            "p99_ms": float(np.percentile(arr, 99)),
            "std_ms": float(np.std(arr)),
        }

    def reset(self):
        self.latencies.clear()
        self.errors = 0
        # keep total_reads for lifetime counter


# ==============================================================================
# xarm_can 读取器 (与采集程序使用相同的 API)
# ==============================================================================
class XArmCANReader:
    """使用 xarm_can 库读取单个 CAN 口 — 仅做 refresh + recv，不读取位置"""

    MOTOR_TYPES = None
    SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
    RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
    GRIPPER_SEND_ID = 0x08
    GRIPPER_RECV_ID = 0x18

    def __init__(self, can_if: str):
        self.can_if = can_if
        self.arm = None
        self._connected = False

    def connect(self) -> bool:
        try:
            # 动态获取 MotorType (避免模块未加载)
            if XArmCANReader.MOTOR_TYPES is None:
                XArmCANReader.MOTOR_TYPES = [
                    oa.MotorType.DM8009, oa.MotorType.DM8009,
                    oa.MotorType.DM4340, oa.MotorType.DM4340,
                    oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310,
                ]

            self.arm = oa.XArm(self.can_if, True)  # True = CAN-FD
            self.arm.init_arm_motors(
                self.MOTOR_TYPES, self.SEND_IDS, self.RECV_IDS
            )

            # 尝试初始化夹爪 (可选, 失败也不影响)
            try:
                self.arm.init_gripper_motor(
                    oa.MotorType.DM4310, self.GRIPPER_SEND_ID, self.GRIPPER_RECV_ID
                )
            except Exception:
                try:
                    self.arm.init_gripper_motor(
                        oa.MotorType.DM4340, self.GRIPPER_SEND_ID, self.GRIPPER_RECV_ID
                    )
                except Exception:
                    pass

            # 验证连接
            self.arm.recv_all()
            self.arm.refresh_all()
            self.arm.recv_all()
            self._connected = True
            print(f"  ✅ {self.can_if}: xarm_can 连接成功")
            return True

        except Exception as e:
            print(f"  ❌ {self.can_if}: xarm_can 连接失败 — {e}")
            return False

    def read_once(self) -> Tuple[bool, float]:
        """执行一次 refresh + recv 周期，返回 (成功, 耗时_s)"""
        t0 = time.perf_counter()
        try:
            self.arm.refresh_all()
            self.arm.recv_all()
            elapsed = time.perf_counter() - t0
            return True, elapsed
        except Exception:
            elapsed = time.perf_counter() - t0
            return False, elapsed

    def disconnect(self):
        self._connected = False
        self.arm = None


# ==============================================================================
# Raw SocketCAN 读取器 (无外部依赖)
# ==============================================================================
class RawSocketCANReader:
    """使用原生 Linux SocketCAN 读取 CAN 帧延迟"""

    # CAN-FD frame struct
    CANFD_MTU = 72
    CAN_MTU = 16
    SOL_CAN_BASE = 100
    CAN_RAW = 1

    def __init__(self, can_if: str):
        self.can_if = can_if
        self.sock: Optional[socket.socket] = None

    def connect(self) -> bool:
        try:
            self.sock = socket.socket(
                socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW
            )
            # 尝试启用 CAN-FD
            try:
                CAN_RAW_FD_FRAMES = 5
                self.sock.setsockopt(
                    self.SOL_CAN_BASE, CAN_RAW_FD_FRAMES, 1
                )
            except Exception:
                pass  # CAN-FD 不可用，使用标准 CAN

            self.sock.bind((self.can_if,))
            self.sock.settimeout(0.05)  # 50ms 超时
            print(f"  ✅ {self.can_if}: Raw SocketCAN 连接成功")
            return True
        except Exception as e:
            print(f"  ❌ {self.can_if}: Raw SocketCAN 连接失败 — {e}")
            return False

    def read_once(self) -> Tuple[bool, float]:
        """从 socket 读取一帧 CAN 数据，返回 (成功, 耗时_s)"""
        t0 = time.perf_counter()
        try:
            frame = self.sock.recv(self.CANFD_MTU)
            elapsed = time.perf_counter() - t0
            # 解析 CAN ID (帧头 4 字节)
            if len(frame) >= 16:
                can_id = struct.unpack_from("<I", frame, 0)[0] & 0x1FFFFFFF
                return True, elapsed
            return True, elapsed
        except socket.timeout:
            elapsed = time.perf_counter() - t0
            return True, elapsed  # 超时也算成功(只是没数据)
        except Exception:
            elapsed = time.perf_counter() - t0
            return False, elapsed

    def disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


# ==============================================================================
# python-can 读取器
# ==============================================================================
class PythonCANReader:
    """使用 python-can 库读取 CAN 帧"""

    def __init__(self, can_if: str):
        self.can_if = can_if
        self.bus: Optional[can.BusABC] = None

    def connect(self) -> bool:
        try:
            self.bus = can.Bus(
                channel=self.can_if,
                interface="socketcan",
                fd=True,
                receive_own_messages=False,
            )
            print(f"  ✅ {self.can_if}: python-can 连接成功")
            return True
        except Exception as e:
            # 回退到非 FD 模式
            try:
                self.bus = can.Bus(
                    channel=self.can_if,
                    interface="socketcan",
                    receive_own_messages=False,
                )
                print(f"  ✅ {self.can_if}: python-can 连接成功 (non-FD)")
                return True
            except Exception as e2:
                print(f"  ❌ {self.can_if}: python-can 连接失败 — {e2}")
                return False

    def read_once(self) -> Tuple[bool, float]:
        t0 = time.perf_counter()
        try:
            msg = self.bus.recv(timeout=0.05)
            elapsed = time.perf_counter() - t0
            return True, elapsed
        except Exception:
            elapsed = time.perf_counter() - t0
            return False, elapsed

    def disconnect(self):
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None


# ==============================================================================
# 延迟测试线程
# ==============================================================================
class LatencyTestThread(threading.Thread):
    """单路 CAN 读取线程: 循环读取并记录延迟"""

    def __init__(
        self,
        reader,           # XArmCANReader | RawSocketCANReader | PythonCANReader
        stats: CANStats,
        interval_s: float = 0.0,   # 两次读取之间的 sleep (0 = 全速)
        stop_event: threading.Event = None,
    ):
        super().__init__(daemon=True)
        self.reader = reader
        self.stats = stats
        self.interval_s = interval_s
        self._stop = stop_event or threading.Event()

    def run(self):
        self.stats.start_time = time.perf_counter()
        while not self._stop.is_set():
            ok, elapsed = self.reader.read_once()
            if ok:
                self.stats.record(elapsed)
            else:
                self.stats.record_error()

            if self.interval_s > 0:
                time.sleep(self.interval_s)

    def stop(self):
        self._stop.set()


# ==============================================================================
# 显示器线程 — 每 2 秒打印统计摘要
# ==============================================================================
class StatsPrinter(threading.Thread):
    """定期打印 4 路 CAN 的延迟统计"""

    def __init__(
        self,
        stats_dict: Dict[str, CANStats],
        interval_s: float = 2.0,
        stop_event: threading.Event = None,
    ):
        super().__init__(daemon=True)
        self.stats_dict = stats_dict
        self.interval_s = interval_s
        self._stop = stop_event or threading.Event()
        self._round = 0

    def run(self):
        # 等第一轮数据积累
        time.sleep(self.interval_s)
        while not self._stop.is_set():
            self._print_summary()
            time.sleep(self.interval_s)

    def _print_summary(self):
        self._round += 1
        now = datetime.now().strftime("%H:%M:%S")
        header = f"\n{'═' * 100}"
        print(header)
        print(f"📊 [{now}] Round {self._round} — CAN 4路读取延迟统计 (单位: ms)")
        print(f"{'─' * 100}")
        print(
            f"{'Port':<8} {'Count':>8} {'Err':>6} "
            f"{'Min':>8} {'Max':>8} {'Avg':>8} "
            f"{'P50':>8} {'P95':>8} {'P99':>8} {'Std':>8}"
        )
        print(f"{'─' * 100}")

        for port in sorted(self.stats_dict.keys()):
            s = self.stats_dict[port].summary()
            if s.get("count", 0) == 0:
                print(f"{port:<8} {'—':>8} {'—':>6} {'—':>56}")
                continue
            print(
                f"{port:<8} {s['count']:>8d} {s['errors']:>6d} "
                f"{s['min_ms']:>8.3f} {s['max_ms']:>8.3f} {s['avg_ms']:>8.3f} "
                f"{s['p50_ms']:>8.3f} {s['p95_ms']:>8.3f} {s['p99_ms']:>8.3f} "
                f"{s['std_ms']:>8.3f}"
            )

        # 汇总: 4路综合
        all_lats = []
        for s in self.stats_dict.values():
            all_lats.extend(s.latencies)
        if all_lats:
            arr = np.array(all_lats) * 1000
            print(f"{'─' * 100}")
            print(
                f"{'ALL':<8} {len(arr):>8d} {'':>6} "
                f"{np.min(arr):>8.3f} {np.max(arr):>8.3f} {np.mean(arr):>8.3f} "
                f"{np.percentile(arr, 50):>8.3f} {np.percentile(arr, 95):>8.3f} "
                f"{np.percentile(arr, 99):>8.3f} {np.std(arr):>8.3f}"
            )

        # 记录异常值
        threshold_ms = 5.0
        slow_ports = []
        for port, s in self.stats_dict.items():
            if s.latencies:
                arr = np.array(s.latencies) * 1000
                p99 = np.percentile(arr, 99)
                if p99 > threshold_ms:
                    slow_ports.append(f"{port}(p99={p99:.2f}ms)")
        if slow_ports:
            print(f"⚠️  高延迟端口 (P99 > {threshold_ms}ms): {', '.join(slow_ports)}")

        print(f"{'═' * 100}\n")

    def stop(self):
        self._stop.set()


# ==============================================================================
# 主逻辑
# ==============================================================================
def create_reader(can_if: str, use_raw: bool = False, use_python_can: bool = False):
    """工厂函数: 根据可用后端创建 CAN 读取器"""
    if use_raw and _HAS_SOCKETCAN:
        return RawSocketCANReader(can_if)
    elif use_python_can and _HAS_PYTHON_CAN:
        return PythonCANReader(can_if)
    elif _HAS_XARM_CAN:
        return XArmCANReader(can_if)  # 默认: 与采集程序一致
    elif _HAS_SOCKETCAN:
        print(f"⚠️  xarm_can 不可用, 回退到 Raw SocketCAN")
        return RawSocketCANReader(can_if)
    elif _HAS_PYTHON_CAN:
        print(f"⚠️  xarm_can 不可用, 回退到 python-can")
        return PythonCANReader(can_if)
    else:
        print("❌ 没有可用的 CAN 后端! 请安装 xarm_can / python-can / 在 Linux 上运行")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="CAN 总线 4路并发读取延迟测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python can_latency_test.py                                     # 默认 can1~can4, xarm_can
  python can_latency_test.py --ports can0 can2 can4 can6         # 自定义 CAN 口
  python can_latency_test.py --duration 120 --interval 0.005     # 跑2分钟, 5ms间隔
  python can_latency_test.py --raw                               # 使用原生 SocketCAN
  python can_latency_test.py --python-can                        # 使用 python-can 库
        """,
    )
    parser.add_argument(
        "--ports", nargs="+", default=["can0", "can1", "can2", "can3"],
        help="要测试的 CAN 接口列表 (默认: can1 can2 can3 can4)"
    )
    parser.add_argument(
        "--duration", type=float, default=30.0,
        help="测试持续时间 (秒, 默认: 30)"
    )
    parser.add_argument(
        "--interval", type=float, default=0.0,
        help="两次 CAN 读取之间的 sleep 间隔 (秒, 默认: 0 = 全速读取)"
    )
    parser.add_argument(
        "--stats-interval", type=float, default=2.0,
        help="统计摘要打印间隔 (秒, 默认: 2)"
    )
    parser.add_argument(
        "--raw", action="store_true", default=False,
        help="使用原生 Linux SocketCAN (不依赖 xarm_can)"
    )
    parser.add_argument(
        "--python-can", action="store_true", default=False,
        help="使用 python-can 库 (pip install python-can)"
    )
    parser.add_argument(
        "--no-connect", action="store_true", default=False,
        help="跳过 CAN 连接, 仅做空循环基准测试"
    )

    args = parser.parse_args()

    # 校正参数
    if args.raw and args.python_can:
        print("⚠️  --raw 和 --python-can 互斥, 优先使用 --raw")
        args.python_can = False

    ports = args.ports
    print(f"\n{'═' * 60}")
    print(f"🔌 CAN 延迟测试 — {len(ports)} 路并发")
    print(f"{'═' * 60}")
    print(f"  CAN 接口:  {', '.join(ports)}")
    print(f"  测试时长:  {args.duration:.0f}s")
    print(f"  读取间隔:  {args.interval * 1000:.1f}ms" if args.interval > 0 else "  读取间隔:  全速 (不休眠)")
    print(f"  后端模式:  ", end="")
    if args.no_connect:
        print("基准测试 (不连接 CAN)")
    elif args.raw:
        print("Raw SocketCAN")
    elif args.python_can:
        print("python-can")
    else:
        print("xarm_can (与采集程序一致)")
    print(f"{'═' * 60}\n")

    # --- 创建读取器 ---
    readers = {}
    for port in ports:
        if args.no_connect:
            readers[port] = None  # 占位, 实际用空循环
        else:
            reader = create_reader(port, use_raw=args.raw, use_python_can=args.python_can)
            if reader.connect():
                readers[port] = reader
            else:
                print(f"⚠️  跳过 {port} (连接失败)")

    if not readers:
        print("❌ 没有任何 CAN 接口可用!")
        sys.exit(1)

    print(f"\n✅ 已连接 {len(readers)}/{len(ports)} 路 CAN, 开始延迟测试...\n")
    print(f"⏱️  将运行 {args.duration:.0f} 秒, 每 {args.stats_interval:.0f} 秒输出统计\n")

    # --- 创建统计和线程 ---
    stop_event = threading.Event()
    stats_dict: Dict[str, CANStats] = {
        port: CANStats(port=port) for port in readers.keys()
    }

    # 基准测试用空读取器
    class _NullReader:
        def read_once(self):
            time.sleep(0.001)
            return True, 0.001
        def disconnect(self):
            pass

    threads: List[LatencyTestThread] = []
    for port, reader in readers.items():
        actual_reader = _NullReader() if args.no_connect else reader
        t = LatencyTestThread(
            reader=actual_reader,
            stats=stats_dict[port],
            interval_s=args.interval,
            stop_event=stop_event,
        )
        t.start()
        threads.append(t)

    # 启动统计打印线程
    printer = StatsPrinter(
        stats_dict=stats_dict,
        interval_s=args.stats_interval,
        stop_event=stop_event,
    )
    printer.start()

    # --- 信号处理: Ctrl+C 提前结束 ---
    def _on_signal(sig, frame):
        print("\n⚠️  收到中断信号, 正在停止...")
        stop_event.set()
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    # --- 等待测试结束 ---
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()

    # --- 等待线程退出 ---
    for t in threads:
        t.join(timeout=2.0)
    printer.join(timeout=2.0)

    # --- 最终统计 ---
    print(f"\n{'█' * 100}")
    print(f"{'█' * 100}")
    print(f"  🏁 最终统计 — {len(readers)} 路 CAN 并发读取延迟")
    print(f"{'█' * 100}")
    printer._print_summary()  # 打印最后一次统计

    # 全局汇总
    all_lats = []
    for port, stats in stats_dict.items():
        all_lats.extend(stats.latencies)
    if all_lats:
        arr = np.array(all_lats) * 1000
        print(f"\n{'▀' * 60}")
        print(f"📋 全量汇总 (所有 CAN 口合并, {len(arr)} 次读取)")
        print(f"{'▀' * 60}")
        print(f"  Min:  {np.min(arr):.3f} ms")
        print(f"  Max:  {np.max(arr):.3f} ms")
        print(f"  Avg:  {np.mean(arr):.3f} ms")
        print(f"  P50:  {np.percentile(arr, 50):.3f} ms")
        print(f"  P95:  {np.percentile(arr, 95):.3f} ms")
        print(f"  P99:  {np.percentile(arr, 99):.3f} ms")
        print(f"  P99.9:{np.percentile(arr, 99.9):.3f} ms")
        print(f"  Std:  {np.std(arr):.3f} ms")
        print(f"  <1ms: {np.sum(arr < 1.0) / len(arr) * 100:.1f}%")
        print(f"  <2ms: {np.sum(arr < 2.0) / len(arr) * 100:.1f}%")
        print(f"  <5ms: {np.sum(arr < 5.0) / len(arr) * 100:.1f}%")
        print(f"{'▀' * 60}\n")

    # --- 清理 ---
    for port, reader in readers.items():
        try:
            reader.disconnect()
        except Exception:
            pass

    print("✅ 测试完成.\n")


if __name__ == "__main__":
    main()

