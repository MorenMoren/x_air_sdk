#!/usr/bin/env python3
"""
xarm_can — Pure Python ctypes wrapper for libxarm_can_sdk.so

Provides the exact same API as the compiled xarm_can C extension module,
but works on any architecture (x86_64 / aarch64) by loading the appropriate
native shared library via ctypes.

Usage is identical to the original:
    import xarm_can as oa
    arm = oa.XArm("can0", True)
    arm.init_arm_motors([oa.MotorType.DM8009, ...], send_ids, recv_ids)
"""

import ctypes
import os
import platform
import select
import socket
import struct
from ctypes import (
    POINTER, byref,
    c_char_p, c_float, c_int, c_uint32, c_void_p,
)
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

# ═══════════════════════════════════════════════════════════════════════════
# Native library discovery & loading
# ═══════════════════════════════════════════════════════════════════════════

def _get_arch_dir() -> str:
    """Detect CPU architecture for subdirectory lookup."""
    machine = platform.machine()
    if machine in ("aarch64", "arm64", "ARM64"):
        return "aarch64"
    elif machine in ("x86_64", "AMD64", "amd64"):
        return "x86_64"
    else:
        return machine


def _find_library() -> str:
    """Locate libxarm_can_sdk.so across multiple possible paths."""
    _dir = os.path.dirname(os.path.abspath(__file__))
    arch = _get_arch_dir()

    # Build search paths
    candidates = [
        # 1) <lib>/x86_64/libxarm_can_sdk.so
        os.path.join(_dir, arch, "libxarm_can_sdk.so"),
        # 2) <lib>/libxarm_can_sdk.so  (flat layout)
        os.path.join(_dir, "libxarm_can_sdk.so"),
        # 3) sibling xarm_can SDK package
        os.path.join(
            _dir, "..", "..", "xarm_can", "package", "lib", arch, "libxarm_can_sdk.so",
        ),
    ]

    for p in candidates:
        if os.path.exists(p):
            print(p)
            return p
    
    # Fallback — let the dynamic linker search LD_LIBRARY_PATH
    return "libxarm_can_sdk.so"


_SDK: Optional[ctypes.CDLL] = None


def _get_sdk() -> ctypes.CDLL:
    """Return the loaded libxarm_can_sdk CDLL instance (singleton)."""
    global _SDK
    if _SDK is None:
        lib_path = _find_library()
        _SDK = ctypes.CDLL(lib_path)
        _bind_api(_SDK)
    return _SDK


# ═══════════════════════════════════════════════════════════════════════════
# C struct definitions  (must match xarm_can_sdk.h / xarm_sdk_export.h)
# ═══════════════════════════════════════════════════════════════════════════

class _CMitParam(ctypes.Structure):
    """xarm_sdk_mit_param_t  —  C field order: pos, vel, kp, kd, torque"""
    _fields_ = [
        ("pos",    c_float),
        ("vel",    c_float),
        ("kp",     c_float),
        ("kd",     c_float),
        ("torque", c_float),
    ]


class _CJointState(ctypes.Structure):
    """xarm_sdk_joint_state_t  —  pos, vel, torque"""
    _fields_ = [
        ("pos",    c_float),
        ("vel",    c_float),
        ("torque", c_float),
    ]


# ═══════════════════════════════════════════════════════════════════════════
# C API binding  (argtypes / restype for every exported function)
# ═══════════════════════════════════════════════════════════════════════════

def _bind_api(sdk: ctypes.CDLL):
    """Declare the function signatures of libxarm_can_sdk."""

    sdk.xarm_sdk_get_version.restype = c_char_p

    sdk.xarm_sdk_create.argtypes = [c_char_p, c_int, POINTER(c_void_p)]
    sdk.xarm_sdk_create.restype = c_int

    sdk.xarm_sdk_destroy.argtypes = [c_void_p]
    sdk.xarm_sdk_destroy.restype = c_int

    sdk.xarm_sdk_init_arm_motors.argtypes = [
        c_void_p, POINTER(c_int), POINTER(c_uint32), POINTER(c_uint32), c_int,
    ]
    sdk.xarm_sdk_init_arm_motors.restype = c_int

    sdk.xarm_sdk_init_gripper_motor.argtypes = [
        c_void_p, c_int, c_uint32, c_uint32,
    ]
    sdk.xarm_sdk_init_gripper_motor.restype = c_int

    sdk.xarm_sdk_enable_all.argtypes = [c_void_p]
    sdk.xarm_sdk_enable_all.restype = c_int

    sdk.xarm_sdk_disable_all.argtypes = [c_void_p]
    sdk.xarm_sdk_disable_all.restype = c_int

    sdk.xarm_sdk_set_zero_all.argtypes = [c_void_p]
    sdk.xarm_sdk_set_zero_all.restype = c_int

    sdk.xarm_sdk_refresh_all.argtypes = [c_void_p]
    sdk.xarm_sdk_refresh_all.restype = c_int

    sdk.xarm_sdk_recv_all.argtypes = [c_void_p, c_int]
    sdk.xarm_sdk_recv_all.restype = c_int

    sdk.xarm_sdk_set_callback_mode_state_all.argtypes = [c_void_p]
    sdk.xarm_sdk_set_callback_mode_state_all.restype = c_int

    sdk.xarm_sdk_get_arm_joint_states.argtypes = [
        c_void_p, POINTER(_CJointState), c_int,
    ]
    sdk.xarm_sdk_get_arm_joint_states.restype = c_int

    sdk.xarm_sdk_get_gripper_state.argtypes = [c_void_p, POINTER(_CJointState)]
    sdk.xarm_sdk_get_gripper_state.restype = c_int

    sdk.xarm_sdk_gripper_open.argtypes = [c_void_p, c_float, c_float]
    sdk.xarm_sdk_gripper_open.restype = c_int

    sdk.xarm_sdk_gripper_close.argtypes = [c_void_p, c_float, c_float]
    sdk.xarm_sdk_gripper_close.restype = c_int

    sdk.xarm_sdk_gripper_mit_control.argtypes = [
        c_void_p, POINTER(_CMitParam),
    ]
    sdk.xarm_sdk_gripper_mit_control.restype = c_int

    sdk.xarm_sdk_arm_mit_control.argtypes = [
        c_void_p, POINTER(_CMitParam), c_int,
    ]
    sdk.xarm_sdk_arm_mit_control.restype = c_int

    sdk.xarm_sdk_get_last_error.argtypes = [c_char_p, c_int]
    sdk.xarm_sdk_get_last_error.restype = c_int


def _get_last_error(sdk: ctypes.CDLL) -> str:
    buf = ctypes.create_string_buffer(512)
    sdk.xarm_sdk_get_last_error(buf, len(buf))
    return buf.value.decode("utf-8", errors="replace")


# ═══════════════════════════════════════════════════════════════════════════
# Public Python API  (mirrors the original xarm_can C extension module)
# ═══════════════════════════════════════════════════════════════════════════

class MotorType(IntEnum):
    """Motor type enum — values MUST match xarm_can_sdk.h."""
    DM4310     = 0
    DM4340     = 1
    DM6006     = 2
    DM8006     = 3
    DM8009     = 4
    DM10010L   = 5
    DM10010    = 6
    DM1015     = 7
    DMH3510    = 8
    DM_J4310_2EC = 9


class CallbackMode(IntEnum):
    """Callback / feedback mode enum."""
    STATE = 0


# ═══════════════════════════════════════════════════════════════════════════
# Passive socketCAN mode
# ═══════════════════════════════════════════════════════════════════════════
#
# Problem this solves
# -------------------
# Only ONE process can own an SDK handle (xarm_sdk_create) on a given CAN
# interface at a time. When a teleop process already holds can0/can1/…, a
# second process (the data collector) that also calls xarm_sdk_create() on the
# same interface blocks forever in the SDK's init handshake.
#
# Insight
# -------
# During teleop every motor is already broadcasting its STATE feedback frame
# (solicited by the teleop's refresh_all / MIT commands). A second process does
# not need to talk to the motors at all — it can open a *raw* AF_CAN socket and
# passively sniff those exact frames. Multiple processes can bind raw sockets to
# the same CAN interface simultaneously; the kernel copies every frame to each.
#
# When XARM_CAN_PASSIVE is enabled, XArm skips the SDK entirely and decodes the
# DM-motor STATE frames itself. The public API (XArm / init_arm_motors /
# recv_all / get_arm().get_motors()[i].get_position() …) is unchanged, so the
# collector code does not need to be touched beyond opting in once.
#
# Frame format (verified against the SDK binary's CanPacketDecoder and live
# candump on this machine) — classic 8-byte DM STATE frame, arbitration id ==
# recv_id of the motor:
#     data[0] : high nibble = status/err, low nibble = slave id
#     data[1..2] : position, 16-bit big-endian  → uint_to_float(-p_max, p_max, 16)
#     data[3..4] : velocity, 12-bit (d3<<4 | d4>>4) → uint_to_float(-v_max, v_max, 12)
#     data[4..5] : torque,   12-bit ((d4&0xF)<<8 | d5) → uint_to_float(-t_max, t_max, 12)
#     data[6] : MOS temperature   data[7] : rotor temperature
#
# Per-motor (p_max, v_max, t_max) limits, lifted verbatim from the SDK binary's
# xarm::damiao_motor::MOTOR_LIMIT_PARAMS table (indexed by MotorType value).

_MOTOR_LIMIT_PARAMS: Dict[int, Tuple[float, float, float]] = {
    MotorType.DM4310:       (12.5, 50.0, 5.0),
    MotorType.DM4340:       (12.5, 30.0, 10.0),
    MotorType.DM6006:       (12.5, 50.0, 10.0),
    MotorType.DM8006:       (12.5, 8.0,  28.0),
    MotorType.DM8009:       (12.5, 10.0, 28.0),
    MotorType.DM10010L:     (12.5, 45.0, 20.0),
    MotorType.DM10010:      (12.5, 45.0, 40.0),
    MotorType.DM1015:       (12.5, 45.0, 54.0),
    MotorType.DMH3510:      (12.5, 25.0, 200.0),
    MotorType.DM_J4310_2EC: (12.5, 20.0, 200.0),
}

# Linux SocketCAN constants (avoid relying on socket module having them)
_CAN_RAW          = getattr(socket, "CAN_RAW", 1)
_PF_CAN           = getattr(socket, "PF_CAN", getattr(socket, "AF_CAN", 29))
_SOL_CAN_RAW      = getattr(socket, "SOL_CAN_RAW", 101)
_CAN_RAW_FD_FRAMES = 5
_CAN_SFF_MASK     = 0x000007FF
_CAN_EFF_MASK     = 0x1FFFFFFF
_CAN_EFF_FLAG     = 0x80000000

# Module-level opt-in. Honour the env var at import time; set_passive_mode()
# can override it programmatically before any XArm is constructed.
_PASSIVE_MODE: bool = os.environ.get("XARM_CAN_PASSIVE", "0").lower() in ("1", "true", "yes", "on")


def set_passive_mode(enabled: bool) -> None:
    """Enable/disable passive socketCAN mode for subsequently-created XArm objects.

    Call this once before constructing any XArm. In passive mode XArm never
    creates an SDK handle — it sniffs DM STATE frames off a raw CAN socket, so
    it can coexist with a teleop process that owns the same interface.
    """
    global _PASSIVE_MODE
    _PASSIVE_MODE = bool(enabled)


def is_passive_mode() -> bool:
    return _PASSIVE_MODE


def _uint_to_float(x: int, lo: float, hi: float, bits: int) -> float:
    """DM-motor fixed-point → physical value (matches SDK CanPacketDecoder)."""
    return x * (hi - lo) / float((1 << bits) - 1) + lo


def _decode_dm_state(data: bytes, limits: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Decode an 8-byte DM STATE payload into (pos, vel, torque)."""
    p_max, v_max, t_max = limits
    pos_raw = (data[1] << 8) | data[2]
    vel_raw = (data[3] << 4) | (data[4] >> 4)
    tor_raw = ((data[4] & 0x0F) << 8) | data[5]
    pos = _uint_to_float(pos_raw, -p_max, p_max, 16)
    vel = _uint_to_float(vel_raw, -v_max, v_max, 12)
    tor = _uint_to_float(tor_raw, -t_max, t_max, 12)
    return pos, vel, tor


# MIT command gain limits — read verbatim from the SDK binary (CanPacketEncoder
# pack_mit_control_data uses kp in [0, 500], kd in [0, 5]).
_KP_MAX = 500.0
_KD_MAX = 5.0


def _float_to_uint(x: float, lo: float, hi: float, bits: int) -> int:
    """Physical value → DM-motor fixed-point (matches SDK double_to_uint)."""
    if hi <= lo:
        return 0
    span = (1 << bits) - 1
    x = lo if x < lo else (hi if x > hi else x)  # clamp into range
    return int(round((x - lo) * span / (hi - lo))) & span


def _encode_dm_mit(kp: float, kd: float, pos: float, vel: float, torque: float,
                   limits: Tuple[float, float, float]) -> bytes:
    """Encode an 8-byte DM MIT-control payload (matches SDK pack_mit_control_data)."""
    p_max, v_max, t_max = limits
    kp_r  = _float_to_uint(kp, 0.0, _KP_MAX, 12)
    kd_r  = _float_to_uint(kd, 0.0, _KD_MAX, 12)
    pos_r = _float_to_uint(pos, -p_max, p_max, 16)
    vel_r = _float_to_uint(vel, -v_max, v_max, 12)
    tor_r = _float_to_uint(torque, -t_max, t_max, 12)
    return bytes((
        (pos_r >> 8) & 0xFF,
        pos_r & 0xFF,
        (vel_r >> 4) & 0xFF,
        ((vel_r & 0x0F) << 4) | ((kp_r >> 8) & 0x0F),
        kp_r & 0xFF,
        (kd_r >> 4) & 0xFF,
        ((kd_r & 0x0F) << 4) | ((tor_r >> 8) & 0x0F),
        tor_r & 0xFF,
    ))


class _PassiveCANBackend:
    """Raw AF_CAN socket: passively decodes DM STATE frames, and (optionally)
    sends DM MIT-control command frames — all without an SDK handle.

    Owns no motors of its own; XArm registers (recv_id → _Motor, limits) pairs
    so that draining the socket updates the same _Motor objects the public API
    hands out. It also records each motor's send_id + limits (in registration
    order) so MIT commands can be encoded and written straight to the bus.

    This is what lets a second process (data collector OR go_home) coexist with
    a teleop process: the kernel broadcasts every RX frame to all bound sockets,
    and accepts TX frames from any of them. The blocking SDK recv path is never
    used, so there is no init-handshake hang.
    """

    def __init__(self, can_if: str, fd: bool = True):
        self.can_if = can_if
        self._fd = fd
        # recv_id -> (motor, (p_max, v_max, t_max))
        self._targets: Dict[int, Tuple["_Motor", Tuple[float, float, float]]] = {}
        # ordered send-side info: list of (send_id, (p_max, v_max, t_max))
        self._arm_send: List[Tuple[int, Tuple[float, float, float]]] = []
        self._gripper_send: Optional[Tuple[int, Tuple[float, float, float]]] = None

        self._sock = socket.socket(_PF_CAN, socket.SOCK_RAW, _CAN_RAW)
        if fd:
            try:
                self._sock.setsockopt(_SOL_CAN_RAW, _CAN_RAW_FD_FRAMES, 1)
            except OSError:
                pass  # FD not supported on this interface; classic frames still arrive
        try:
            self._sock.bind((can_if,))
        except OSError as e:
            self._sock.close()
            raise RuntimeError(f"passive bind('{can_if}') failed: {e}") from e
        self._sock.setblocking(False)

    def register(self, recv_id: int, motor: "_Motor", motor_type: int,
                 send_id: Optional[int] = None, is_gripper: bool = False):
        limits = _MOTOR_LIMIT_PARAMS.get(int(motor_type), (12.5, 50.0, 5.0))
        self._targets[int(recv_id) & _CAN_EFF_MASK] = (motor, limits)
        if send_id is not None:
            if is_gripper:
                self._gripper_send = (int(send_id), limits)
            else:
                self._arm_send.append((int(send_id), limits))

    def _write_frame(self, can_id: int, payload: bytes):
        """Write one classic 8-byte CAN frame (struct can_frame, 16 bytes)."""
        if self._sock is None:
            return
        data = payload[:8].ljust(8, b"\x00")
        frame = struct.pack("<IB3x8s", can_id & _CAN_SFF_MASK, len(payload[:8]), data)
        try:
            self._sock.send(frame)
        except OSError:
            pass

    def send_mit_arm(self, params: List["MITParam"]):
        """Encode + send one MIT command frame per arm motor (registration order)."""
        for i, p in enumerate(params):
            if i >= len(self._arm_send):
                break
            send_id, limits = self._arm_send[i]
            payload = _encode_dm_mit(p.kp, p.kd, p.pos, p.vel, p.torque, limits)
            self._write_frame(send_id, payload)

    def send_mit_gripper(self, param: "MITParam"):
        if self._gripper_send is None:
            return
        send_id, limits = self._gripper_send
        payload = _encode_dm_mit(param.kp, param.kd, param.pos, param.vel, param.torque, limits)
        self._write_frame(send_id, payload)

    def drain(self, timeout_us: int = 2000):
        """Read all currently-available frames, keep the newest per motor, decode."""
        if self._sock is None:
            return
        timeout_s = max(timeout_us, 0) / 1e6
        latest: Dict[int, bytes] = {}
        deadline = None
        # First select honours the caller's timeout (wait for at least one frame
        # window); subsequent reads are non-blocking to drain the backlog.
        first = True
        while True:
            wait = timeout_s if first else 0.0
            try:
                r, _, _ = select.select([self._sock], [], [], wait)
            except (OSError, ValueError):
                break
            if not r:
                break
            first = False
            # Drain everything currently readable
            drained_any = False
            for _ in range(512):  # safety cap
                try:
                    frame = self._sock.recv(72)
                except BlockingIOError:
                    break
                except OSError:
                    break
                if len(frame) < 16:
                    continue
                drained_any = True
                can_id = struct.unpack_from("<I", frame, 0)[0]
                arb = can_id & (_CAN_EFF_MASK if (can_id & _CAN_EFF_FLAG) else _CAN_SFF_MASK)
                if arb in self._targets:
                    latest[arb] = frame[8:16]
            if not drained_any:
                break

        for arb, payload in latest.items():
            motor, limits = self._targets[arb]
            try:
                pos, vel, tor = _decode_dm_state(payload, limits)
                motor._sync(pos, vel, tor)
            except Exception:
                pass  # malformed frame — keep last good value

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None


@dataclass
class MITParam:
    """MIT control parameters.

    ⚠  Field order is **(kp, kd, pos, vel, torque)** — this matches the
    original xarm_can Python module, NOT the C struct (pos, vel, kp, kd, torque).
    The wrapper handles the reordering transparently.
    """
    kp:     float = 0.0
    kd:     float = 0.0
    pos:    float = 0.0
    vel:    float = 0.0
    torque: float = 0.0

    def _to_c(self) -> _CMitParam:
        """Convert to C struct with correct field order."""
        return _CMitParam(self.pos, self.vel, self.kp, self.kd, self.torque)


# ── Internal motor state cache ─────────────────────────────────────────

class _Motor:
    """A single motor whose state is refreshed from the C SDK."""
    __slots__ = ("_pos", "_vel", "_torque")

    def __init__(self):
        self._pos    = 0.0
        self._vel    = 0.0
        self._torque = 0.0

    def _sync(self, pos: float, vel: float, torque: float):
        self._pos = pos
        self._vel = vel
        self._torque = torque

    # public accessors (matching original module)
    def get_position(self) -> float:
        return self._pos

    def get_velocity(self) -> float:
        return self._vel

    def get_torque(self) -> float:
        return self._torque


class _ArmSubsystem:
    """Represents the arm joint motors (7-DOF typical)."""

    def __init__(self, xarm: "XArm"):
        self._xarm = xarm
        self._motors: List[_Motor] = []

    def _configure(self, count: int):
        self._motors = [_Motor() for _ in range(count)]

    def get_motors(self) -> List[_Motor]:
        return self._motors

    def mit_control_all(self, params: List[MITParam]):
        if self._xarm._passive:
            # SDK-less raw send: encode and write MIT frames straight to the bus.
            self._xarm._backend.send_mit_arm(params)
            return
        sdk = _get_sdk()
        n = len(params)
        c_arr = (_CMitParam * n)(*[p._to_c() for p in params])
        ret = sdk.xarm_sdk_arm_mit_control(self._xarm._handle, c_arr, n)
        if ret != 0:
            raise RuntimeError(
                f"arm_mit_control failed: {_get_last_error(sdk)}"
            )


class _GripperSubsystem:
    """Represents the gripper motor."""

    def __init__(self, xarm: "XArm"):
        self._xarm = xarm
        self._motors: List[_Motor] = []

    def _configure(self):
        self._motors = [_Motor()]

    def get_motors(self) -> List[_Motor]:
        return self._motors

    def mit_control_all(self, params: List[MITParam]):
        if self._xarm._passive:
            # SDK-less raw send: one frame per param to the gripper send_id.
            for p in params:
                self._xarm._backend.send_mit_gripper(p)
            return
        sdk = _get_sdk()
        for p in params:
            c_param = p._to_c()
            ret = sdk.xarm_sdk_gripper_mit_control(self._xarm._handle, byref(c_param))
            if ret != 0:
                raise RuntimeError(
                    f"gripper_mit_control failed: {_get_last_error(sdk)}"
                )


# ── Main class ─────────────────────────────────────────────────────────

class XArm:
    """XArm CAN bus interface.

    Usage:
        arm = XArm("can0", True)           # True = CAN-FD enabled
        arm.init_arm_motors(motor_types, send_ids, recv_ids)
        arm.init_gripper_motor(MotorType.DM4310, 0x08, 0x18)
        arm.enable_all()
        arm.set_callback_mode_all(CallbackMode.STATE)
        arm.recv_all()
        arm.refresh_all()
        arm.recv_all()
    """

    def __init__(self, can_if: str, fd: bool = True, passive: Optional[bool] = None):
        """Create an XArm CAN interface.

        passive:
            None  → use the module-level default (set_passive_mode / XARM_CAN_PASSIVE)
            True  → SDK-less raw socket: sniff STATE frames for reads and write
                    MIT command frames directly for sends. Never owns an SDK
                    handle and never uses the blocking SDK recv path, so it
                    coexists with a teleop process on the same bus without the
                    init-handshake hang. Cannot enable/disable motors (teleop
                    already enabled them).
            False → active: own an SDK handle (full control incl. enable/disable).

        NOTE: passive is per-instance. A single process can hold active objects
        and passive objects at the same time. The data collector's CANArmReader
        uses passive (read), and go_home's LeaderCANController uses passive
        (read current pos + send MIT to drive the leader) — both while teleop runs.
        """
        self._can_if = can_if
        self._passive = _PASSIVE_MODE if passive is None else bool(passive)
        self._backend: Optional[_PassiveCANBackend] = None
        self._handle = c_void_p()

        if self._passive:
            # No SDK handle — open a raw passive socket instead. This is what
            # lets the collector coexist with a teleop process on the same bus.
            self._backend = _PassiveCANBackend(can_if, fd)
        else:
            sdk = _get_sdk()
            ret = sdk.xarm_sdk_create(
                can_if.encode("utf-8"), 1 if fd else 0, byref(self._handle),
            )
            if ret != 0:
                raise RuntimeError(
                    f"xarm_sdk_create('{can_if}') failed: {_get_last_error(sdk)}"
                )

        self._arm      = _ArmSubsystem(self)
        self._gripper  = _GripperSubsystem(self)
        self._arm_motor_count = 0
        self._has_gripper     = False

    # ── Initialisation ─────────────────────────────────────────────────

    def init_arm_motors(
        self,
        motor_types: List[MotorType],
        send_ids: List[int],
        recv_ids: List[int],
    ):
        n = len(motor_types)
        self._arm._configure(n)
        self._arm_motor_count = n

        if self._passive:
            # No SDK call — map each motor's recv_id (for STATE routing) and
            # send_id (for MIT command encoding) into the raw backend.
            for i in range(n):
                self._backend.register(
                    recv_ids[i], self._arm._motors[i], int(motor_types[i]),
                    send_id=send_ids[i],
                )
            return

        sdk = _get_sdk()
        c_types = (c_int * n)(*[int(t) for t in motor_types])
        c_send  = (c_uint32 * n)(*send_ids)
        c_recv  = (c_uint32 * n)(*recv_ids)

        ret = sdk.xarm_sdk_init_arm_motors(
            self._handle, c_types, c_send, c_recv, n,
        )
        if ret != 0:
            raise RuntimeError(
                f"init_arm_motors failed: {_get_last_error(sdk)}"
            )

    def init_gripper_motor(
        self, motor_type: MotorType, send_id: int, recv_id: int,
    ):
        self._gripper._configure()
        self._has_gripper = True

        if self._passive:
            self._backend.register(
                recv_id, self._gripper._motors[0], int(motor_type),
                send_id=send_id, is_gripper=True,
            )
            return

        sdk = _get_sdk()
        ret = sdk.xarm_sdk_init_gripper_motor(
            self._handle, int(motor_type), send_id, recv_id,
        )
        if ret != 0:
            self._has_gripper = False
            raise RuntimeError(
                f"init_gripper_motor failed: {_get_last_error(sdk)}"
            )

    # ── Mode & enable ──────────────────────────────────────────────────

    def set_callback_mode_all(self, mode: CallbackMode):
        # Passive mode: we never command the motors; teleop already put them in
        # STATE mode. Nothing to do (and nothing to send that could disturb it).
        if self._passive:
            return
        sdk = _get_sdk()
        ret = sdk.xarm_sdk_set_callback_mode_state_all(self._handle)
        if ret != 0:
            raise RuntimeError(
                f"set_callback_mode_all failed: {_get_last_error(sdk)}"
            )

    def enable_all(self):
        if self._passive:
            raise RuntimeError("enable_all() is not allowed in passive (read-only) mode")
        sdk = _get_sdk()
        ret = sdk.xarm_sdk_enable_all(self._handle)
        if ret != 0:
            raise RuntimeError(f"enable_all failed: {_get_last_error(sdk)}")

    def disable_all(self):
        if self._passive:
            raise RuntimeError("disable_all() is not allowed in passive (read-only) mode")
        sdk = _get_sdk()
        ret = sdk.xarm_sdk_disable_all(self._handle)
        if ret != 0:
            raise RuntimeError(f"disable_all failed: {_get_last_error(sdk)}")

    # ── Communication ──────────────────────────────────────────────────

    def refresh_all(self):
        """Send refresh commands to all motors (does NOT update local cache).

        In passive mode this is a deliberate no-op: the collector must not
        transmit anything onto a bus that teleop is driving.
        """
        if self._passive:
            return
        sdk = _get_sdk()
        ret = sdk.xarm_sdk_refresh_all(self._handle)
        if ret != 0:
            raise RuntimeError(f"refresh_all failed: {_get_last_error(sdk)}")

    def recv_all(self, timeout_us: int = 2000):
        """Receive CAN responses and sync local motor state cache."""
        if self._passive:
            self._backend.drain(timeout_us)
            return
        sdk = _get_sdk()
        ret = sdk.xarm_sdk_recv_all(self._handle, timeout_us)
        if ret != 0:
            raise RuntimeError(f"recv_all failed: {_get_last_error(sdk)}")
        self._sync_motor_states()

    def _sync_motor_states(self):
        """Pull latest joint states from C SDK → Python motor objects."""
        sdk = _get_sdk()

        # Arm motors
        if self._arm_motor_count > 0 and len(self._arm._motors) > 0:
            n = self._arm_motor_count
            states = (_CJointState * n)()
            ret = sdk.xarm_sdk_get_arm_joint_states(self._handle, states, n)
            if ret == 0:
                for i, m in enumerate(self._arm._motors):
                    m._sync(states[i].pos, states[i].vel, states[i].torque)

        # Gripper motor
        if self._has_gripper and len(self._gripper._motors) > 0:
            state = _CJointState()
            ret = sdk.xarm_sdk_get_gripper_state(self._handle, byref(state))
            if ret == 0:
                self._gripper._motors[0]._sync(state.pos, state.vel, state.torque)

    # ── Accessors ──────────────────────────────────────────────────────

    def get_arm(self) -> _ArmSubsystem:
        return self._arm

    def get_gripper(self) -> _GripperSubsystem:
        return self._gripper

    def get_gripper_state(self) -> float:
        """Return gripper motor position (convenience, mirrors original API)."""
        if self._passive:
            if self._has_gripper and self._gripper._motors:
                return float(self._gripper._motors[0].get_position())
            return 0.0
        sdk = _get_sdk()
        state = _CJointState()
        ret = sdk.xarm_sdk_get_gripper_state(self._handle, byref(state))
        if ret != 0:
            raise RuntimeError(
                f"get_gripper_state failed: {_get_last_error(sdk)}"
            )
        return float(state.pos)

    # ── Cleanup ────────────────────────────────────────────────────────

    def close(self):
        """Explicitly destroy the native handle / close the passive socket."""
        if self._passive:
            if self._backend is not None:
                self._backend.close()
                self._backend = None
            return
        if hasattr(self, "_handle") and self._handle:
            try:
                sdk = _get_sdk()
                sdk.xarm_sdk_destroy(self._handle)
            except Exception:
                pass
            self._handle = None

    def __del__(self):
        self.close()
