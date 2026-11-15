#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dxl.py — 主机侧上电测试工具(imu_to_ttl, Dynamixel Protocol 2.0 / XL330 固件)

用一根 USB-TTL(如 U2D2 / FE-URT-2,1 Mbps)把本板当成一颗 Dynamixel 从机来测,
不需要 HAT、不需要主控、不需要电池(板子逻辑侧由 J1 限流电源供)。

覆盖 Dynamixel 版固件的验收项目:
    ping        ID 200 在不在总线上,型号/固件版本对不对(逐字节)
    ident       身份寄存器(型号 0、固件 6、ID 7、波特率 8)
    imu         读 124/20:12 字节姿态块 + 8 字节诊断,解码 gyro dps + 四元数
    stream      连续解码(默认 20 Hz)
    sync        广播 Sync Read (0x82) —— 逐设备状态包
    fast        广播 Fast Sync Read (0x8A) —— 单包追加块,duck-control 的默认读法
    coexist     与一颗真实 XL330 同车读(同一帧里本板 + 舵机)
    resync      截断帧后能否重新同步
    stress      压测并统计错误
    run         按验收顺序跑全套,PASS/FAIL 收尾

纯标准库(termios + select),不依赖 pyserial。

用法:
    dxl.py ports                       列出候选串口
    dxl.py selftest                    校验封包/解析/解码向量(不需要硬件)
    dxl.py ping
    dxl.py ident
    dxl.py imu
    dxl.py stream --hz 20 --count 100
    dxl.py sync --ids 200,10,11
    dxl.py fast --ids 200,10,11
    dxl.py coexist --servo 1
    dxl.py stress --count 20000
    dxl.py run --servo 1

接法(J3/J4/J5 任一口):
    URT-2 的信号线 → 该口 pin1(总线数据) · GND → pin3
    板子供电:J1 接限流电源 8.4 V / 100 mA(或 2S 电池)
    信号线不要接 pin2(VBAT),那是电池正极。
"""

import argparse
import glob
import math
import os
import select
import struct
import sys
import termios
import time

# ---------------------------------------------------------------- 契约常量
# 与 firmware/include/dxl_config.h 一一对应;主机侧独立写死是**故意的**:
# 这些是对外契约,改了就是破坏主机,仿真门和本工具都应当报错而不是跟着变。

ID_IMU = 200
BAUDRATE = 1000000
BLOCK_ADDR = 124            # 0x7C,12 字节姿态块(原版 imu_to_dxl 的地址)
BLOCK_LEN = 12
DIAG_ADDR = 136             # 0x88,紧跟姿态块的 8 字节诊断
DIAG_LEN = 8
WINDOW_LEN = BLOCK_LEN + DIAG_LEN
MODEL_NUMBER = 10200
FW_VERSION = 1
BAUD_CODE_1M = 3

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83
INST_FAST_SYNC_READ = 0x8A
INST_STATUS = 0x55
BROADCAST = 0xFE

A_MODEL_LO = 0
A_FW_VERSION = 6
A_ID = 7
A_BAUD = 8
A_RETURN_DELAY = 9

# LSM6DSV16X 量程与灵敏度(固件 CTRL6=0x02 → ±500 dps;CTRL8=0x01 → ±4 g)
GYRO_DPS_PER_LSB = 0.0175
ACCEL_G_PER_LSB = 0.000122


# ---------------------------------------------------------------- 协议编解码

def crc16(data, crc=0):
    """Dynamixel Protocol 2.0 的 CRC-16:MSB-first,poly 0x8005,init 0。"""
    for b in data:
        crc ^= (b & 0xFF) << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _pattern_run(run, b):
    if run == 0:
        return 1 if b == 0xFF else 0
    if run == 1:
        return 2 if b == 0xFF else 0
    return 3 if b == 0xFD else (2 if b == 0xFF else 0)


def stuff(data):
    """字节填充:FF FF FD → FF FF FD FD(扫过的字节不再参与匹配)。"""
    out = bytearray()
    run = 0
    for b in data:
        run = _pattern_run(run, b)
        out.append(b)
        if run == 3:
            out.append(0xFD)
            run = 0
    return bytes(out)


def unstuff(data):
    out = bytearray()
    run = 0
    for b in data:
        if run == 3:
            run = 0
            if b == 0xFD:
                continue
        run = _pattern_run(run, b)
        out.append(b)
    return bytes(out)


def frame(sid, inst, params=b""):
    """指令包:FF FF FD 00 ID LEN INST PARAMS CRC,LEN 计 INST+填充后参数+CRC。"""
    wire = stuff(bytes(params))
    ln = 1 + len(wire) + 2
    body = bytes([sid, ln & 0xFF, ln >> 8, inst]) + wire
    pkt = b"\xff\xff\xfd\x00" + body
    c = crc16(pkt)
    return pkt + bytes([c & 0xFF, c >> 8])


def status_packets(buf):
    """把一段总线字节拆成 (id, err, params, 原始字节) 列表,只保留 CRC 正确的。"""
    out = []
    i = 0
    n = len(buf)
    while i + 11 <= n:
        if buf[i:i + 4] != b"\xff\xff\xfd\x00":
            i += 1
            continue
        ln = buf[i + 5] | (buf[i + 6] << 8)
        total = 7 + ln
        if i + total > n:
            break
        if buf[i + 7] != INST_STATUS:
            i += 1
            continue
        want = buf[i + total - 2] | (buf[i + total - 1] << 8)
        if crc16(buf[i:i + total - 2]) != want:
            i += 1
            continue
        body = unstuff(buf[i + 8:i + total - 2])
        out.append((buf[i + 4], body[0], body[1:], bytes(buf[i:i + total])))
        i += total
    return out


def parse_fast_sync(buf, ids, length):
    """按主机库(rustypot)的方式解析 Fast Sync Read 应答:一个广播包头 +
    每设备一个 [ERR ID DATA CRC] 块,块 CRC 累积到本块数据末尾,不做去填充。

    返回 (blocks, reason):blocks 是 {id: (err, data)},缺失的设备不出现在里面。
    """
    block_size = length + 4
    if len(buf) < 8 or buf[:4] != b"\xff\xff\xfd\x00" or buf[4] != BROADCAST or buf[7] != INST_STATUS:
        return {}, "no fast-sync header"
    ln = buf[5] | (buf[6] << 8)
    total = 7 + ln
    if len(buf) < total:
        return {}, "short packet %d/%d" % (len(buf), total)
    blocks = {}
    for i, sid in enumerate(ids):
        b = 8 + i * block_size
        crc_at = b + 2 + length
        if crc_at + 2 > total:
            break
        got = buf[crc_at] | (buf[crc_at + 1] << 8)
        if crc16(buf[:crc_at]) != got or buf[b + 1] != sid:
            return blocks, "bad block %d" % i
        blocks[sid] = (buf[b], buf[b + 2:crc_at])
    return blocks, "ok"


# ---------------------------------------------------------------- 串口底层

def find_port(explicit=None):
    if explicit:
        if not os.path.exists(explicit):
            raise SystemExit("串口不存在: %s" % explicit)
        return explicit
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    pref = [p for p in by_id if "if00" in p]
    cands = pref or by_id
    if not cands:
        cands = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not cands:
        raise SystemExit("找不到串口。插上 USB-TTL,或用 -p 指定。")
    return cands[0]


def open_port(dev, baud=BAUDRATE):
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except PermissionError:
        raise SystemExit("打开 %s 被拒绝 —— 需要 udev uaccess 规则或加入 dialout 组。" % dev)
    a = termios.tcgetattr(fd)
    a[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
              termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
    a[1] &= ~termios.OPOST
    a[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    a[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
    speed = getattr(termios, "B%d" % baud, None)
    if speed is None:
        raise SystemExit("termios 不支持波特率 %d" % baud)
    a[4] = a[5] = speed
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


class Bus(object):
    def __init__(self, dev, baud=BAUDRATE, timeout=0.03, verbose=False):
        self.dev = dev
        self.fd = open_port(dev, baud)
        self.timeout = timeout
        self.verbose = verbose

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass

    def send(self, pkt, collect_ms=None, until=None):
        """发一包,收满 `collect_ms` 毫秒(默认 timeout),返回字节串。

        until 给定时,一旦已收字节满足该函数(例如"16 个块都齐了")就提前返回。
        """
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        t0 = time.perf_counter()
        os.write(self.fd, pkt)
        deadline = t0 + (self.timeout if collect_ms is None else collect_ms / 1000.0)
        buf = bytearray()
        while True:
            left = deadline - time.perf_counter()
            if left <= 0:
                break
            r, _, _ = select.select([self.fd], [], [], left)
            if r:
                try:
                    chunk = os.read(self.fd, 1024)
                except BlockingIOError:
                    continue
                if chunk:
                    buf += chunk
                    if until is not None and until(bytes(buf)):
                        break
        return bytes(buf), time.perf_counter() - t0

    def ping(self, sid):
        buf, rtt = self.send(frame(sid, INST_PING),
                             until=lambda b: any(f[0] == sid for f in status_packets(b)))
        for fid, err, params, raw in status_packets(buf):
            if fid == sid:
                return err, params, rtt
        return None, buf, rtt

    def read(self, sid, addr, n):
        buf, rtt = self.send(frame(sid, INST_READ, bytes([addr & 0xFF, addr >> 8, n & 0xFF, n >> 8])),
                             until=lambda b: any(f[0] == sid for f in status_packets(b)))
        for fid, err, params, raw in status_packets(buf):
            if fid == sid:
                return err, params, rtt
        return None, buf, rtt

    def write(self, sid, addr, data):
        buf, rtt = self.send(frame(sid, INST_WRITE, bytes([addr & 0xFF, addr >> 8]) + bytes(data)),
                             until=lambda b: any(f[0] == sid for f in status_packets(b)))
        for fid, err, params, raw in status_packets(buf):
            if fid == sid:
                return err, params, rtt
        return None, buf, rtt

    def sync_read(self, ids, addr, n):
        req = frame(BROADCAST, INST_SYNC_READ,
                    bytes([addr & 0xFF, addr >> 8, n & 0xFF, n >> 8]) + bytes(ids))
        need = len(ids)
        buf, rtt = self.send(req, until=lambda b: len(status_packets(b)) >= need)
        got = {}
        for fid, err, params, raw in status_packets(buf):
            got.setdefault(fid, (err, params))
        return got, rtt

    def fast_sync_read(self, ids, addr, n):
        req = frame(BROADCAST, INST_FAST_SYNC_READ,
                    bytes([addr & 0xFF, addr >> 8, n & 0xFF, n >> 8]) + bytes(ids))
        total = 8 + len(ids) * (n + 4)
        buf, rtt = self.send(req, until=lambda b: len(b) >= total)
        return parse_fast_sync(buf, ids, n), rtt


# ---------------------------------------------------------------- 解码

def half_to_float(h):
    return struct.unpack("<e", struct.pack("<H", h))[0]


def decode_block(block, diag=None):
    gyro = [struct.unpack_from("<h", block, 2 * i)[0] for i in range(3)]
    qh = [struct.unpack_from("<H", block, 6 + 2 * i)[0] for i in range(3)]
    q = [half_to_float(v) for v in qh]
    w2 = 1.0 - sum(c * c for c in q)
    w = math.sqrt(w2) if w2 > 0 else 0.0
    out = {
        "gyro_raw": gyro,
        "gyro_dps": [g * GYRO_DPS_PER_LSB for g in gyro],
        "quat_raw": qh,
        "quat": [w] + q,
        "quat_norm": math.sqrt(w * w + sum(c * c for c in q)),
        "sflp_ready": any(qh),
        "euler_deg": None,
        "sample_count": None,
        "status": None,
        "accel_raw": None,
    }
    if out["sflp_ready"]:
        x, y, z = q
        out["euler_deg"] = (
            math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))),
            math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))),
        )
    if diag and len(diag) >= 3:
        out["sample_count"] = struct.unpack_from("<H", diag, 0)[0]
        out["status"] = diag[2]
        if len(diag) >= 8:
            out["accel_raw"] = [struct.unpack_from("<h", diag, 4 + 2 * i)[0] for i in range(2)]
    return out


def fmt_block(info):
    g = " ".join("%+7.1f" % v for v in info["gyro_dps"])
    q = " ".join("%+.3f" % v for v in info["quat"])
    s = "gyro dps [%s] quat [%s] |q|=%.3f" % (g, q, info["quat_norm"])
    if info["euler_deg"]:
        s += " rpy [%s]" % " ".join("%+6.1f" % v for v in info["euler_deg"])
    if info["sample_count"] is not None:
        s += " n=%d st=0x%02X" % (info["sample_count"], info["status"])
    return s


def parse_ids(text):
    if not text:
        return [ID_IMU]
    return [int(v, 0) for v in text.replace(" ", "").split(",") if v != ""]


# ---------------------------------------------------------------- 子命令

def cmd_ports(_args):
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    print("by-id 路径:" if by_id else "by-id 路径:(无)")
    for p in by_id:
        print("  %s" % p)
    nodes = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    print("内核 tty 节点:" if nodes else "内核 tty 节点:(无)")
    for p in nodes:
        print("  %s" % p)
    return 0


def cmd_selftest(_args):
    """不需要硬件的封包 / 解析 / 解码向量校验。"""
    ok = True

    def check(cond, name, detail=""):
        nonlocal ok
        print(("PASS  " if cond else "FAIL  ") + name + (("  [%s]" % detail) if detail else ""))
        if not cond:
            ok = False

    # ROBOTIS 手册的固定向量
    check(crc16(b"\xff\xff\xfd\x00\x01\x03\x00\x01") == 0x4E19, "CRC ping(ID1) = 0x4E19")
    check(crc16(b"\xff\xff\xfd\x00\x01\x07\x00\x02\x84\x00\x04\x00") == 0x151D,
          "CRC read(ID1,132/4) = 0x151D")
    fsr = bytes([0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x19, 0x00, 0x55,
                 0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x84, 0x08,
                 0x00, 0x07, 0x1F, 0x08, 0x00, 0x00, 0x16, 0xCA,
                 0x00, 0x04, 0xFF, 0x03, 0x00, 0x00, 0xD1, 0x9E])
    check(crc16(fsr[:14]) == 0x0884 and crc16(fsr[:22]) == 0xCA16 and crc16(fsr[:30]) == 0x9ED1,
          "fast sync 手册示例的逐块累积 CRC")
    blocks, why = parse_fast_sync(fsr, [3, 7, 4], 4)
    check(why == "ok" and blocks.get(3, (None, b""))[1] == b"\xA6\x00\x00\x00",
          "fast sync 手册示例解析", why)

    # 字节填充
    check(stuff(b"\xff\xff\xfd\x07") == b"\xff\xff\xfd\xfd\x07", "stuff FF FF FD")
    check(stuff(b"\xff\xff\xfd\xfd") == b"\xff\xff\xfd\xfd\xfd", "stuff FF FF FD FD")
    check(unstuff(stuff(bytes(range(256)))) == bytes(range(256)), "stuff/unstuff 往返")

    # 本板的固定封包
    check(frame(ID_IMU, INST_PING) == bytes.fromhex("FF FF FD 00 C8 03 00 01 3B FA"),
          "ping 请求字节固定")
    want_rsp = bytes.fromhex("FF FF FD 00 C8 07 00 55 00 D8 27 01 B2 8B")
    got = status_packets(want_rsp)
    check(got and got[0][0] == ID_IMU and got[0][2] == bytes.fromhex("D8 27 01"),
          "ping 应答解析(型号 %d + 固件 %d)" % (MODEL_NUMBER, FW_VERSION))

    # 解码:四元数 x=0.5 y=-0.5 z=0(整数值来自仿真模型)
    block = struct.pack("<hhhHHH", 100, -200, 300, 0x3800, 0xB800, 0x0000)
    info = decode_block(block)
    check(abs(info["quat"][1] - 0.5) < 1e-6 and abs(info["quat"][2] + 0.5) < 1e-6
          and abs(info["quat"][0] - math.sqrt(0.5)) < 1e-6,
          "姿态块解码", "w=%.4f" % info["quat"][0])

    print("\nselftest %s" % ("passed" if ok else "FAILED"))
    return 0 if ok else 1


def connect(args):
    dev = find_port(getattr(args, "port", None))
    return Bus(dev, timeout=getattr(args, "timeout", 30) / 1000.0)


def cmd_ping(args):
    bus = connect(args)
    err, params, rtt = bus.ping(ID_IMU)
    bus.close()
    if err is None:
        print("FAIL  ID %d 没有应答(buf=%s)" % (ID_IMU, params[:24].hex(":")))
        return 1
    model = params[0] | (params[1] << 8) if len(params) >= 2 else None
    fw = params[2] if len(params) >= 3 else None
    ok = err == 0 and model == MODEL_NUMBER and fw == FW_VERSION
    print("%s  ping ID %d: err=0x%02X model=%s fw=%s rtt=%.1f ms"
          % ("PASS" if ok else "FAIL", ID_IMU, err, model, fw, rtt * 1e3))
    return 0 if ok else 1


def cmd_ident(args):
    bus = connect(args)
    err, model_b, _ = bus.read(ID_IMU, A_MODEL_LO, 2)
    err2, ident_b, _ = bus.read(ID_IMU, A_FW_VERSION, 3)
    bus.close()
    ok = err == 0 and err2 == 0 and len(model_b) == 2 and len(ident_b) == 3
    if not ok:
        print("FAIL  ident 读失败 err=0x%02X/0x%02X data=%s/%s"
              % (err or 0xFF, err2 or 0xFF, model_b[:8].hex(), ident_b[:8].hex()))
        return 1
    model = model_b[0] | (model_b[1] << 8)
    print("model=%d (want %d)  fw=%d (want %d)  id=%d (want %d)  baud_code=%d (want %d)"
          % (model, MODEL_NUMBER, ident_b[0], FW_VERSION, ident_b[1], ID_IMU,
             ident_b[2], BAUD_CODE_1M))
    ok = (model == MODEL_NUMBER and ident_b[0] == FW_VERSION
          and ident_b[1] == ID_IMU and ident_b[2] == BAUD_CODE_1M)
    print("PASS  ident" if ok else "FAIL  ident")
    return 0 if ok else 1


def cmd_imu(args):
    bus = connect(args)
    err, data, rtt = bus.read(ID_IMU, BLOCK_ADDR, WINDOW_LEN)
    bus.close()
    if err != 0 or len(data) != WINDOW_LEN:
        print("FAIL  imu 读失败 err=0x%02X len=%d" % (err if err is not None else 0xFF, len(data)))
        return 1
    info = decode_block(data[:BLOCK_LEN], data[BLOCK_LEN:])
    print("PASS  imu  " + fmt_block(info))
    if info["sample_count"] is not None and info["sample_count"] == 0:
        print("WARN  sample_count 为 0 —— SFLP 可能还没出新样本")
    return 0


def cmd_stream(args):
    bus = connect(args)
    period = 1.0 / args.hz
    last = None
    same = 0
    bad = 0
    for i in range(args.count):
        t0 = time.perf_counter()
        err, data, rtt = bus.read(ID_IMU, BLOCK_ADDR, WINDOW_LEN)
        if err != 0 or len(data) != WINDOW_LEN:
            bad += 1
            print("FAIL  #%d err=0x%02X len=%d" % (i, err if err is not None else 0xFF, len(data)))
        else:
            if data[:BLOCK_LEN] == last:
                same += 1
            last = data[:BLOCK_LEN]
            info = decode_block(data[:BLOCK_LEN], data[BLOCK_LEN:])
            print("#%3d  %s" % (i, fmt_block(info)))
        dt = period - (time.perf_counter() - t0)
        if dt > 0:
            time.sleep(dt)
    bus.close()
    print("stream: %d 次,%d 次失败,%d 次样本未刷新" % (args.count, bad, same))
    return 0 if bad == 0 else 1


def _print_sync_result(title, got, ids, rtt, expect_len):
    ok = True
    print("%s (rtt %.1f ms):" % (title, rtt * 1e3))
    for sid in ids:
        if sid not in got:
            print("  id %3d: 无应答" % sid)
            ok = False
            continue
        err, data = got[sid]
        tail = " err=0x%02X" % err
        if err != 0:
            ok = False
        if sid == ID_IMU and len(data) >= BLOCK_LEN:
            tail += "  " + fmt_block(decode_block(data[:BLOCK_LEN],
                                                  data[BLOCK_LEN:BLOCK_LEN + DIAG_LEN]))
        elif len(data) != expect_len:
            print("  id %3d: 数据长度 %d(期望 %d)%s" % (sid, len(data), expect_len, tail))
            ok = False
            continue
        print("  id %3d:%s" % (sid, tail))
    return ok


def cmd_sync(args):
    ids = parse_ids(args.ids)
    bus = connect(args)
    got, rtt = bus.sync_read(ids, BLOCK_ADDR, BLOCK_LEN)
    bus.close()
    ok = _print_sync_result("Sync Read 0x82 @124/12", got, ids, rtt, BLOCK_LEN) and len(got) >= len(ids)
    print("PASS  sync" if ok else "FAIL  sync")
    return 0 if ok else 1


def cmd_fast(args):
    ids = parse_ids(args.ids)
    bus = connect(args)
    (blocks, why), rtt = bus.fast_sync_read(ids, BLOCK_ADDR, BLOCK_LEN)
    bus.close()
    got = dict(blocks)
    ok = why == "ok" and _print_sync_result("Fast Sync Read 0x8A @124/12", got, ids, rtt, BLOCK_LEN)
    if why != "ok":
        print("  parse: %s" % why)
    print("PASS  fast" if ok else "FAIL  fast")
    return 0 if ok else 1


def cmd_coexist(args):
    """同一帧里读本板 + 一颗真实 XL330 —— duck-control 每 tick 的读法。"""
    ids = [ID_IMU, args.servo]
    bus = connect(args)
    print("同车 id: %s" % ids)
    ping = []
    for sid in ids:
        err, params, rtt = bus.ping(sid)
        ok = err == 0
        ping.append(ok)
        print("  id %3d ping: %s model=%s fw=%s" % (
            sid, "PASS" if ok else "FAIL",
            (params[0] | (params[1] << 8)) if len(params) >= 2 else "-",
            params[2] if len(params) >= 3 else "-"))
    (blocks, why), rtt = bus.fast_sync_read(ids, BLOCK_ADDR, BLOCK_LEN)
    bus.close()
    ok = why == "ok" and len(blocks) == len(ids)
    if ok:
        _, data_imu = blocks[ID_IMU]
        _, data_srv = blocks[args.servo]
        print("  fast sync (rtt %.1f ms):" % (rtt * 1e3))
        if len(data_imu) >= BLOCK_LEN:
            print("    id %3d  %s" % (ID_IMU, fmt_block(decode_block(data_imu[:BLOCK_LEN]))))
        print("    id %3d  present pwm/current/velocity/position = %s" % (args.servo, data_srv.hex(":")))
    else:
        print("  fast sync 失败: %s(blocks=%s)" % (why, sorted(blocks)))
    ok = ok and all(ping)
    print("PASS  coexist" if ok else "FAIL  coexist")
    return 0 if ok else 1


def cmd_resync(args):
    bus = connect(args)
    # 一帧被截断的请求,紧跟着一条正常 ping:板上解析器必须靠包间隙/重同步恢复
    cut = frame(ID_IMU, INST_READ, bytes([BLOCK_ADDR & 0xFF, BLOCK_ADDR >> 8, BLOCK_LEN, 0]))[:-3]
    termios.tcflush(bus.fd, termios.TCIOFLUSH)
    os.write(bus.fd, cut)
    time.sleep(0.005)
    err, params, rtt = bus.ping(ID_IMU)
    bus.close()
    ok = err == 0
    print("%s  截断帧后 ping" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def cmd_stress(args):
    bus = connect(args)
    ids = [ID_IMU] + [v for v in range(1, 16)]
    errors = 0
    t0 = time.perf_counter()
    last = None
    stale = 0
    for i in range(args.count):
        err, params, _ = bus.ping(ID_IMU)
        if err != 0 or len(params) != 3:
            errors += 1
            if errors <= 5:
                print("  ping #%d 失败 err=%s params=%s" % (i, err, params[:8].hex()))
        if i % 5 == 0:
            (blocks, why), _ = bus.fast_sync_read(ids, BLOCK_ADDR, BLOCK_LEN)
            if why != "ok" or ID_IMU not in blocks:
                errors += 1
                if errors <= 5:
                    print("  fast sync #%d 失败: %s" % (i, why))
            else:
                blk = blocks[ID_IMU][1][:BLOCK_LEN]
                if blk == last:
                    stale += 1
                last = blk
    dt = time.perf_counter() - t0
    bus.close()
    print("stress: %d 次 ping + %d 次 fast sync,%.1f s,错误 %d,样本未刷新 %d"
          % (args.count, args.count // 5 + 1, dt, errors, stale))
    return 0 if errors == 0 else 1


def cmd_run(args):
    """按验收顺序跑全套。"""
    bus = connect(args)
    results = []

    def step(name, cond, detail=""):
        print("%s  %s%s" % ("PASS" if cond else "FAIL", name, ("  [%s]" % detail) if detail else ""))
        results.append((name, bool(cond)))

    err, params, rtt = bus.ping(ID_IMU)
    model = (params[0] | (params[1] << 8)) if (err == 0 and len(params) >= 2) else None
    step("ping ID %d" % ID_IMU, err == 0,
         "err=0x%02X rtt=%.1f ms" % (err if err is not None else 0xFF, rtt * 1e3))
    step("model = %d" % MODEL_NUMBER, model == MODEL_NUMBER, "got %s" % model)

    err, data, _ = bus.read(ID_IMU, BLOCK_ADDR, WINDOW_LEN)
    step("read 124/20", err == 0 and len(data) == WINDOW_LEN,
         "err=0x%02X len=%d" % (err if err is not None else 0xFF, len(data)))
    first = data[:BLOCK_LEN] if len(data) >= BLOCK_LEN else b""
    if first:
        info = decode_block(first, data[BLOCK_LEN:])
        step("block decodes", info["quat_norm"] > 0.5, fmt_block(info))

    time.sleep(0.05)
    err, data2, _ = bus.read(ID_IMU, BLOCK_ADDR, WINDOW_LEN)
    step("fresh samples", err == 0 and data2[:BLOCK_LEN] != first,
         "%s -> %s" % (first[:6].hex(":"), data2[:6].hex(":")))

    ids = [ID_IMU] + list(range(1, 16))
    got, _ = bus.sync_read(ids, BLOCK_ADDR, BLOCK_LEN)
    step("sync read (16 ids) 有本板", ID_IMU in got and len(got[ID_IMU][1]) == BLOCK_LEN)
    (blocks, why), _ = bus.fast_sync_read(ids, BLOCK_ADDR, BLOCK_LEN)
    step("fast sync read (16 ids) 有本板", why == "ok" and ID_IMU in blocks,
         "why=%s have=%d" % (why, len(blocks)))

    if args.servo:
        (blocks, why), _ = bus.fast_sync_read([ID_IMU, args.servo], BLOCK_ADDR, BLOCK_LEN)
        step("coexist with servo %d" % args.servo, why == "ok" and len(blocks) == 2,
             "why=%s have=%s" % (why, sorted(blocks)))

    cut = frame(ID_IMU, INST_READ, bytes([BLOCK_ADDR & 0xFF, BLOCK_ADDR >> 8, BLOCK_LEN, 0]))[:-3]
    termios.tcflush(bus.fd, termios.TCIOFLUSH)
    os.write(bus.fd, cut)
    time.sleep(0.005)
    err, _, _ = bus.ping(ID_IMU)
    step("resync after truncated frame", err == 0, "err=0x%02X" % (err if err is not None else 0xFF))

    bad = 0
    for _ in range(args.stress):
        err, params, _ = bus.ping(ID_IMU)
        if err != 0 or len(params) != 3:
            bad += 1
    step("stress %d pings" % args.stress, bad == 0, "bad=%d" % bad)
    bus.close()

    failed = [n for n, ok in results if not ok]
    print("\nrun: %d/%d passed%s" % (len(results) - len(failed), len(results),
                                     (", 失败: " + ", ".join(failed)) if failed else ""))
    return 0 if not failed else 1


def main(argv=None):
    p = argparse.ArgumentParser(description="imu_to_ttl Dynamixel (XL330) 固件上电测试")
    p.add_argument("-p", "--port", help="串口设备(默认自动找)")
    p.add_argument("--timeout", type=int, default=30, help="单次收包超时 ms(默认 30)")
    sub = p.add_subparsers(dest="cmd")

    sub.add_parser("ports", help="列出候选串口")
    sub.add_parser("selftest", help="校验向量(不需要硬件)")
    sub.add_parser("ping", help="ID 200 在不在")
    sub.add_parser("ident", help="身份寄存器")
    sub.add_parser("imu", help="读并解码 124/20 窗口")

    sp = sub.add_parser("stream", help="连续解码")
    sp.add_argument("--hz", type=float, default=20.0)
    sp.add_argument("--count", type=int, default=100)

    for name, help_text in (("sync", "广播 Sync Read (0x82)"),
                            ("fast", "广播 Fast Sync Read (0x8A)")):
        sp = sub.add_parser(name, help=help_text)
        sp.add_argument("--ids", default="", help="逗号分隔,默认只读 200")

    sp = sub.add_parser("coexist", help="与一颗真实 XL330 同车读")
    sp.add_argument("--servo", type=int, required=True, help="同车的真实 XL330 ID")

    sp = sub.add_parser("stress", help="压测 ping + fast sync")
    sp.add_argument("--count", type=int, default=20000)

    sub.add_parser("resync", help="截断帧后重新同步")

    sp = sub.add_parser("run", help="按验收顺序跑全套")
    sp.add_argument("--servo", type=int, default=None, help="同车的真实 XL330 ID(可选)")
    sp.add_argument("--stress", type=int, default=2000)

    args = p.parse_args(argv)
    handlers = {
        "ports": cmd_ports,
        "selftest": cmd_selftest,
        "ping": cmd_ping,
        "ident": cmd_ident,
        "imu": cmd_imu,
        "stream": cmd_stream,
        "sync": cmd_sync,
        "fast": cmd_fast,
        "coexist": cmd_coexist,
        "resync": cmd_resync,
        "stress": cmd_stress,
        "run": cmd_run,
    }
    if not args.cmd:
        p.print_help()
        return 1
    if args.cmd == "run" and args.servo is None:
        print("(未给 --servo,跳过与真实舵机同车的检查)")
    return handlers[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
