#!/usr/bin/env python3
"""Official RemoteIo v1 endpoint backed by this robot's calibrated Dynamixel bus.

The official daemon calls this interface --sim; this endpoint is REAL HARDWARE.
Default observe mode rejects torque-on and permits only read packets on the wire.
Shadow mode evaluates the whole official controller but discards all motor writes.
Live mode must be explicitly selected by the supervised runner. Nothing auto-arms.
"""
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import select
import socket
import struct
import sys
import termios
import threading
import time

from policy_coordinates import Coordinates, TrunkImu, IDS, CONTRACT

ROOT = Path(__file__).resolve().parent
BASE = ROOT.parent
LOCAL = BASE / 'local-joint-viewer' if (BASE / 'local-joint-viewer').exists() else BASE
sys.path.insert(0, str(LOCAL))
sys.path.insert(0, str(BASE / 'tools'))
sys.path.insert(0, str(BASE / 'mumu-duck/imu-to-ttl/tools'))
from joint_limits import JointLimits
import dxl

# Mode is the sole EEPROM field this adapter may change. Homing, drive direction,
# device identity, limits and shutdown flags are deliberately absent.
FIELDS = {11: (1, False), 76: (2, False), 78: (2, False),
          80: (2, False), 82: (2, False), 84: (2, False),
          88: (2, False), 90: (2, False), 98: (1, True),
          100: (2, True), 102: (2, True), 108: (4, False), 112: (4, False)}
WIDTHS = {**{a: s[0] for a, s in FIELDS.items()}, 64: 1, 116: 4}
TIMEOUT = .25
# The IMU also hears every servo status packet. Its firmware resets a partial
# frame after a >=2 ms byte gap. A measured 2.5 ms quiet window before its own
# request avoids losing that request after a burst of 15 servo replies.
IMU_QUIET_SECONDS = .0025
_CRC_TABLE = tuple(dxl.crc16(bytes([value])) for value in range(256))


def fast_crc16(data, crc=0):
    for value in data:
        crc = ((crc << 8) & 0xffff) ^ _CRC_TABLE[((crc >> 8) ^ value) & 0xff]
    return crc


def checked_status_packets(buffer, diagnostics=None):
    """Decode complete CRC-checked status packets, resynchronizing after noise.

    The caller passes the cumulative receive buffer, so skipping an incomplete
    candidate here does not lose it on the next call. A corrupted length must
    not hide a later complete packet. Valid payloads stuff the header sequence.
    """
    result = []
    offset = 0
    def count(name):
        if diagnostics is not None:
            diagnostics[name] = diagnostics.get(name, 0) + 1
    while offset + 11 <= len(buffer):
        if buffer[offset:offset+4] != b'\xff\xff\xfd\x00':
            count('discarded_bytes')
            offset += 1
            continue
        total = 7 + int.from_bytes(buffer[offset+5:offset+7], 'little')
        if total < 11:
            count('invalid_lengths')
            offset += 1
            continue
        if buffer[offset+7] != 0x55:
            count('non_status_headers')
            offset += 1
            continue
        if offset+total > len(buffer):
            count('incomplete_headers')
            offset += 1
            continue
        packet = bytes(buffer[offset:offset+total])
        if fast_crc16(packet[:-2]) != int.from_bytes(packet[-2:], 'little'):
            count('crc_errors')
            offset += 1
            continue
        body = dxl.unstuff(packet[8:-2])
        if not body:
            offset += 1
            continue
        result.append((packet[4], body[0], body[1:], packet))
        offset += total
    return result


def save_journal(path, value):
    """Commit recovery data durably before a persistent mode change."""
    temporary = path.with_suffix('.tmp')
    with temporary.open('w') as stream:
        stream.write(json.dumps(value, indent=2)+'\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    directory = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


class WireBus(dxl.Bus):
    def __init__(self, *args, live=False, **kwargs):
        self.live = live
        self.write_packets = 0
        self.read_retries = 0
        self.read_retries_by_operation = {}
        self.last_sync_diagnostic = None
        super().__init__(*args, **kwargs)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self.close()
            raise

    def send(self, packet, *args, **kwargs):
        inst = packet[7]
        if inst not in (2, 0x82, 0x8a):
            if not self.live or inst not in (3, 0x83):
                raise RuntimeError('Wire gate refused a write instruction')
            payload = dxl.unstuff(packet[8:-2])
            address = int.from_bytes(payload[:2], 'little')
            if address not in WIDTHS:
                raise RuntimeError('Register outside write whitelist')
            if inst == 3:
                if packet[4] not in IDS or len(payload)-2 != WIDTHS[address]:
                    raise RuntimeError('Invalid register write')
            else:
                width = int.from_bytes(payload[2:4], 'little')
                if width != WIDTHS[address] or len(payload[4:]) != len(IDS)*(width+1):
                    raise RuntimeError('Invalid synchronous write width')
                if set(payload[4::width+1]) != set(IDS):
                    raise RuntimeError('Synchronous write must cover exactly the 15 motors')
            self.write_packets += 1
        return self.exchange(packet, *args, **kwargs)

    def exchange(self, packet, collect_ms=None, until=None):
        # Never discard queued output. Broadcast writes have no reply; the UART
        # preserves their order before a following read without a blocking drain.
        termios.tcflush(self.fd, termios.TCIFLUSH)
        started = time.perf_counter()
        pending = memoryview(packet)
        transmit_deadline = started + self.timeout
        while pending:
            try:
                count = os.write(self.fd, pending)
            except BlockingIOError:
                count = 0
            if count:
                pending = pending[count:]
                continue
            remaining = transmit_deadline - time.perf_counter()
            if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                raise TimeoutError('UART transmit queue stalled')
        deadline = started + (self.timeout if collect_ms is None else collect_ms / 1000.)
        received = bytearray()
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                break
            if select.select([self.fd], [], [], remaining)[0]:
                try:
                    chunk = os.read(self.fd, 1024)
                except BlockingIOError:
                    continue
                if chunk:
                    received.extend(chunk)
                    if until is not None and until(bytes(received)):
                        break
        return bytes(received), time.perf_counter() - started

    def sync_read(self, ids, address, width):
        ids = list(ids)
        expected = set(ids)
        request = dxl.frame(254, 0x82, struct.pack('<HH', address, width)+bytes(ids))
        minimum = len(ids)*(11+width)
        def select_packets(packets):
            result = {}
            for sid, error, params, _ in packets:
                if sid not in expected:
                    continue
                # A genuine error wins over any otherwise usable duplicate.
                # A stale, wrong-width acknowledgement cannot hide good data.
                if error & 0x7f:
                    if sid not in result or not result[sid][0] & 0x7f:
                        result[sid] = (error, params)
                elif len(params) == width:
                    result.setdefault(sid, (error, params))
            return result

        # Byte length is only a cheap prefilter. Each expected ID must have a
        # CRC-valid reply of the requested width, or report a device fault.
        def complete(buffer):
            if len(buffer) < minimum:
                return False
            result = select_packets(checked_status_packets(buffer))
            return any(error & 0x7f for error, _ in result.values()) or expected.issubset(result)
        started = time.perf_counter()
        attempts = []
        for attempt in range(2):
            buffer, elapsed = self.send(request, until=complete)
            parser = {}
            packets = checked_status_packets(buffer, parser)
            result = select_packets(packets)
            device_fault = any(error & 0x7f for error, _ in result.values())
            valid = expected.issubset(result) and not device_fault
            unexpected = sorted({sid for sid, _, _, _ in packets} - expected)
            bad_widths = {}
            device_errors = {}
            for sid, error, params, _ in packets:
                if sid not in expected:
                    continue
                if len(params) != width:
                    bad_widths.setdefault(str(sid), []).append(len(params))
                if error & 0x7f:
                    device_errors.setdefault(str(sid), []).append(error)
            # Only retain anomalous attempts (and their recovery). This is
            # bounded in-memory evidence; there is no real-time disk logging.
            if attempts or not valid or unexpected or bad_widths or parser:
                attempts.append({'attempt': attempt+1, 'elapsed_ms': elapsed*1000.,
                                 'received_bytes': len(buffer), 'valid_ids': sorted(result),
                                 'missing_ids': sorted(expected-set(result)),
                                 'unexpected_ids': unexpected, 'bad_widths': bad_widths,
                                 'device_errors': device_errors, 'parser': parser,
                                 'wire_hex': buffer[:512].hex(),
                                 'wire_truncated': len(buffer) > 512})
            # A device-reported fault is never hidden by a retry. Only missing
            # or malformed read data gets one new, fully checked transaction.
            if device_fault or valid:
                break
            if attempt == 0:
                self.note_read_retry(f'sync:{address}')
        self.last_sync_diagnostic = ({'operation': 'sync_read', 'address': address,
                                     'width': width, 'expected_ids': ids,
                                     'complete': valid, 'attempts': attempts}
                                    if attempts else None)
        return result, time.perf_counter() - started

    def note_read_retry(self, operation):
        self.read_retries = getattr(self, 'read_retries', 0)+1
        counts = getattr(self, 'read_retries_by_operation', {})
        counts[operation] = counts.get(operation, 0)+1
        self.read_retries_by_operation = counts
        time.sleep(.001)

    def read(self, sid, address, width):
        started = time.perf_counter()
        for attempt in range(2):
            if sid == 200:
                time.sleep(IMU_QUIET_SECONDS)
            error, data, _ = super().read(sid, address, width)
            if error is not None and (error & 0x7f or len(data) == width):
                break
            if attempt == 0:
                self.note_read_retry(f'read:{sid}:{address}')
        return error, data, time.perf_counter() - started


class Hardware:
    def __init__(self, bus, profile_bytes, limits_path, *, mode='observe',
                 journal=None, clock=time.monotonic, cancelled=lambda: None, recorder=None,
                 allow_floor_start=False,policy_target_bias=None,hold_mouth_closed=False):
        if mode not in ('observe', 'shadow', 'live'):
            raise ValueError('Unknown hardware mode')
        self.bus, self.mode, self.clock = bus, mode, clock
        if type(allow_floor_start) is not bool:raise ValueError('Floor start must be explicit')
        self.allow_floor_start=allow_floor_start
        if type(hold_mouth_closed) is not bool:raise ValueError('Closed-mouth hold must be explicit')
        self.hold_mouth_closed=hold_mouth_closed
        if policy_target_bias is not None and (not isinstance(policy_target_bias,list) or len(policy_target_bias)!=15 or
                any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>math.radians(8)+1e-9 for v in policy_target_bias)):
            raise ValueError('Native policy reference requires 15 finite biases within eight degrees')
        self.policy_target_bias=policy_target_bias[:] if policy_target_bias is not None else None
        self.cancelled = cancelled
        self.recorder = recorder
        self.last_torque_readback = {}
        self.last_hardware_errors = {}
        self.last_status_time = None
        self.nominal_range_outside_ids=[]
        self.profile_sha = hashlib.sha256(profile_bytes).hexdigest()
        self.coords = Coordinates(json.loads(profile_bytes))
        self.limits = JointLimits(limits_path, profile_sha256=self.profile_sha, ids=IDS)
        self.imu = TrunkImu(mount=CONTRACT.get('robot_imu_mount_wxyz'))
        self.journal = Path(journal) if journal else None
        self.lock = threading.RLock()
        self.active = False
        self.fault = None
        self.failure_cause = None
        self.first_motor_frame_failure = None
        self.original = {}
        self.homing = {}
        self.current_limits = {}
        self.positions = {}
        self.frame = None
        self.sample_time = 0.
        self.slow_frame = None
        self.minimum_volts = self.maximum_volts = None
        self.voltage_sample_time = -1e10
        self.fast_volts_by_id = {}
        self.fast_temps_by_id = {}
        self.target_time = 0.
        self.slow_time = -1e10
        self.check_time = -1e10
        self.counter = None
        self.counter_time = self.clock()
        self.kp = 200
        self.last_verified_gain = None
        self.reads = self.target_frames = self.clamps = 0
        self.motor_sample_sequence = 0
        self.idle_target_frames = 0
        self.last_goals = None
        self.last_requested_targets = None
        self.last_commanded_targets = None
        self.prepare_pose_change_deg = None
        self.cleanup = None
        self.durations = []
        self.record('software_electrical_limits',voltage_minimum_v=None,
                    voltage_maximum_v=8.45,temperature_maximum_c=55,
                    minimum_voltage_cutoff_disabled_by_operator=True)
        self.record('mouth_hold_configuration',enabled=hold_mouth_closed,
                    closed_degrees=self.limits.bounds(34)[0],closed_readback_tolerance_deg=1.)

    def read_reg(self, sid, address, width, signed=False):
        error, data, _ = self.bus.read(sid, address, width)
        if error is None or error & 0x7f or len(data) != width:
            raise RuntimeError(f'ID {sid}: incomplete read at {address}')
        return int.from_bytes(data, 'little', signed=signed)

    def record(self,event,**values):
        if self.recorder is not None:
            try:self.recorder.record(event,**values)
            except Exception:pass

    def measured_bounds(self,sid,index):
        if sid==34 and self.hold_mouth_closed:
            return tuple(map(math.radians,self.limits.bounds(sid)))
        if self.allow_floor_start and self.policy_target_bias is not None:
            # Ground recovery follows the official actuator envelope. MJCF
            # nominal ranges have not been verified as this build's hard stops.
            bias=self.policy_target_bias[index]
            return -math.pi+bias,math.pi+bias
        return tuple(map(math.radians,self.limits.bounds(sid)))

    def measured_pose_allowed(self,sid,index,angle,*,preparing=False):
        lo,hi=self.measured_bounds(sid,index)
        slack=math.radians(.5 if not preparing or self.allow_floor_start else 0.)
        lower_slack=max(slack,math.radians(1.)) if sid==34 and self.hold_mouth_closed else slack
        return lo-lower_slack<=angle<=hi+slack

    def bulk(self, address, width):
        result, _ = self.bus.sync_read(list(IDS), address, width)
        if set(result) != set(IDS) or any(e & 0x7f or len(b) != width for e, b in result.values()):
            if address==124 and width in (12,23):
                self.record_motor_sample({sid:b for sid,(e,b) in result.items()
                                          if sid in IDS and not e & 0x7f and len(b)==width},complete=False,width=width)
            if self.first_motor_frame_failure is None:
                diagnostic = getattr(self.bus, 'last_sync_diagnostic', None)
                if not diagnostic or (diagnostic.get('address'), diagnostic.get('width')) != (address, width):
                    diagnostic = {'operation': 'sync_read', 'address': address, 'width': width,
                                  'expected_ids': list(IDS), 'received_ids': sorted(result),
                                  'missing_ids': sorted(set(IDS)-set(result)),
                                  'bad_widths': {str(s): len(b) for s, (_, b) in result.items() if len(b) != width},
                                  'device_errors': {str(s): e for s, (e, _) in result.items() if e & 0x7f}}
                self.first_motor_frame_failure = copy.deepcopy(diagnostic)
            raise RuntimeError(f'Incomplete motor frame at {address}')
        return {sid: b for sid, (_, b) in result.items()}

    def record_motor_sample(self, blocks, *, complete=True, width=23):
        # Emit BEFORE the IMU read and safety evaluation, including partial bus
        # replies. Otherwise the measurement causing an exit could be lost.
        if getattr(self,'recorder',None) is None:return
        self.motor_sample_sequence += 1
        self.record('motor_sample',sample_sequence=self.motor_sample_sequence,
                    joint_ids=list(IDS),complete=complete,active=self.active,
                    missing_ids=sorted(set(IDS)-set(blocks)),motor_register_span=[124,width],
                    voltage_measured_this_frame=width==23,
                    voltage_age_seconds=0. if width==23 else (self.clock()-self.voltage_sample_time if self.fast_volts_by_id else None),
                    servos={sid:{'voltage_v':struct.unpack_from('<H',b,20)[0]/10. if width==23 else self.fast_volts_by_id.get(sid),
                                 'voltage_raw':struct.unpack_from('<H',b,20)[0] if width==23 else None,
                                 'temperature_c':b[22] if width==23 else self.fast_temps_by_id.get(sid),'current_ma':struct.unpack_from('<h',b,2)[0],
                                 'pwm_raw':struct.unpack_from('<h',b)[0],
                                 'velocity_raw':struct.unpack_from('<i',b,4)[0],
                                 'position_raw':struct.unpack_from('<i',b,8)[0],
                                 'goal_raw':(self.last_goals or {}).get(sid),
                                 'torque':self.last_torque_readback.get(sid),
                                 'hardware_error':self.last_hardware_errors.get(sid)} for sid,b in blocks.items()},
                    status_age_seconds=None if self.last_status_time is None else self.clock()-self.last_status_time,
                    read_retries=getattr(self.bus,'read_retries',0),
                    bus_diagnostic=copy.deepcopy(getattr(self.bus,'last_sync_diagnostic',None)) if not complete else None)

    def torque(self):
        return {sid: b[0] for sid, b in self.bulk(64, 1).items()}

    def start(self):
        if any(self.torque().values()):
            raise RuntimeError('Another controller left torque enabled; start refused')
        self.homing = {sid: self.read_reg(sid, 20, 4, True) for sid in IDS}
        for sid in IDS:
            if self.read_reg(sid, 0, 2) != 1200 or self.read_reg(sid, 6, 1) != 53:
                raise RuntimeError(f'ID {sid}: unexpected servo identity/firmware')
            if self.read_reg(sid, 10, 1) != 0 or self.read_reg(sid, 11, 1) != 3:
                raise RuntimeError(f'ID {sid}: expected drive 0 / position mode 3')
            if self.read_reg(sid, 63, 1) != 52:
                raise RuntimeError(f'ID {sid}: unexpected shutdown protection')
            self.current_limits[sid] = self.read_reg(sid, 38, 2)
        # RemoteIo has no IMU readiness field: do not serve even the first frame
        # until the decoder is warm, rather than presenting synthetic gravity.
        for _ in range(30):
            self.read()
            if self.imu.samples >= 25:
                return
            time.sleep(.02)
        raise RuntimeError('IMU did not converge')

    def slow(self):
        blocks = self.bulk(144, 3)
        volts = [int.from_bytes(blocks[sid][:2], 'little')/10. for sid in IDS]
        temps = [blocks[sid][2] for sid in IDS]
        if any(v <= 0 for v in volts):
            raise RuntimeError('Invalid voltage reading')
        self.slow_frame = {'volts': sum(volts)/15, 'temps_c': temps,
                           'volts_by_id': dict(zip(IDS, volts))}
        self.minimum_volts = min(volts)
        self.maximum_volts = max(volts)
        self.slow_time = self.clock()
        return self.slow_frame

    def electrical_gate(self):
        # The operator explicitly disabled our former 6.6 V minimum cutoff.
        # Keep real voltage telemetry and the independent upper/thermal guards.
        if self.minimum_volts is None or self.maximum_volts is None or not 0 < self.minimum_volts <= self.maximum_volts:
            raise RuntimeError('Invalid voltage reading')
        if self.maximum_volts > 8.45:
            raise RuntimeError(f'Supply exceeds 8.45 V: {self.maximum_volts:.1f} V')
        if max(self.slow_frame['temps_c']) >= 55:
            raise RuntimeError('Servo temperature reached 55 C')

    def read(self):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            tick = self.clock()
            try:
                # Keep the 50 Hz position/current path short. Every 0.1 s its
                # existing request also covers voltage/temperature (124..146).
                voltage_due=tick-self.voltage_sample_time>=.1-1e-9 or tick-self.slow_time>=.5
                width=23 if voltage_due else 12
                blocks = self.bulk(124, width)
                if voltage_due:
                    self.record_motor_sample(blocks)
                    self.fast_volts_by_id={sid:struct.unpack_from('<H',blocks[sid],20)[0]/10. for sid in IDS}
                    self.fast_temps_by_id={sid:blocks[sid][22] for sid in IDS}
                    self.voltage_sample_time=tick
                volts = self.fast_volts_by_id
                temps = [self.fast_temps_by_id[sid] for sid in IDS]
                # Preserve the previous electrical guard's 0.5 s cadence.
                # The independent logger receives every fast measurement;
                # this diagnostic change does not add new shutdown decisions.
                if tick-self.slow_time >= .5:
                    if any(v <= 0 for v in volts.values()):
                        raise RuntimeError('Invalid voltage reading')
                    self.slow_frame = {'volts': sum(volts.values())/15, 'temps_c': temps,
                                       'volts_by_id': volts}
                    self.minimum_volts, self.maximum_volts = min(volts.values()), max(volts.values())
                    self.slow_time = tick
                error, imu, _ = self.bus.read(200, 124, 20)
                if error is None or error & 0x7f or len(imu) != 20:
                    raise RuntimeError('Incomplete IMU frame')
                counter, status = struct.unpack_from('<HB', imu, 12)
                if status & 3 != 3 or status & 4:
                    raise RuntimeError('IMU reports stale/unready data')
                if counter != self.counter:
                    self.counter_time = tick
                elif tick-self.counter_time > .1:
                    raise RuntimeError('IMU sample counter froze')
                self.counter = counter
                self.positions = {sid: struct.unpack_from('<i', blocks[sid], 8)[0] for sid in IDS}
                velocity = {sid: struct.unpack_from('<i', blocks[sid], 4)[0] for sid in IDS}
                angles, rates = self.coords.sensors(self.positions, velocity, self.homing)
                signed_currents = [struct.unpack_from('<h', blocks[sid], 2)[0] for sid in IDS]
                currents = [abs(v) for v in signed_currents]
                decoded = self.imu.decode(imu[:12])
                self.frame = {'positions': angles, 'velocities': rates, 'currents_ma': currents,
                              'imu': {k: decoded[k] for k in ('gyro', 'gravity', 'quat')}}
                if not all(math.isfinite(v) for v in angles+rates+currents+decoded['gyro']+decoded['gravity']):
                    raise RuntimeError('Non-finite sensor frame')
                if tick-self.check_time >= .2:
                    torque = self.torque()
                    self.last_torque_readback=torque.copy()
                    if self.mode != 'live' and any(torque.values()):
                        raise RuntimeError('Unexpected real torque in read-only mode')
                    if self.mode == 'live' and self.active and not all(torque.values()):
                        raise RuntimeError('A motor lost torque')
                    errors = self.bulk(70, 1)
                    self.last_hardware_errors={sid:b[0] for sid,b in errors.items()}
                    if any(b[0] & ~1 for b in errors.values()):
                        raise RuntimeError('Servo reports a hardware error')
                    self.check_time = tick
                    self.last_status_time = tick
                if self.active and self.mode == 'live':
                    self.electrical_gate()
                    nominal_outside=[]
                    for i, sid in enumerate(IDS):
                        lo,hi=self.measured_bounds(sid,i)
                        if not self.measured_pose_allowed(sid,i,angles[i]):
                            raise RuntimeError(f'ID {sid}: measured angle outside joint range '
                                f'({math.degrees(angles[i]):.2f} deg; nominal {math.degrees(lo):.1f}..{math.degrees(hi):.1f} deg)')
                        if currents[i] > self.current_limits[sid]:
                            raise RuntimeError(f'ID {sid}: current exceeds saved device limit')
                        nominal_lo,nominal_hi=map(math.radians,self.limits.bounds(sid))
                        if not nominal_lo-math.radians(.5)<=angles[i]<=nominal_hi+math.radians(.5):
                            nominal_outside.append(sid)
                    if nominal_outside!=self.nominal_range_outside_ids:
                        self.record('model_nominal_range_advisory',joint_ids=nominal_outside,
                                    measured=self.frame['positions'][:],floor_recovery=self.allow_floor_start,
                                    physical_calibration_verified=False)
                        self.nominal_range_outside_ids=nominal_outside
                finished = self.clock()
                if finished-tick > .1:
                    raise RuntimeError('Sensor acquisition exceeded 100 ms freshness limit')
                self.sample_time = finished
                self.reads += 1
                self.durations.append(self.sample_time-tick)
                self.record('sensor_frame',active=self.active,frame=self.frame,
                            raw_positions=self.positions.copy(),homing=self.homing.copy(),
                            imu_counter=counter,imu_status=status,imu_raw_hex=imu.hex(),
                            torque=self.last_torque_readback.copy(),hardware_errors=self.last_hardware_errors.copy(),
                            slow={k:self.slow_frame[k] for k in ('volts','temps_c')},
                            voltage_min=self.minimum_volts,voltage_max=self.maximum_volts,
                            signed_currents_ma=dict(zip(IDS,signed_currents)),
                            voltage_age_seconds=tick-self.voltage_sample_time,
                            voltage_measured_this_frame=voltage_due,
                            status_age_seconds=None if self.last_status_time is None else tick-self.last_status_time,
                            motor_register_span=[124,width],joint_ids=list(IDS),
                            acquisition_seconds=finished-tick,read_retries=getattr(self.bus,'read_retries',0))
                return self.frame
            except Exception as exc:
                self.fail(str(exc))
                raise

    def write_reg(self, sid, address, value):
        if self.mode != 'live':
            raise RuntimeError('Writes require live mode')
        width = WIDTHS[address]
        if address == 11 and self.read_reg(sid, 64, 1):
            raise RuntimeError('Cannot change operating mode with torque on')
        signed = FIELDS.get(address, (width, address == 116))[1]
        err, _, _ = self.bus.write(sid, address, int(value).to_bytes(width, 'little', signed=signed))
        if err is None or err & 0x7f:
            raise RuntimeError(f'ID {sid}: write failed at {address}')
        if address == 11:
            time.sleep(.02)
        if self.read_reg(sid, address, width, signed) != value:
            raise RuntimeError(f'ID {sid}: write readback differs at {address}')

    def sync(self, address, values):
        if self.mode != 'live' or set(values) != set(IDS):
            raise RuntimeError('Live complete motor vector required')
        width = WIDTHS[address]
        params = struct.pack('<HH', address, width)
        for sid in IDS:
            params += bytes([sid])+int(values[sid]).to_bytes(width, 'little', signed=address == 116)
        self.bus.send(dxl.frame(254, 0x83, params), collect_ms=0)

    def gain(self, kp):
        if type(kp) is not int or not 0 <= kp <= 400:
            raise ValueError('Position gain must be 0..400')
        self.kp = kp
        if self.mode == 'live' and self.original:
            self.sync(80, {sid: 0 for sid in IDS})
            self.sync(82, {sid: 0 for sid in IDS})
            self.sync(84, {sid: kp for sid in IDS})
            if self.last_verified_gain != kp:
                blocks = self.bulk(80, 6)
                measured = {sid: dict(zip(('D', 'I', 'P'), struct.unpack('<HHH', blocks[sid]))) for sid in IDS}
                matched = all(v == {'D': 0, 'I': 0, 'P': kp} for v in measured.values())
                self.record('servo_gain_readback',requested={'P':kp,'I':0,'D':0},measured=measured,
                            matched=matched,active=self.active)
                if not matched:raise RuntimeError('Servo gain readback differs from the requested P/D')
                self.last_verified_gain = kp

    def prepare(self):
        """Configure torque-off motors before starting the 200 ms RPC peer.

        Snapshot/readback takes longer than one RPC deadline. Keeping it outside
        the daemon handshake also prevents a timed-out enable from arming later.
        """
        with self.lock:
            if self.cancelled():
                raise RuntimeError(str(self.cancelled()))
            if self.fault:
                raise RuntimeError(self.fault)
            if self.mode != 'live' or self.active or self.original:
                raise RuntimeError('Prepare requires a fresh live session')
            self.read()
            self.slow()
            self.electrical_gate()
            if self.imu.samples < 25 or (not self.allow_floor_start and self.frame['imu']['gravity'][2] > -.8):
                raise RuntimeError('Hold trunk upright; IMU gravity must agree before arming')
            if any(self.torque().values()):
                raise RuntimeError('Torque must be off before preparing')
            for i,(sid, angle) in enumerate(zip(IDS, self.frame['positions'])):
                lo,hi=self.measured_bounds(sid,i)
                if not self.measured_pose_allowed(sid,i,angle,preparing=True):
                    raise RuntimeError(f'ID {sid}: current pose outside joint range')
            original_angles = self.frame['positions'][:]
            self.original = {sid: {a: self.read_reg(sid, a, *spec) for a, spec in FIELDS.items()} for sid in IDS}
            if any(v[98] != 0 for v in self.original.values()):
                self.original = {}
                raise RuntimeError('Existing bus watchdog must be cleared by its owner')
            if self.journal is None:
                self.original = {}
                raise RuntimeError('Live control requires a recovery journal')
            if self.journal.exists():
                previous = json.loads(self.journal.read_text())
                if not previous.get('cleanup', {}).get('restored'):
                    self.original = {}
                    raise RuntimeError('Previous recovery journal needs inspection before another live session')
            save_journal(self.journal, {'profile_sha256': self.profile_sha,
                'original': self.original, 'homing': self.homing, 'state': 'preparing'})
            try:
                for sid in IDS:
                    if self.cancelled():
                        raise RuntimeError(str(self.cancelled()))
                    self.write_reg(sid, 11, 4)
                    for a, value in ((80, 0), (82, 0), (84, self.kp), (88, 0), (90, 0),
                                     (100, self.original[sid][100]), (108, 0), (112, 0)):
                        if self.cancelled():
                            raise RuntimeError(str(self.cancelled()))
                        self.write_reg(sid, a, value)
                self.read()
                self.prepare_pose_change_deg=math.degrees(max(
                    abs(a-b) for a,b in zip(original_angles,self.frame['positions'])))
                # Preparation is torque-off and may take seconds. A person
                # holding the robot need not freeze every unpowered joint.
                # Revalidate the current pose; enable() seeds the measured
                # encoder positions again immediately before torque-on.
                for i,(sid,angle) in enumerate(zip(IDS,self.frame['positions'])):
                    lo,hi=self.measured_bounds(sid,i)
                    if not self.measured_pose_allowed(sid,i,angle,preparing=True):
                        raise RuntimeError(f'ID {sid}: pose moved outside joint range during preparation')
            except BaseException as exc:
                self.fail(str(exc))
                raise

    def enable(self):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if self.mode == 'observe':
                raise RuntimeError('Observe mode refuses torque-on')
            if self.active:
                return
            if self.mode == 'shadow':
                self.active = True
                self.target_time = self.clock()
                return
            if not self.original:
                raise RuntimeError('Start a prepared live session before requesting torque')
            try:
                if self.cancelled():
                    raise RuntimeError(str(self.cancelled()))
                self.read()
                self.electrical_gate()
                if not self.allow_floor_start and self.frame['imu']['gravity'][2] > -.8:
                    raise RuntimeError('Trunk must remain upright before torque-on')
                for i,(sid,angle) in enumerate(zip(IDS,self.frame['positions'])):
                    lo,hi=self.measured_bounds(sid,i)
                    if not self.measured_pose_allowed(sid,i,angle,preparing=True):
                        raise RuntimeError(f'ID {sid}: current pose outside joint range before torque-on')
                if any(self.torque().values()):
                    raise RuntimeError('Unexpected torque before enable')
                # Seed the fresh encoder branch immediately before torque.
                self.sync(116, self.positions)
                goals = self.bulk(116, 4)
                if any(abs(int.from_bytes(goals[s], 'little', signed=True)-self.positions[s]) > 2 for s in IDS):
                    raise RuntimeError('Pre-arm goal differs from present position')
                self.sync(98, {sid: 10 for sid in IDS})  # 200 ms hardware communication watchdog.
                if self.cancelled():
                    raise RuntimeError(str(self.cancelled()))
                self.sync(64, {sid: 1 for sid in IDS})
                self.active = True
                self.target_time = self.clock()
                if not all(self.torque().values()):
                    raise RuntimeError('Torque-on did not reach all motors')
                fresh = self.bulk(132, 4)
                if any(abs(int.from_bytes(fresh[s], 'little', signed=True)-self.positions[s]) > 8 for s in IDS):
                    raise RuntimeError('Encoder branch changed on torque-on')
            except BaseException as exc:
                self.fail(str(exc))
                raise

    def targets(self, values):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if self.cancelled():
                self.fail(str(self.cancelled()))
                raise RuntimeError(self.fault)
            if not isinstance(values, list) or len(values) != 15 or not all(
                    type(v) in (int, float) and math.isfinite(v) and
                    abs(v-(self.policy_target_bias[i] if self.policy_target_bias is not None else 0.)) <= math.pi+1e-8
                    for i,v in enumerate(values)):
                # Repr keeps NaN/Inf diagnostics JSON-safe for the recorder.
                self.record('target_rejected',reason='invalid_actuator_target_vector',
                            expected_count=15,actual_count=len(values) if isinstance(values,list) else None,
                            targets_repr=repr(values),
                            invalid=[{'joint_id':IDS[i] if i<len(IDS) else None,'index':i,'value_repr':repr(v),
                                      'reason':'not_finite' if type(v) not in (int,float) or not math.isfinite(v) else 'outside_actuator_range'}
                                     for i,v in enumerate(values) if type(v) not in (int,float) or not math.isfinite(v) or
                                     abs(v-(self.policy_target_bias[i] if self.policy_target_bias is not None and i<15 else 0.))>math.pi+1e-8]
                            if isinstance(values,list) else [])
                raise ValueError('Targets must contain 15 finite actuator-range radians')
            # Official robotd also publishes hold targets while Bringup::Limp.
            # Acknowledge and discard them; never cache a goal for future enable.
            if not self.active:
                self.idle_target_frames += 1
                return
            if self.clock()-self.sample_time > .1:
                raise RuntimeError('Cannot command from stale sensor coordinates')
            goals = {}
            commanded = []
            for i, sid in enumerate(IDS):
                lo, hi = map(math.radians, self.limits.bounds(sid))
                # A learned position target is also a PD effort request. Keep
                # native targets intact; clipping to anatomical travel removes
                # that effort at a stop. Actual measured travel remains guarded.
                value = values[i] if self.policy_target_bias is not None else max(lo, min(hi, values[i]))
                if sid==34 and self.hold_mouth_closed:value=lo
                commanded.append(value)
                self.clamps += value != values[i]
                current = self.frame['positions'][i]
                # Shadow is intentionally also run on arbitrary unpowered poses;
                # live must stay inside nominal range (half degree encoder slack).
                measured_lo,measured_hi=self.measured_bounds(sid,i)
                if self.mode == 'live' and not self.measured_pose_allowed(sid,i,current):
                    raise RuntimeError(f'ID {sid}: out-of-range measured pose')
                limits = ((-math.pi+self.policy_target_bias[i],math.pi+self.policy_target_bias[i])
                          if self.policy_target_bias is not None else (min(lo,current),max(hi,current)))
                goals[sid] = self.coords.raw_target(sid, value, self.positions[sid], current, limits)
            if self.mode == 'live':
                self.sync(116, goals)
            self.last_goals = goals
            self.last_requested_targets = values[:]
            self.last_commanded_targets = commanded
            self.target_frames += 1
            self.target_time = self.clock()
            self.record('actuator_targets',requested=values[:],commanded=commanded[:],raw_goals=goals.copy(),
                        native_pd_targets=self.policy_target_bias is not None,
                        measured=self.frame['positions'][:],sensor_age=self.clock()-self.sample_time,
                        clamped_ids=[sid for sid,a,b in zip(IDS,values,commanded) if a!=b])

    def stop(self):
        with self.lock:
            import traceback
            self.record('torque_off_intent',active_before=self.active,mode=self.mode,
                        has_saved_configuration=bool(self.original),failure_cause=self.failure_cause,
                        fault=self.fault,torque=self.last_torque_readback.copy(),
                        hardware_errors=self.last_hardware_errors.copy(),
                        callers=[{'file':Path(f.filename).name,'line':f.lineno,'function':f.name} for f in traceback.extract_stack(limit=7)[:-1]])
            self.active = False
            if self.mode != 'live' or not self.original:
                return
            errors = []
            off = False
            for _ in range(3):
                try:
                    self.sync(64, {sid: 0 for sid in IDS})
                    off = not any(self.torque().values())
                    if off:
                        break
                except Exception as exc:
                    errors.append(str(exc))
            if off:
                # Mode changes reset RAM gains, so restore mode first, then all
                # saved settings. Never restore a stale Goal Position.
                for sid in IDS:
                    for address in (98, 11, *(a for a in FIELDS if a not in (98, 11))):
                        try:
                            spec = FIELDS[address]
                            want = self.original[sid][address]
                            if self.read_reg(sid, address, *spec) != want:
                                self.write_reg(sid, address, want)
                        except Exception as exc:
                            errors.append(str(exc))
                try:
                    if {sid: self.read_reg(sid, 20, 4, True) for sid in IDS} != self.homing:
                        errors.append('Homing offset changed outside adapter')
                except Exception as exc:
                    errors.append(str(exc))
            else:
                errors.append('Torque-off not confirmed; original configuration retained in journal')
            self.cleanup = {'torque_off': off, 'restored': off and not errors, 'errors': errors}
            self.record('torque_off_result',cleanup=self.cleanup)
            if self.journal:
                value = json.loads(self.journal.read_text())
                value['cleanup'] = self.cleanup
                save_journal(self.journal, value)
            if self.cleanup['restored']:
                self.original = {}
            else:
                self.fault = 'Cleanup incomplete: '+ '; '.join(errors)

    def fail(self, why):
        self.record('hardware_fault',reason=why,active=self.active,
                    torque=self.last_torque_readback.copy(),hardware_errors=self.last_hardware_errors.copy(),
                    voltage_min=self.minimum_volts,voltage_max=self.maximum_volts,slow=self.slow_frame,
                    last_frame=self.frame)
        if self.failure_cause is None:
            self.failure_cause = why
        self.fault = why
        self.stop()

    def watchdog(self):
        with self.lock:
            if self.active and self.clock()-self.target_time > TIMEOUT:
                self.fail('Target stream stopped for 250 ms')

    def dispatch(self, req):
        with self.lock:
            op = req.get('op')
            if op == 'hello':
                if req.get('protocol') != 1 or req.get('joints') != 15:
                    raise ValueError('RemoteIo protocol/joint count mismatch')
                return {'protocol': 1}
            if op == 'read':
                return self.read()
            if op == 'slow':
                return self.slow()
            if op == 'write':
                self.targets(req.get('targets'))
            elif op == 'gain':
                self.gain(req.get('kp'))
            elif op == 'torque':
                if type(req.get('on')) is not bool:
                    raise ValueError('Torque field must be boolean')
                self.enable() if req['on'] else self.stop()
            else:
                raise ValueError('Unsupported RemoteIo operation')
            return {}


class Endpoint:
    """One loopback-only owner; a disconnect latches a fault while powered."""
    def __init__(self, hardware):
        self.hardware = hardware
        self.socket = socket.socket()
        self.socket.bind(('127.0.0.1', 0))
        self.socket.listen(1)
        self.socket.settimeout(.1)
        self.port = self.socket.getsockname()[1]
        self.stopping = threading.Event()
        self.connection = None
        self.worker = threading.Thread(target=self.serve, daemon=True)
        self.monitor = threading.Thread(target=self.watch, daemon=True)

    def start(self):
        self.worker.start()
        self.monitor.start()

    def watch(self):
        while not self.stopping.wait(.02):
            self.hardware.watchdog()

    def serve(self):
        while not self.stopping.is_set():
            try:
                conn, _ = self.socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.connection = conn
            conn.settimeout(.5)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                with conn, conn.makefile('rb') as stream:
                    hello = False
                    while not self.stopping.is_set():
                        raw = stream.readline(8193)
                        if not raw:
                            break
                        if len(raw) > 8192 or not raw.endswith(b'\n'):
                            raise ValueError('Oversized/partial RemoteIo frame')
                        req = json.loads(raw)
                        if not isinstance(req, dict) or (not hello and req.get('op') != 'hello'):
                            raise ValueError('Handshake required')
                        try:
                            reply = self.hardware.dispatch(req)
                            hello = True
                        except Exception as exc:
                            self.hardware.fail(str(exc))
                            reply = {'error': str(exc)}
                        conn.sendall((json.dumps(reply, allow_nan=False)+'\n').encode())
            except (OSError, ValueError):
                pass
            finally:
                if self.hardware.active:
                    self.hardware.fail('Official daemon disconnected')
                self.connection = None

    def close(self):
        self.stopping.set()
        self.socket.close()
        if self.connection:
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.hardware.stop()
        self.worker.join(timeout=2)
        self.monitor.join(timeout=2)
