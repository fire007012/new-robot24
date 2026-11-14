#!/usr/bin/env python3
"""Bounded DM velocity test over raw SocketCAN, with packet evidence and cleanup."""
import argparse
import fcntl
import json
import math
from pathlib import Path
import signal
import socket
import struct
import tempfile
import time

from motor_discovery import link_state


def speed_payload(speed):
    if not math.isfinite(speed):
        raise ValueError('non-finite speed')
    return struct.pack('<f', speed)


def mit_velocity_payload(speed, ranges, kd):
    pmax, vmax, tmax = ranges
    if not math.isfinite(speed) or abs(speed) > vmax or not 0 < kd <= 5:
        raise ValueError('invalid MIT velocity/damping')
    def encode(value, low, high, bits):
        return int((value-low)*((1 << bits)-1)/(high-low))
    p = encode(0., -pmax, pmax, 16)
    v = encode(speed, -vmax, vmax, 12)
    d = encode(kd, 0., 5., 12)
    t = encode(0., -tmax, tmax, 12)
    # Kp=0, Tff=0: the encoded position cannot pull toward an absolute angle.
    return bytes((p >> 8, p & 255, v >> 4, (v & 15) << 4,
                  0, d >> 4, ((d & 15) << 4) | (t >> 8), t & 255))


def decode_status(ident, data, node, master, ranges):
    if ident != master or len(data) != 8 or data[0] & 15 != node:
        return None
    if data[:2] == struct.pack('<H', node) and data[2] in (0x33, 0x55):
        return None
    pmax, vmax, tmax = ranges
    return dict(state=data[0] >> 4,
                position=((data[1] << 8) | data[2]) * 2 * pmax / 65535 - pmax,
                velocity=((data[3] << 4) | (data[4] >> 4)) * 2 * vmax / 4095 - vmax,
                torque=(((data[4] & 15) << 8) | data[5]) * 2 * tmax / 4095 - tmax,
                mos_temp=data[6], rotor_temp=data[7])


class Session:
    def __init__(self, args, folder):
        self.args = args
        self.initial_stats = link_state(args.interface)
        self.bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, struct.pack('I', 0x1fffffff))
        self.bus.bind((args.interface,))
        self.bus.settimeout(.005)
        self.log = (folder / 'frames.jsonl').open('w', buffering=1)
        self.latest = None
        self.latest_at = 0.
        self.ranges = None
        self.samples = []
        self.started = time.monotonic()
        self.mode = 3

    def record(self, direction, ident, data):
        self.log.write(json.dumps(dict(t=time.monotonic()-self.started, direction=direction,
                                      id=hex(ident), data=data.hex()))+'\n')

    def send(self, ident, data):
        self.bus.send(struct.pack('=IB3x8s', ident, len(data), data.ljust(8, b'\0')))
        self.record('tx', ident, data)

    def velocity(self, value):
        if self.mode == 1:
            self.send(self.args.node, mit_velocity_payload(value, self.ranges, self.args.kd))
        else:
            self.send(0x200+self.args.node, speed_payload(value))

    def control(self, command):
        self.send(self.args.node if self.mode == 1 else 0x200+self.args.node,
                  bytes([255])*7+bytes([command]))

    def receive(self, strict=True):
        try:
            ident, size, payload = struct.unpack('=IB3x8s', self.bus.recv(16))
        except TimeoutError:
            return None
        data = payload[:size]
        self.record('rx', ident, data)
        if ident & 0x20000000:
            # Only the observed empty controller notice is deferred. All actual
            # ACK/protocol/warning/passive/BUS-OFF errors stop the test immediately.
            if strict and not (ident == 0x20000004 and size == 8 and data == bytes(8)):
                raise RuntimeError(f'CAN error {ident:#x}/{data.hex()}')
        if self.ranges:
            sample = decode_status(ident, data, self.args.node, self.args.master, self.ranges)
            if sample is not None:
                sample['t'] = time.monotonic()-self.started
                self.latest, self.latest_at = sample, time.monotonic()
                self.samples.append(sample)
        return ident, data

    def read(self, rid, fmt='I'):
        for _ in range(3):
            self.send(0x7ff, struct.pack('<HBBI', self.args.node, 0x33, rid, 0))
            deadline = time.monotonic()+.5
            while time.monotonic() < deadline:
                row = self.receive()
                if row and row[0] == self.args.master and len(row[1]) == 8 and row[1][:4] == struct.pack('<HBB',self.args.node,0x33,rid):
                    return struct.unpack('<'+fmt,row[1][4:])[0]
        raise RuntimeError(f'no register {rid} reply')

    def write_mode(self, mode):
        self.send(0x7ff, struct.pack('<HBBI', self.args.node, 0x55, 10, mode))
        deadline = time.monotonic()+.5
        while time.monotonic() < deadline:
            row = self.receive(strict=False)
            if row and row[0] == self.args.master and row[1] == struct.pack('<HBBI',self.args.node,0x55,10,mode):
                if self.read(10) != mode:
                    raise RuntimeError('mode readback mismatch')
                return
        raise RuntimeError('mode write ACK timeout')

    def check(self, enabled=False):
        if self.latest is None or time.monotonic()-self.latest_at > .15:
            raise RuntimeError('status feedback stale')
        if self.latest['state'] not in (0,1):
            raise RuntimeError('motor fault: '+str(self.latest))
        if enabled and self.latest['state'] != 1:
            raise RuntimeError('motor disabled unexpectedly')
        if abs(self.latest['velocity']) > 2.:
            raise RuntimeError('velocity limit exceeded')
        if enabled and abs(self.latest['torque']) > self.args.torque_limit:
            raise RuntimeError('torque limit exceeded: '+str(self.latest))

    def settle(self, seconds, enabled=False):
        deadline=time.monotonic()+seconds
        next_send=0.
        while time.monotonic()<deadline:
            if time.monotonic()>=next_send:
                self.velocity(0.)
                next_send=time.monotonic()+.02
            self.receive()
            if self.latest is not None:
                self.check(enabled)

    def stop(self):
        requested=time.monotonic()
        deadline=requested+2.
        next_send=0.
        while time.monotonic()<deadline:
            if time.monotonic()>=next_send:
                try:
                    self.velocity(0.)
                    self.control(0xfd)
                except OSError as error:
                    print('stop send:',error,flush=True)
                next_send=time.monotonic()+.1
            self.receive(strict=False)
            if self.latest_at>=requested and self.latest and self.latest['state']==0 and abs(self.latest['velocity'])<.05:
                print('STOPPED: fresh disabled and zero-speed feedback',self.latest,flush=True)
                return True
        print('STOP NOT CONFIRMED:',self.latest,flush=True)
        return False


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface',default='can0')
    parser.add_argument('--node',type=int,required=True)
    parser.add_argument('--master',type=int,required=True)
    parser.add_argument('--speed',type=float,required=True)
    parser.add_argument('--duration',type=float,default=.6)
    parser.add_argument('--ramp',type=float,default=.3)
    parser.add_argument('--torque-limit',type=float,default=15.)
    parser.add_argument('--control',choices=('native','mit'),default='native')
    parser.add_argument('--kd',type=float,default=4.)
    args=parser.parse_args()
    if not 0<=args.node<=15 or not 0<=args.master<=2047:
        parser.error('invalid node/master')
    if not all(math.isfinite(v) for v in (args.speed,args.duration,args.ramp,args.torque_limit,args.kd)) or not 0<abs(args.speed)<=.5 or not 0<args.ramp<=args.duration<=2 or not 0<args.torque_limit<=20 or not 0<args.kd<=5:
        parser.error('speed <= .5, 0 < ramp <= duration <= 2, torque limit <= 20 required')
    lock=open('/tmp/robot24-base-keyboard.lock','w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    folder=Path(tempfile.mkdtemp(prefix='dm-raw-diagnostic-'))
    print('Evidence:',folder,flush=True)
    session=Session(args,folder)
    attempted=False
    passed=False
    failure=None
    mode_changed=False
    def interrupt(*_):raise KeyboardInterrupt
    old={sig:signal.signal(sig,interrupt) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        registers={rid:session.read(rid,'f' if rid in (21,22,23,80) else 'I') for rid in (7,8,10,21,22,23,80)}
        (folder/'registers-before.json').write_text(json.dumps(registers,indent=2))
        print('Registers:',registers,flush=True)
        if (registers[7],registers[8],registers[10])!=(args.master,args.node,3):
            raise RuntimeError('identity or velocity mode not confirmed; no parameters changed')
        session.ranges=[registers[rid] for rid in (21,22,23)]
        if not all(math.isfinite(v) and v>0 for v in session.ranges):raise RuntimeError('invalid feedback ranges')
        attempted=True
        session.settle(.15)
        session.check()
        if session.latest['state']!=0 or abs(session.latest['velocity'])>.05:raise RuntimeError('not initially disabled/stopped')
        if args.control == 'mit':
            mode_changed=True
            session.mode=1
            session.write_mode(1)
            session.latest=None
            session.settle(.1)
            session.check()
        before=link_state(args.interface)
        if before!=session.initial_stats:raise RuntimeError('CAN counters changed before enable')
        session.control(0xfc)
        deadline=time.monotonic()+.15
        while time.monotonic()<deadline:
            session.velocity(0.)
            session.receive()
            if session.latest and session.latest['state']==1:break
        session.check(True)
        origin=session.latest['position']
        start=time.monotonic();next_send=start;next_print=start
        print('MOTION: bounded ramp start',flush=True)
        while time.monotonic()-start<args.duration:
            now=time.monotonic()
            session.check(True)
            if now>=next_send:
                target=args.speed*min(1.,(now-start)/args.ramp)
                session.velocity(target);next_send=now+.02
            session.receive()
            session.check(True)
            if now>=next_print:
                print('sample:',session.latest,flush=True);next_print=now+.1
        session.velocity(0.)
        moved=abs(session.latest['position']-origin)
        peak=max(abs(s['velocity']) for s in session.samples)
        print('RESULT: displacement=',moved,'peak velocity=',peak,flush=True)
        passed=moved>.005 and peak>abs(args.speed)*.4
        if not passed:failure='no confirmed rotation'
    except BaseException as error:
        failure=repr(error)
        print('ABORT:',failure,flush=True)
    finally:
        for sig in old:signal.signal(sig,signal.SIG_IGN)
        stopped=session.stop() if attempted else None
        mode_restored=not mode_changed
        if mode_changed and stopped:
            try:
                session.write_mode(3)
                session.mode=3
                mode_restored=True
                print('RESTORED: CTRL_MODE=3 confirmed; no Flash write',flush=True)
            except BaseException as error:
                print('MODE RESTORE FAILED:',repr(error),flush=True)
                failure=failure or repr(error)
        summary=dict(arguments=vars(args),passed=passed,failure=failure,stopped=stopped,
                     mode_restored=mode_restored,samples=session.samples)
        (folder/'result.json').write_text(json.dumps(summary,indent=2))
        session.bus.close();session.log.close()
        for sig,handler in old.items():signal.signal(sig,handler)
    return 0 if passed and stopped and mode_restored else 1


if __name__=='__main__':raise SystemExit(main())
