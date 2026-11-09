#!/usr/bin/env python3
"""Standalone, attended bare-motor CSP round trip; requires a configured master.

Targets are radians. This diagnostic is not a watchdog or a production controller.
Use only after confirming the device scale, travel clearance and stop mechanism.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from canopen_interfaces.srv import CORead, COTargetDouble, COPositionTrajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True)
    parser.add_argument('--execution', choices=['driver', 'stream'], default='driver',
                        help='driver submits one complete timed plan; stream is a timing diagnostic')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--degrees', type=float, default=30.)
    parser.add_argument('--duration', type=float, default=3.)
    parser.add_argument('--period', type=float, default=.02,
                        help='seconds; match the configured driver/SYNC period (default: .02)')
    parser.add_argument('--profile', choices=['linear', 'quintic'], default='quintic')
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.degrees, args.duration, args.period)):
        parser.error('parameters must be finite')
    if not (0 < args.degrees <= 90 and args.duration >= 3 and .005 <= args.period <= .02):
        parser.error('requires 0 < degrees <= 90, duration >= 3, .005 <= period <= .02')
    if args.execution == 'driver' and (args.duration > 15 or args.profile != 'quintic'):
        parser.error('driver execution requires quintic profile and duration <= 15 seconds')
    rclpy.init()
    node = Node('csp_round_trip')
    base = '/' + args.device.strip('/') + '/'
    samples, calls, commands, moves = [], [], [], []
    clients = {}
    result = {'parameters': vars(args) | {'output': str(args.output)}}
    attempted = False

    def state(msg):
        if msg.position and msg.velocity:
            samples.append({'t': time.monotonic(), 'position': msg.position[0],
                            'velocity': msg.velocity[0]})

    subscription = node.create_subscription(JointState, base + 'joint_states', state, 100)

    def spin_until(deadline):
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.002, deadline - time.monotonic())
                             if deadline > time.monotonic() else 0.)

    def call(name, typ=Trigger, req=None, timeout=12.):
        if name not in clients:
            clients[name] = node.create_client(typ, base + name)
            if not clients[name].wait_for_service(timeout_sec=5.):
                raise RuntimeError('missing service: ' + name)
        started = time.monotonic()
        future = clients[name].call_async(req if req is not None else typ.Request())
        while not future.done() and time.monotonic() - started < timeout:
            rclpy.spin_once(node, timeout_sec=.001)
        if not future.done():
            clients[name].remove_pending_request(future)
            raise RuntimeError('service timeout: ' + name)
        response = future.result()
        calls.append({'name': name, 't': time.monotonic(),
                      'duration': time.monotonic() - started, 'success': response.success})
        if not response.success:
            raise RuntimeError('service rejected: ' + name + ': ' + getattr(response, 'message', ''))
        return response

    def target(value):
        call('target', COTargetDouble, COTargetDouble.Request(target=value), timeout=.1)
        commands.append({'t': time.monotonic(), 'target': value})

    def feedback():
        if not samples or time.monotonic() - samples[-1]['t'] > .1:
            raise RuntimeError('missing/stale feedback')
        return samples[-1]['position']

    try:
        discovery_deadline = time.monotonic() + 5.
        while not samples and time.monotonic() < discovery_deadline:
            spin_until(time.monotonic() + .01)
        feedback()
        attempted = True
        call('init')
        call('cyclic_position_mode')
        spin_until(time.monotonic() + .1)
        origin = feedback()
        target(origin)
        spin_until(time.monotonic() + .2)
        if args.execution == 'driver':
            session = call('position_trajectory', COPositionTrajectory)
            begin = time.monotonic()
            goal = origin + math.radians(args.degrees)
            call('position_trajectory', COPositionTrajectory,
                 COPositionTrajectory.Request(epoch=session.epoch,
                     positions=[origin, goal, origin],
                     times_from_start=[0., args.duration, 2*args.duration]))
            result['trajectory_epoch'] = session.epoch
            first = len(samples)
            # No per-point RPCs: the driver owns the accepted finite plan.
            while time.monotonic() < begin + 2*args.duration + .5:
                spin_until(time.monotonic() + .01)
                if abs(feedback() - origin) > math.radians(args.degrees + 10):
                    raise RuntimeError('travel guard')
            error = feedback() - origin
            moves.append({'begin': begin, 'end': time.monotonic(),
                          'return_error_deg': math.degrees(error),
                          'sample_start': first, 'sample_end': len(samples)})
            if abs(error) > math.radians(2):
                raise RuntimeError('return error exceeds 2 degrees')
        else:
            for direction in (1, -1):
                start = feedback()
                delta = math.radians(args.degrees) * direction
                goal = origin + math.radians(args.degrees) if direction == 1 else origin
                begin = time.monotonic()
                first = len(samples)
                tick = 0
                while True:
                    now = time.monotonic()
                    elapsed = now - begin
                    if abs(feedback() - start) > abs(delta) + math.radians(10):
                        raise RuntimeError('travel guard')
                    if now - (begin + tick * args.period) > .1:
                        raise RuntimeError('command scheduling overrun')
                    u = min(elapsed / args.duration, 1.)
                    fraction = u if args.profile == 'linear' else u**3 * (10 - 15*u + 6*u*u)
                    target(start + (goal - start) * fraction)
                    if u >= 1:
                        break
                    # Absolute deadlines: service latency consumes the cycle instead of
                    # being added to a full sleep. Skip missed ticks; never burst replay.
                    tick = max(tick + 1, math.floor((time.monotonic() - begin) / args.period) + 1)
                    spin_until(begin + tick * args.period)
                spin_until(time.monotonic() + .5)
                error = feedback() - goal
                moves.append({'begin': begin, 'end': time.monotonic(), 'delta_deg':
                              math.degrees(feedback() - start), 'error_deg': math.degrees(error),
                              'sample_start': first, 'sample_end': len(samples)})
                if abs(error) > math.radians(2):
                    raise RuntimeError('position error exceeds 2 degrees')
        result['motion_passed'] = True
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if attempted:
            for service in ('halt', 'disable'):
                try:
                    call(service)
                except Exception as exc:
                    result[service + '_error'] = str(exc)
        spin_until(time.monotonic() + .2)
        for index, name in ((0x6041, 'status'), (0x603f, 'fault'), (0x606c, 'velocity')):
            try:
                result[name] = call('sdo_read', CORead, CORead.Request(index=index, subindex=0)).data
            except Exception as exc:
                result[name + '_error'] = str(exc)
        result['passed'] = (result.get('motion_passed', False) and
                            not any(k.endswith('_error') for k in result) and
                            result.get('velocity') == 0 and result.get('fault') == 0 and
                            'status' in result and (result['status'] & 0x6f) == 0x40)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({'result': result, 'moves': moves, 'calls': calls,
                                          'commands': commands, 'samples': samples}, indent=2) + '\n')
        print(json.dumps(result, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
