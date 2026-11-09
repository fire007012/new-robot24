#!/usr/bin/env python3
"""Read-only DM register probe. Host interface changes require root; never enables motors."""
import argparse
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import time


def link(interface):
    return json.loads(subprocess.check_output(
        ['ip', '-details', '-statistics', '-json', 'link', 'show', interface]))[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='can0')
    parser.add_argument('--output', default='/tmp/dm-baud-probe.json')
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error('Run with sudo: changing host CAN bit timing requires root.')
    original = link(args.interface)
    original_rate = original['linkinfo']['info_data'].get('bittiming', {}).get('bitrate', 1000000)
    original_up = 'UP' in original.get('flags', [])
    original_queue = original.get('txqlen', 1000)
    result = {'interface': args.interface, 'original': original, 'attempts': [], 'found': []}
    frame_format = '=IB3x8s'
    matched_rate = None

    def configure(rate, up=True):
        for command in [
            ['ip', 'link', 'set', args.interface, 'down'],
            ['ip', 'link', 'set', args.interface, 'type', 'can', 'bitrate', str(rate)],
            ['ip', 'link', 'set', args.interface, 'txqueuelen', str(original_queue)],
        ]:
            subprocess.run(command, check=True)
        if up:
            subprocess.run(['ip', 'link', 'set', args.interface, 'up'], check=True)

    try:
        for rate in [1000000, 500000, 250000, 200000, 125000]:
            configure(rate)
            attempt = {'bitrate': rate, 'before': link(args.interface), 'frames': [], 'sent_nodes': []}
            result['attempts'].append(attempt)
            print(f'Querying at {rate} bit/s...', flush=True)
            with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
                bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, struct.pack('I', 0x1fffffff))
                bus.bind((args.interface,))
                bus.settimeout(.02)
                link_errors = False
                # Node range matches the feedback low nibble supported by the driver.
                for node in [7, 5, 1, 0, 2, 3, 4, 6, 8, 9, 10, 11, 12, 13, 14, 15]:
                    if link(args.interface)['linkinfo']['info_data'].get('state') == 'BUS-OFF':
                        attempt['stopped'] = 'BUS-OFF; no further frames sent'
                        break
                    data = struct.pack('<HBBI', node, 0x33, 10, 0)
                    try:
                        bus.send(struct.pack(frame_format, 0x7ff, 8, data))
                    except OSError as error:
                        attempt['send_error'] = str(error)
                        break
                    attempt['sent_nodes'].append(node)
                    end = time.monotonic() + .18
                    while time.monotonic() < end:
                        try:
                            raw = bus.recv(16)
                        except TimeoutError:
                            continue
                        ident, length, payload = struct.unpack(frame_format, raw)
                        payload = payload[:length]
                        if len(attempt['frames']) < 100:
                            attempt['frames'].append({'id': hex(ident), 'data': payload.hex()})
                        if ident & 0x20000000:
                            link_errors = True
                        elif not ident & 0xc0000000 and length == 8 and payload[2] == 0x33 and payload[3] == 10:
                            returned_node = struct.unpack('<H', payload[:2])[0]
                            mode = struct.unpack('<I', payload[4:])[0]
                            if returned_node == node and mode in [1, 2, 3, 4]:
                                match = {'bitrate': rate, 'node': node, 'master_id': ident, 'mode': mode}
                                result['found'].append(match)
                                matched_rate = rate
                                print('DM reply:', json.dumps(match), flush=True)
                    if matched_rate is not None or link_errors:
                        break
            attempt['after'] = link(args.interface)
            if matched_rate is not None:
                break
    except Exception as error:
        result['error'] = str(error)
    finally:
        # With no match restore the original host bit timing; never touch motor settings.
        try:
            configure(matched_rate or original_rate, original_up or matched_rate is not None)
        except Exception as error:
            result['restore_error'] = str(error)
        result['final'] = link(args.interface)
        Path(args.output).write_text(json.dumps(result, indent=2))
        print('Saved:', args.output, flush=True)
        if matched_rate is None:
            print('No validated DM reply. No enable or motion frames were sent.', flush=True)
    return 0 if matched_rate is not None else 1


if __name__ == '__main__':
    raise SystemExit(main())
