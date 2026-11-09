#!/usr/bin/env python3
"""Read SocketCAN link statistics without sending frames or changing the interface."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import time


def counters(snapshot):
    result = {}

    def visit(prefix, value):
        if isinstance(value, dict):
            for key, child in value.items():
                visit(f'{prefix}.{key}', child)
        elif type(value) is int:
            result[prefix] = value

    for field in ('stats64', 'stats', 'xstats'):
        visit(field, snapshot.get(field, {}))
    visit('linkinfo.info_xstats', snapshot.get('linkinfo', {}).get('info_xstats', {}))
    return result


def compare(first, last):
    # Replug/recreation or counter reset makes an interval delta ambiguous.
    if first['ifindex'] != last['ifindex']:
        return {'comparable': False, 'reason': 'interface recreated', 'delta': {}}
    start, end = counters(first), counters(last)
    if start.keys() != end.keys():
        return {'comparable': False, 'reason': 'counter fields changed', 'delta': {}}
    delta = {key: end[key] - start[key] for key in start}
    if any(value < 0 for value in delta.values()):
        return {'comparable': False, 'reason': 'counter reset detected', 'delta': {}}
    return {'comparable': bool(delta), 'reason': 'counters read' if delta else 'no counters exposed',
            'delta': delta}


def sample(interface):
    result = subprocess.run(['ip', '-json', '-details', '-statistics', 'link', 'show',
                             'dev', interface], check=True, capture_output=True, text=True,
                            timeout=5)
    links = json.loads(result.stdout)
    if len(links) != 1 or links[0].get('linkinfo', {}).get('info_kind') != 'can':
        raise ValueError('expected exactly one CAN interface')
    return {'utc': datetime.now(timezone.utc).isoformat(),
            'monotonic_ns': time.monotonic_ns(), 'link': links[0]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('interface')
    parser.add_argument('--interval', type=float, default=1.0, help='seconds between samples')
    parser.add_argument('--samples', type=int, default=2)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.interval) or args.interval <= 0 or args.samples < 2:
        parser.error('interval must be positive and finite; samples must be at least 2')
    samples, intervals = [], []
    error = None
    try:
        for index in range(args.samples):
            current = sample(args.interface)
            if samples:
                intervals.append(compare(samples[-1]['link'], current['link']))
            samples.append(current)
            if index + 1 < args.samples:
                time.sleep(args.interval)
    except KeyboardInterrupt:
        error = 'interrupted'
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        error = str(exc)
    report = {'interface': args.interface, 'samples': samples, 'intervals': intervals,
              'error': error,
              'scope': 'kernel link statistics only; no motion or stopping-time measurement'}
    content = json.dumps(report, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        args.output.write_text(content)
    else:
        print(content, end='')
    return 1 if error or len(samples) < 2 else 0


if __name__ == '__main__':
    raise SystemExit(main())
