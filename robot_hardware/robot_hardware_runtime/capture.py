"""Linux SocketCAN evidence decoding, independent of ROS and device writes."""
import socket
import struct
import time

# Linux UAPI asm-generic/socket.h; Python on Humble omits these names.
SO_TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)
SO_RXQ_OVFL = getattr(socket, 'SO_RXQ_OVFL', 40)
TIMESPEC = struct.Struct('@ll')
FRAME = struct.Struct('=IB3x8s')
ANCILLARY_SIZE = socket.CMSG_SPACE(TIMESPEC.size) + socket.CMSG_SPACE(4)


def open_channel(interface):
    channel = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        channel.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
        channel.setsockopt(socket.SOL_SOCKET, SO_RXQ_OVFL, 1)
        channel.bind((interface,))
        channel.setblocking(False)
    except BaseException:
        channel.close()
        raise
    return channel


def receive_batch(channel, interface, limit=1024):
    """Drain queued frames fairly; SYNC timing uses kernel receive timestamps.

    t remains the userspace monotonic read time for compatibility. kernel_time_ns
    is CLOCK_REALTIME, so retain both clocks rather than silently mixing them.
    """
    records = []
    for _ in range(limit):
        try:
            raw, ancillary, flags, _address = channel.recvmsg(FRAME.size, ANCILLARY_SIZE)
        except BlockingIOError:
            break
        if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC) or len(raw) != FRAME.size:
            raise ValueError('truncated CAN evidence frame or receive metadata')
        identifier, length, payload = FRAME.unpack(raw)
        if length > 8:
            raise ValueError('expected a classic CAN frame with DLC <= 8')
        timestamp = None
        dropped = None
        for level, kind, value in ancillary:
            if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS:
                seconds, nanos = TIMESPEC.unpack(value[:TIMESPEC.size])
                timestamp = seconds * 1_000_000_000 + nanos
            elif level == socket.SOL_SOCKET and kind == SO_RXQ_OVFL:
                dropped, = struct.unpack('=I', value[:4])
        if timestamp is None:
            raise ValueError('kernel receive timestamp missing; SYNC timing unavailable')
        records.append({'t': time.monotonic(), 'kernel_time_ns': timestamp,
                        'interface': interface, 'id': identifier, 'dlc': length,
                        'data': payload[:length].hex(), 'rx_dropped_total': dropped})
    return records


class CanStatistics:
    def __init__(self, interfaces):
        self.frames = dict.fromkeys(interfaces, 0)
        self.bits = dict.fromkeys(interfaces, 0)
        self.dropped = dict.fromkeys(interfaces, 0)
        self.intervals = {name: [] for name in interfaces}
        self.last_sync = {}
        self.clock_regressions = dict.fromkeys(interfaces, 0)

    def add(self, record):
        interface, identifier = record['interface'], record['id']
        self.frames[interface] += 1
        # Approximate classic CAN load, including extended/RTR frame overhead.
        extended = bool(identifier & socket.CAN_EFF_FLAG)
        remote = bool(identifier & socket.CAN_RTR_FLAG)
        if not identifier & socket.CAN_ERR_FLAG:
            self.bits[interface] += (47 + 20 * extended +
                                     (0 if remote else 8 * record['dlc'])) * 1.25 + 3
        if record['rx_dropped_total'] is not None:
            self.dropped[interface] = record['rx_dropped_total']
        # A SYNC is a standard data frame, not an RTR/extended/error alias.
        if identifier == 0x80:
            stamp = record['kernel_time_ns']
            if interface in self.last_sync:
                delta = stamp - self.last_sync[interface]
                if delta < 0:
                    self.clock_regressions[interface] += 1
                else:
                    self.intervals[interface].append(delta / 1_000_000_000)
            self.last_sync[interface] = stamp

    def report(self, duration, bitrate):
        return {'frames': self.frames,
                'estimated_bus_load': {i: b / (duration * bitrate) for i, b in self.bits.items()},
                'sync_intervals_s': self.intervals,
                'rx_dropped_total': self.dropped,
                'timestamp_clock_regressions': self.clock_regressions,
                'can_timestamp_clock': 'CLOCK_REALTIME (kernel SO_TIMESTAMPNS)',
                't_clock': 'CLOCK_MONOTONIC (userspace read)',
                'drop_counter_scope': 'kernel socket queue; last delivered counter, not physical bus loss'}
