#!/usr/bin/env python3
"""Observe feedback on the public ROS domain with motor power isolated."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.amr_nav2_source import FeedbackWindow
from amr_ground_protocol import CLOCK_OFFSET_NS, WallClockGuard, metadata_executor


def run(seconds, tolerance):
    import rclpy
    rclpy.init(args=[])
    node = rclpy.create_node('amr_feedback_preflight')
    executor = metadata_executor()
    executor.add_node(node)
    health = FeedbackWindow(tolerance)
    clock = WallClockGuard()
    failed = set()
    ages = {k: [] for k in health.stamps}
    min_margin = {k: math.inf for k in health.stamps}
    previous = {}
    scan_timing = {}
    start = time.monotonic()
    measuring = False

    def inspect(wall):
        # Also test previously observed state immediately before replacing it.
        for key in health.failures(wall):
            if key not in failed:
                print('FAIL', health.describe(key, wall), file=sys.stderr, flush=True)
            failed.add(key)
        for key in health.stamps:
            min_margin[key] = min(min_margin[key], (health.deadline(key)-wall)/1e6)

    def register(msg_type, topic, handler, qos):
        key = {'/scan': 'scan', '/tf': 'localization', '/odom': 'odom'}[topic]
        def observe(msg, info):
            wall = clock.check()
            if measuring:
                inspect(wall)
            handler(msg)
            if topic == '/tf':
                headers = [tf.header for tf in msg.transforms
                           if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom']
                if not headers:
                    return
                header = headers[-1]
            else:
                header = msg.header
            pub = info['source_timestamp']
            recv = info['received_timestamp']
            row = dict(sensor=key, header_ns=header.stamp.sec * 10**9 + header.stamp.nanosec,
                       measurement_ns=health.stamps[key], dds_write_ns=pub, dds_receive_ns=recv,
                       callback_ns=wall, callback_delay_ms=(wall-recv)/1e6,
                       delivery_ms=(recv-pub)/1e6, update_interval_ms=health.interval_ms.get(key))
            if key in previous:
                row['write_interval_ms'] = (pub - previous[key])/1e6
            previous[key] = pub
            if topic == '/scan':
                row.update(scan_time_ms=msg.scan_time*1000, ray_span_ms=msg.time_increment*1000*(len(msg.ranges)-1))
                if measuring:
                    row['header_to_write_ms'] = (pub - row['header_ns']) / 1e6
                    for field in ('scan_time_ms', 'ray_span_ms', 'header_to_write_ms', 'write_interval_ms',
                                  'delivery_ms', 'callback_delay_ms'):
                        if field in row:
                            scan_timing.setdefault(field, []).append(row[field])
                    if (not all(math.isfinite(row[k]) and row[k] > 0 for k in ('scan_time_ms', 'ray_span_ms'))
                            or pub <= 0 or recv <= 0):
                        if 'scan timing metadata' not in failed:
                            print('FAIL scan timing metadata unavailable', file=sys.stderr)
                        failed.add('scan timing metadata')
            print(json.dumps(row))
        executor.subscribe(node, msg_type, topic, observe, qos)

    health.subscribe(node, register)
    try:
        next_sample = start
        while rclpy.ok():
            executor.spin_once(timeout_sec=.01)
            now = time.monotonic()
            wall = clock.check()
            if not measuring:
                if not health.failures(wall):
                    measuring = True
                    start = next_sample = now
                elif now - start >= 10:
                    inspect(wall)
                    break
                else:
                    continue
            inspect(wall)
            if now >= next_sample:
                for key, stamp in health.stamps.items():
                    if stamp:
                        ages[key].append((wall-stamp)/1e6)
                next_sample = now + .02
            if now - start >= seconds:
                break
        for key, values in ages.items():
            values.sort()
            if values:
                print(f'{key}: p95_age_ms={values[math.ceil(.95*len(values))-1]:.2f} '
                      f'max_age_ms={values[-1]:.2f} min_source_margin_ms={min_margin[key]:.2f}', file=sys.stderr)
                if min_margin[key] <= 2 * CLOCK_OFFSET_NS / 1e6:
                    print(f'{key}: insufficient margin for worst-case Pi clock lead and skew allowance',
                          file=sys.stderr)
        for field, values in scan_timing.items():
            values.sort()
            print(f'scan {field}: min={values[0]:.2f} p95={values[math.ceil(.95*len(values))-1]:.2f} '
                  f'max={values[-1]:.2f}', file=sys.stderr)
        passed = measuring and time.monotonic() - start >= seconds and not failed
        print('PASS feedback window' if passed else 'FAIL feedback window', file=sys.stderr)
        if not passed:
            return 1
        if min(min_margin.values()) <= 2 * CLOCK_OFFSET_NS / 1e6:
            print('HOLD: insufficient downstream clock margin', file=sys.stderr)
            return 2
        return 0
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=120.)
    parser.add_argument('--amcl-tolerance', type=float, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0 or not math.isfinite(args.amcl_tolerance) or args.amcl_tolerance < 0:
        parser.error('invalid interval or AMCL tolerance')
    sys.exit(run(args.seconds, args.amcl_tolerance))
