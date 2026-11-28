#!/usr/bin/env python3
"""Temporary, supervised mouse hold-to-drive source for AMR Bridge V3.3."""
import argparse
import json
import re
import sys
import time
import tkinter as tk
from pathlib import Path

# Run this file from the ESP32_AMR checkout (or place it in its tools folder).
sys.path.insert(0, str(Path.cwd()))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from amr_ground_protocol import command_qos, status_qos


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True)
    parser.add_argument('--stationary-check', action='store_true', help='publish STOP only')
    args = parser.parse_args()
    if not re.fullmatch(r'[0-9a-f]{32}', args.session):
        parser.error('Expected the current 32-character bridge session')
    rclpy.init(args=[])
    node = rclpy.create_node('amr_mapping_teleop')
    pub = node.create_publisher(Twist, '/amr/teleop_cmd_vel', command_qos())
    root = None
    try:
        root = tk.Tk()
        root.title('AMR Mapping — Hold to drive')
        root.geometry('500x360')
        status = None
        status_at = 0.0
        held = None
        pressed_at = 0.0
        last_tick = time.monotonic()
        closing = False
        label = tk.StringVar(value='Waiting for Bridge V3.3')
        action = tk.StringVar(value='Stationary check — STOP only' if args.stationary_check
                              else 'STOP — hold a direction to move')

        def publish(v=0.0, w=0.0):
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
            pub.publish(msg)

        def stop(event=None):
            nonlocal held
            held = None
            action.set('STOP requested')
            if rclpy.ok():
                publish()

        def safe_status():
            if not status or time.monotonic() - status_at >= 0.20:
                return False
            remaining = status.get('session_time_remaining_s')
            return (status.get('version') == '3.3'
                    and status.get('session_id') == args.session
                    and status.get('bridge_mode') == 'ground'
                    and status.get('command_source') == 'teleop'
                    and status.get('authority_state') == 'OWNED'
                    and status.get('clock_status') == 'VERIFIED'
                    and status.get('uart_link_ready') is True
                    and status.get('fault_latched') is False
                    and status.get('firmware_fault') is False
                    and status.get('encoder_continuity_valid') is True
                    and status.get('uart_operational_errors') == 0
                    and status.get('control_faults') == 0
                    and status.get('state') in ('READY', 'STARTING', 'RUNNING')
                    and isinstance(remaining, (float, int)) and remaining > 2.0)

        def on_status(msg):
            nonlocal status, status_at
            try:
                data = json.loads(msg.data)
                if not isinstance(data, dict) or data.get('session_id') != args.session:
                    raise ValueError('unexpected session')
                status = data
                status_at = time.monotonic()
            except (ValueError, TypeError):
                status = None
                status_at = 0.0
            if held is not None and not safe_status():
                stop()

        node.create_subscription(String, '/amr_bridge/status', on_status, status_qos())

        def press(event, name, v, w):
            nonlocal held, pressed_at
            if args.stationary_check:
                return 'break'
            if held is not None:
                stop()
                return 'break'
            if (safe_status() and status.get('state') == 'READY'
                    and status.get('stationary_ready') is True
                    and status.get('stop_verified') is True
                    and status.get('firmware_armed') is False
                    and status.get('firmware_running') is False):
                held = (v, w)
                pressed_at = time.monotonic()
                action.set(name + ' — release to stop (max 8 s per hold)')
            else:
                stop()
                action.set('Not ready — release and press again when READY')
            return 'break'

        def close(event=None):
            nonlocal closing
            if closing:
                return
            closing = True
            stop()
            root.destroy()

        def tick():
            nonlocal last_tick
            if closing:
                return
            if not rclpy.ok():
                close()
                return
            now = time.monotonic()
            if now - last_tick > 0.15:
                stop()
            last_tick = now
            rclpy.spin_once(node, timeout_sec=0.0)
            if held is not None and (not safe_status() or now - pressed_at >= 8.0):
                stop()
            publish(*(held if held is not None else (0.0, 0.0)))
            if status is None or time.monotonic() - status_at >= 0.20:
                label.set('No fresh bridge status — STOP')
            else:
                remaining = status.get('session_time_remaining_s')
                seconds = f'{remaining:.1f}s' if isinstance(remaining, (int, float)) else '?'
                label.set(f"{status.get('state')} | {status.get('authority_state')} | {seconds} left")
            root.after(50, tick)

        tk.Label(root, text='AMR — Manual SLAM', font=('Sans', 18, 'bold')).pack(pady=12)
        tk.Label(root, textvariable=label).pack()
        panel = tk.Frame(root)
        panel.pack(pady=15)
        for name, v, w, row, col in (
                ('Forward', .05, 0., 0, 1), ('Left', 0., .25, 1, 0),
                ('Right', 0., -.25, 1, 2), ('Reverse', -.04, 0., 2, 1)):
            button = tk.Button(panel, text=name, width=11, height=2, takefocus=False)
            button.grid(row=row, column=col, padx=3, pady=3)
            button.bind('<ButtonPress-1>', lambda e, n=name, x=v, z=w: press(e, n, x, z))
            button.bind('<ButtonRelease-1>', stop)
            button.bind('<Leave>', stop)
        tk.Button(panel, text='STOP', width=11, height=2, bg='#b22222', fg='white',
                  command=stop, takefocus=False).grid(row=1, column=1, padx=3, pady=3)
        root.bind('<ButtonRelease-1>', stop, add='+')
        root.bind('<FocusOut>', stop, add='+')
        root.bind('<Unmap>', stop, add='+')
        root.bind('<Escape>', stop)
        root.bind('<space>', stop)
        root.protocol('WM_DELETE_WINDOW', close)
        tk.Label(root, textvariable=action, wraplength=470).pack(pady=5)
        tk.Label(root, text='No automatic obstacle avoidance. Keep physical cutoff ready.').pack()
        root.after(0, tick)
        root.mainloop()
    finally:
        if rclpy.ok():
            pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
