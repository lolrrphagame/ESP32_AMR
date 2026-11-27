"""Session-bound Nav2 ingress on a trusted Pi/laptop LAN."""
import hashlib
import hmac
import json
import math
import os
import secrets
import socket

from amr_ground_protocol import (AuthorityError, CLOCK_OFFSET_NS, CLOCK_PROOF_S, DDSStamp, DEADMAN_NS,
                                 clock_proven)
from amr_ros2_bridge_v3 import DEADMAN_S, wheel_targets


class PeerSocket:
    def __init__(self, session, host, *, pi):
        self.key = bytes.fromhex(session)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 46011 if pi else 46010))
        self.sock.connect((host, 46010 if pi else 46011))
        self.sock.setblocking(False)

    def send(self, data):
        payload = json.dumps(data, allow_nan=False, separators=(',', ':')).encode()
        try:
            self.sock.send(hmac.digest(self.key, payload, hashlib.sha256) + payload)
        except (BlockingIOError, ConnectionRefusedError):
            pass  # Drop this datagram. Motion is never retried.

    def receive(self):
        for _ in range(8):
            try:
                packet = self.sock.recv(2048)
            except (BlockingIOError, ConnectionRefusedError):
                return
            mac, payload = packet[:32], packet[32:]
            if hmac.compare_digest(mac, hmac.digest(self.key, payload, hashlib.sha256)):
                yield json.loads(payload)


class PiIngress:
    def __init__(self, link, adapter):
        self.link, self.adapter = link, adapter
        self.pending = {}
        self.next_probe = 0.
        self.owner = None
        self.gid = None
        self.proof_at = None

    def poll(self, on_command):
        now = self.adapter.now()
        if self.proof_at is not None and now - self.proof_at >= CLOCK_PROOF_S:
            raise AuthorityError('remote Nav2 clock proof expired')
        if now >= self.next_probe:
            self.next_probe = now + .05
            self.pending = {key: v for key, v in self.pending.items() if now - v[1] < DEADMAN_S}
            nonce = secrets.token_hex(12)
            self.pending[nonce] = self.adapter.clock.check(), now
            self.link.send([nonce])
        for reply in self.link.receive():
            self.accept(reply, on_command)

    def accept(self, reply, on_command):
        kind, nonce, instance, gid, t2, t3, *body = reply
        probe = self.pending.get(nonce)
        if probe is None:
            return
        t1, issued_at = probe
        wall = self.adapter.clock.check()
        if kind == 'fault':
            reason, = body
            raise AuthorityError('laptop ingress fault: ' + str(reason)[:120])
        if self.owner is not None and self.owner != (instance, gid):
            raise AuthorityError('remote Nav2 owner changed')
        if (self.adapter.now() - issued_at >= DEADMAN_S or
                not clock_proven(t1, t2, t3, wall)):
            if self.adapter.metrics:
                self.adapter.metrics.event('remote_clock_rejected')
            return  # No proof, no forwarding or freshness update.
        if not isinstance(gid, str) or len(bytes.fromhex(gid)) != 24:
            raise AuthorityError('invalid remote DDS writer identity')
        self.owner = instance, gid
        self.gid = tuple(bytes.fromhex(gid))
        self.proof_at = max(issued_at, self.proof_at or issued_at)
        if self.adapter.metrics:
            self.adapter.metrics.latest.update(
                remote_nav2_gid=gid, remote_clock_bounds_ms=[(t3 - wall) / 1e6, (t2 - t1) / 1e6])
        if kind == 'ready':
            return
        if kind != 'cmd':
            raise AuthorityError('invalid remote message')
        source, receipt, values, health_until = body
        if type(source) is int and self.adapter.last_source is not None and source <= self.adapter.last_source:
            return
        if (type(source) is not int or type(receipt) is not int
                or not 0 < source <= receipt <= t3
                or not 0 <= wall - source + CLOCK_OFFSET_NS < DEADMAN_NS):
            raise AuthorityError('stale remote Nav2 command')
        if len(values) != 6 or not all(type(v) in (float, int) and math.isfinite(v) for v in values):
            raise AuthorityError('invalid remote Twist')
        wheel_targets(values)
        if any(values) and (type(health_until) is not int or wall + CLOCK_OFFSET_NS >= health_until):
            raise AuthorityError('stale scan, odometry or localization')
        on_command(values, DDSStamp(source, receipt), health_until)


def public_domain():
    value = os.environ.get('AMR_NAV2_PUBLIC_DOMAIN')
    if value is None:
        return None
    if (os.environ.get('ROS_LOCALHOST_ONLY') != '1'
            or int(value) == int(os.environ.get('ROS_DOMAIN_ID', '0'))
            or not 0 <= int(value) <= 232):
        raise AuthorityError('select distinct public and Pi localhost ROS domains')
    return int(value)
