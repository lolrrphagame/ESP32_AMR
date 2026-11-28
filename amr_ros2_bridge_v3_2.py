#!/usr/bin/env python3
"""Guarded AMR ground and raised-wheel commissioning bridge for ROS 2 Humble."""
import argparse
from dataclasses import dataclass
import json
import math
import os
import secrets
import sys
import time

from amr_ground_protocol import (
    CLOCK_OFFSET_NS, CLOCK_PROOF_S, CONTROL_TOPIC, DEADMAN_NS, DISCOVERY_S, GROUND_TOPIC,
    NS, SOURCE_TOPICS, STATUS_TOPIC, AuthorityError, CommandStamp, DDSStamp, WallClockGuard,
    arrival_monotonic, check_dds_publisher, clock_proven, command_qos, inspect_publishers,
    metadata_executor, status_qos, telemetry_qos, token_valid,
)
from amr_joint_telemetry import JointTelemetry
from amr_link_metrics import LinkMetrics, PeriodicLinkReport, summarize
from amr_ros2_bridge_v3 import (
    CPR, DEADMAN_S, LOOP_PERIOD_S, MAX_ANGULAR_RPS, MAX_LINEAR_MPS, MAX_WHEEL_RPM,
    MIN_ACTIVE_RPM, SESSION_MAX_S, SESSION_STOP_MARGIN_S, WHEEL_SEPARATION_M,
    ZERO, ProtocolError, State, directions, wheel_targets,
)
from amr_ros2_bridge_v3_1 import (
    MPS_PER_RPM, SYNC_MARGIN_RPM, BridgeV31, Mode, Transition, exclusive_uart,
    twist_components,
)

ACTIVE = (Transition.STARTING, State.RUNNING)
REENTRY_RPM = 5.5
REENTRY_S = 0.12
GROUND_DEFAULT_S = 10.0
GROUND_MAX_S = 120.0
MIN_START_LINEAR = MIN_ACTIVE_RPM * MPS_PER_RPM
MIN_START_ANGULAR = 2 * MIN_START_LINEAR / WHEEL_SEPARATION_M


@dataclass(frozen=True)
class Limits:
    linear: float = MAX_LINEAR_MPS
    angular: float = MAX_ANGULAR_RPS
    wheel: float = MAX_WHEEL_RPM

    def __post_init__(self):
        if not (MIN_START_LINEAR <= self.linear <= MAX_LINEAR_MPS
                and MIN_START_ANGULAR <= self.angular <= MAX_ANGULAR_RPS
                and MIN_ACTIVE_RPM <= self.wheel <= MAX_WHEEL_RPM):
            raise ValueError('commissioning limits must contain the 5 RPM startup pair and obey phase maxima')

    def targets(self, components):
        requested, _ = wheel_targets(components)
        vx, wz = components[0], components[5]
        # Reserve trim even for asymmetric forward pairs. Interpolated setpoints
        # can cross the firmware sync band while slewing between those pairs.
        margin = SYNC_MARGIN_RPM if min(requested) > 0 and self.wheel >= 6.99999 else 0.0
        roundoff = 1 - 1e-7
        linear_cap = max(MIN_START_LINEAR, self.linear * roundoff)
        angular_cap = max(MIN_START_ANGULAR, self.angular * roundoff)
        ratio = max(1.0, abs(vx) / linear_cap, abs(wz) / angular_cap,
                    max(map(abs, requested)) / (self.wheel - margin))
        return requested, tuple(rpm / ratio for rpm in requested), (vx / ratio, wz / ratio)


class BridgeV32(BridgeV31):
    def __init__(self, uart, commissioning=None, source=None, duration=None, limits=Limits(), *, metrics=None, **kwargs):
        if commissioning not in (None, 'raised', 'ground') or (
                commissioning is None and (source is not None or duration is not None)):
            raise ValueError('a source and duration can only be selected for commissioning')
        if commissioning is not None and source not in SOURCE_TOPICS:
            raise ValueError('select one authorized command source')
        if commissioning == 'raised' and duration is not None:
            raise ValueError('raised commissioning uses the fixed 15-second bench budget')
        if duration is not None and not 1.0 <= duration <= GROUND_MAX_S:
            raise ValueError('ground duration must be between 1 and 120 seconds')
        super().__init__(uart, Mode.CONTINUOUS if commissioning else Mode.READ_ONLY, **kwargs)
        self.commissioning, self.source = commissioning, source
        self.duration = SESSION_MAX_S if commissioning == 'raised' else (
            (duration if duration is not None else GROUND_DEFAULT_S) if commissioning else None)
        self.limits = limits
        self.metrics = metrics
        self.session = secrets.token_hex(16) if commissioning else None
        self.clock = WallClockGuard(self.now, self.wall_ns)
        self.challenges = {}
        self.candidate_endpoints = None
        self.discovery_at = None
        self.owner_endpoints = None
        self.activation = None
        self.clock_ok_at = None
        self.clock_status = 'WAITING' if commissioning else None
        self.authorized_after = 0
        self.last_stamp = None
        self.requested_body = self.limited_body = None
        self.stop_reason = None
        self.low_inhibited = False
        self.zero_after_inhibit = False
        self.reentry = None
        self.encoder_boot = None
        self.encoder_valid = False
        self.encoder_invalid_reason = None
        self.previous_encoder = None
        self.feedback_stamp_ns = None
        self.pi_nav2 = False

    def start(self):
        super().start()
        if self.commissioning and self.state != State.FAULT_LATCHED and self.protocol != self.extended_version:
            self.latch('ground commissioning requires firmware V3.1')

    def query(self):
        previous = self.previous_encoder
        started = time.monotonic() if self.metrics else None
        try:
            feedback = super().query()
            self.feedback_stamp_ns = self.wall_ns()
        finally:
            if self.metrics:
                self.metrics.sample('uart_query_ms', (time.monotonic() - started) * 1000)
        if self.encoder_boot is None:
            self.encoder_boot = feedback.boot
        if self.encoder_boot is None:
            self.encoder_valid = False
        elif feedback.boot != self.encoder_boot:
            self.encoder_invalid_reason = 'firmware boot identity changed'
        elif feedback.reason == 'ENCODER':
            self.encoder_invalid_reason = 'firmware encoder fault'
        elif previous is not None:
            previous_ticks, previous_at = previous
            # Bound jumps by the firmware's existing 160 RPM measured speed ceiling.
            max_delta = 200 + (self.feedback_at - previous_at) * 160 * CPR / 60
            if any(abs(a - b) > max_delta for a, b in zip(feedback.ticks, previous_ticks)):
                self.encoder_invalid_reason = 'discontinuous encoder count'
        self.previous_encoder = feedback.ticks, self.feedback_at
        self.encoder_valid = self.encoder_boot is not None and self.encoder_invalid_reason is None
        return feedback

    def check_feedback(self, **kwargs):
        if self.encoder_invalid_reason:
            raise ProtocolError(self.encoder_invalid_reason)
        super().check_feedback(**kwargs)

    def begin_stop(self, reason='PREFLIGHT'):
        self.stop_reason = reason
        self.reentry = None
        super().begin_stop()

    def check_session(self):
        if self.session_deadline is not None and self.now() + SESSION_STOP_MARGIN_S >= self.session_deadline:
            raise ProtocolError('commissioning session budget expired')

    def observe_publishers(self, commands, controls):
        if not self.commissioning or self.state == State.FAULT_LATCHED:
            return
        if len(commands) > 1 or len(controls) > 1:
            self.latch('conflicting ground command publishers')
            return
        pair = (commands[0], controls[0]) if commands and controls else None
        if self.owner_endpoints is not None and pair != self.owner_endpoints:
            self.latch('ground command owner lost or changed')
        elif pair != self.candidate_endpoints:
            self.candidate_endpoints = pair
            self.discovery_at = self.now() if pair else None

    def issue_challenge(self):
        if (not self.commissioning or self.state == State.FAULT_LATCHED
                or self.protocol != self.extended_version):
            return None
        try:
            t1 = self.clock.check()
        except AuthorityError as exc:
            self.latch(str(exc))
            return None
        now = self.now()
        # Retain enough history for a timely response that waited in DDS.
        # Both ingress age and callback age are still checked independently.
        self.challenges = {key: when for key, when in self.challenges.items()
                           if now - when < 2 * DEADMAN_S}
        self.challenges[t1] = now
        return t1

    def latch(self, reason):
        if self.metrics and self.state != State.FAULT_LATCHED:
            latest = self.metrics.latest
            latest.update(fault_reason=reason, fault_clock_proof_age_s=self.clock_proof_age(),
                          fault_control_timestamps=latest.get('control_timestamps'),
                          fault_command_age_s=(self.now() - self.command_at
                                               if self.command_at is not None else None))
        super().latch(reason)

    def clock_result(self, result, reason=None):
        if self.metrics:
            self.metrics.event('clock_' + result)
            if reason:
                self.metrics.event('clock_reason:' + reason)

    def control(self, message, info):
        if not self.commissioning or self.state == State.FAULT_LATCHED:
            return
        if (message.get('session') != self.session or message.get('source') != self.source
                or not token_valid(message.get('activation'))):
            self.clock_result('ignored', 'session_source_or_activation')
            return
        activation = message['activation']
        if self.activation is not None and activation != self.activation:
            self.latch('ground control activation changed')
            return
        kind = message.get('kind')
        if kind == 'fault' and self.activation is not None:
            try:
                check_dds_publisher(info, self.owner_endpoints[1])
                DDSStamp.read(info, self.clock.check(), remote=not self.pi_nav2)
            except AuthorityError as exc:
                self.latch(str(exc))
            else:
                reason = message.get('reason')
                self.latch('adapter fault: ' + (reason[:160] if isinstance(reason, str) else 'cancelled'))
            return
        if kind != 'clock':
            return
        if self.candidate_endpoints is None or self.now() - self.discovery_at < DISCOVERY_S:
            self.clock_result('ignored', 'publisher_discovery')
            return
        t1, t2, t3 = (message.get(name) for name in ('t1', 't2', 't3'))
        challenge_at = self.challenges.pop(t1, None) if type(t1) is int else None
        if challenge_at is None:
            self.clock_result('failure', 'unknown_or_expired_probe')
            return
        try:
            wall = self.clock.check()
            check_dds_publisher(info, self.candidate_endpoints[1])
            if not isinstance(info, dict):
                raise AuthorityError('DDS timestamp metadata unavailable')
            sent, received = info.get('source_timestamp'), info.get('received_timestamp')
            if (type(sent) is not int or type(received) is not int or type(t3) is not int
                    or not 0 <= sent - t3 < CLOCK_OFFSET_NS
                    or not 0 <= wall - received < DEADMAN_NS):
                raise AuthorityError('invalid clock response DDS metadata')
            received_at = arrival_monotonic(received, wall, self.now())
            if not 0 <= received_at - challenge_at < DEADMAN_S:
                raise AuthorityError('late clock response')
            if self.metrics and type(t2) is int and 0 < t2 < (1 << 63):
                self.metrics.sample('clock_rtt_ms', ((received - t1) - (t3 - t2)) / 1e6)
                self.metrics.latest['clock_offset_bounds_ms'] = [(t3 - received) / 1e6, (t2 - t1) / 1e6]
            if not clock_proven(t1, t2, t3, received):
                raise AuthorityError('clock offset bounds unverified')
        except AuthorityError as exc:
            self.clock_result('failure', str(exc))
            self.clock_status = 'CLOCK_UNVERIFIED'
            return
        if self.clock_ok_at is not None:
            age = received_at - self.clock_ok_at
            if age >= CLOCK_PROOF_S:
                self.clock_result('failure', 'proof_arrived_after_expiry')
                if self.metrics:
                    self.metrics.latest['timeout_age_s'] = age
                self.latch('ground authority clock proof missing or expired')
                return
            if age <= 0:
                self.clock_result('ignored', 'out_of_order_proof')
                return
        if self.activation is None:
            self.activation = activation
            self.owner_endpoints = self.candidate_endpoints
            self.authorized_after = wall
        self.clock_ok_at = received_at
        self.clock_status = 'VERIFIED'
        self.clock_result('success')

    def clock_proof_age(self):
        return self.now() - self.clock_ok_at if self.clock_ok_at is not None else None

    def require_authority(self):
        self.clock.check()
        age = self.clock_proof_age()
        if age is None or age >= CLOCK_PROOF_S:
            if self.metrics:
                self.metrics.latest['timeout_age_s'] = age
            raise AuthorityError('ground authority clock proof missing or expired')

    def command(self, components, stamp, info):
        if self.activation is None or self.state not in (*ACTIVE, State.READY):
            return
        if stamp.session != self.session:
            return
        if stamp.activation != self.activation or stamp.source != self.source:
            self.latch('ground command owner mismatch')
            return
        if (stamp.source_ns - CLOCK_OFFSET_NS <= max(self.authorized_after, self.source_cutoff)
                or stamp.source_ns <= self.last_source):
            return
        try:
            self.check_session()
            self.require_authority()
            active = self.state in ACTIVE
            if active:
                self.require_fresh()
            wall = self.clock.check()
            check_dds_publisher(info, self.owner_endpoints[0])
            dds = DDSStamp.read(info, wall, remote=not self.pi_nav2)
            age = wall - stamp.source_ns + CLOCK_OFFSET_NS
            # Pi ingress preserves both laptop timestamps. DDS publication is on the Pi.
            if self.pi_nav2 != (stamp.health_until_ns is not None):
                raise AuthorityError('command clock domain mismatch')
            receipt_limit = dds.source + (CLOCK_OFFSET_NS if self.pi_nav2 else 0)
            if not (stamp.source_ns <= stamp.receipt_ns <= receipt_limit and 0 <= age < DEADMAN_NS):
                raise AuthorityError('stale original command or inconsistent adapter receipt stamp')
            self.last_source = stamp.source_ns
            self.last_stamp = stamp
            self.requested_body = (components[0], components[5])
            self.requested, self.limited, self.limited_body = self.limits.targets(components)
            if not any(components):
                if active or self.command_at is not None:
                    self.begin_stop('NORMAL_ZERO')
                elif self.low_inhibited:
                    self.zero_after_inhibit = True
                    self.reentry = None
                return
            if any(raw and abs(rpm) < MIN_ACTIVE_RPM
                   for raw, rpm in zip(self.requested, self.limited)) or not any(self.limited):
                self.low_inhibited = True
                self.zero_after_inhibit = False
                self.reentry = None
                self.stop_reason = 'LOW_SPEED_INHIBITED'
                if active or self.command_at is not None:
                    self.begin_stop(self.stop_reason)
                return
            if self.low_inhibited and not self.readmit(stamp.source_ns):
                return
            if active and directions(self.sent) != directions(self.limited):
                self.begin_stop('DIRECTION_OR_DRIVEN_WHEEL_CHANGE')
                return
            if self.pi_nav2:
                remaining = stamp.health_until_ns - wall - CLOCK_OFFSET_NS
                if remaining <= 0:
                    raise AuthorityError('navigation feedback expired')
                age = max(age, DEADMAN_NS - remaining)
            self.command_at = self.now() - age / NS
        except (ValueError, RuntimeError, OSError) as exc:
            # Reject invalid numeric values without putting NaN/Inf on status JSON.
            if self.requested_body and not all(map(math.isfinite, self.requested_body)):
                self.requested_body = None
            self.latch(str(exc))

    def readmit(self, source_ns):
        minimum = MIN_ACTIVE_RPM if self.zero_after_inhibit else REENTRY_RPM
        if any(0 < abs(rpm) < minimum for rpm in self.limited):
            self.reentry = None
            return False
        pattern = directions(self.limited)
        if self.reentry:
            since, last, previous_pattern, count = self.reentry
        else:
            since, last, previous_pattern, count = source_ns, source_ns, pattern, 0
        if pattern != previous_pattern or source_ns - last >= DEADMAN_NS:
            since, count = source_ns, 0
        self.reentry = (since, source_ns, pattern, count + 1)
        if count + 1 >= 3 and source_ns - since >= round(REENTRY_S * NS):
            self.low_inhibited = self.zero_after_inhibit = False
            self.reentry = None
            return True
        return False

    def tick(self):
        if not self.commissioning:
            super().tick()
            return
        try:
            if self.state != State.FAULT_LATCHED:
                self.check_session()
                self.clock.check()
                if self.activation is not None:
                    self.require_authority()
            active = self.state in ACTIVE
            if active:
                self.require_fresh()
            self.query()
            if self.state == State.FAULT_LATCHED:
                return
            self.check_feedback(armed=active, running=active)
            if self.state == State.STOPPING:
                self.settle()
            elif active:
                self.require_authority()
                self.send_velocity(starting=False)
            elif self.state == State.READY and self.command_at is not None:
                self.require_authority()
                self.require_fresh()
                if not self.feedback.stationary:
                    raise ProtocolError('wheels moved before arming')
                self.check_session()
                if self.session_deadline is None:
                    self.session_deadline = self.now() + self.duration
                self.stop_verified = False
                self.uart.exchange('ARM_CONT', 'OK ARMED CONT')
                self.query()
                self.check_feedback(armed=True, running=False)
                self.require_authority()
                self.send_velocity(starting=True)
        except (RuntimeError, OSError) as exc:
            self.latch(str(exc))

    def status(self):
        result = super().status()
        wall = self.wall_ns()
        status = ('DISABLED' if not self.commissioning else
                  'INHIBITED' if self.state == State.FAULT_LATCHED else
                  'OWNED' if self.activation is not None else 'WAITING_FOR_OWNER')
        remaining = (max(0.0, self.session_deadline - self.now()) if self.session_deadline is not None
                     else self.duration)
        result.update({
            'version': '3.2', 'bridge_mode': self.commissioning or 'read-only',
            'session_id': self.session, 'authority_state': status, 'adapter_activation': self.activation,
            'pi_nav2': self.pi_nav2, 'clock_status': self.clock_status, 'clock_proof_age_s': self.clock_proof_age(),
            'command_source': self.source,
            'session_state': ('DISABLED' if not self.commissioning else
                              'INHIBITED' if self.state == State.FAULT_LATCHED else
                              'TIMED' if self.session_deadline is not None else 'AWAITING_FIRST_ARM'),
            'session_time_remaining_s': remaining,
            'requested_body': self.requested_body, 'limited_body': self.limited_body,
            'source_age_s': ((wall - self.last_stamp.source_ns + CLOCK_OFFSET_NS) / NS
                             if self.last_stamp else None),
            'adapter_receipt_age_s': ((wall - self.last_stamp.receipt_ns + CLOCK_OFFSET_NS) / NS
                                      if self.last_stamp else None),
            'stop_reason': self.stop_reason, 'low_speed_inhibited': self.low_inhibited,
            'stationary_source_cutoff_ns': (
                max(self.source_cutoff + CLOCK_OFFSET_NS, self.last_source)
                if result['stationary_ready'] and self.command_at is None else None),
            'encoder_continuity_valid': self.encoder_valid,
            'encoder_discontinuity_reason': self.encoder_invalid_reason,
        })
        return result

    def shutdown(self, *, stationary=False):
        if not self.commissioning and not stationary:
            return super().shutdown()
        if self.commissioning:
            self.latch('operator cancellation or ROS shutdown')
        try:
            self.verify_stationary_stop()
        except (RuntimeError, OSError) as exc:
            self.report(f'FINAL STATIONARY STOP UNVERIFIED: {exc}. CUT MOTOR POWER PHYSICALLY.')
            return False
        self.report('Final STOP and stationary feedback verified')
        return True


def attach_bridge_link(node, executor, bridge, *, status_topic=STATUS_TOPIC,
                       ground_topic=GROUND_TOPIC, control_topic=CONTROL_TOPIC):
    """Shared clock/status wiring for the real bridge and no-motion link probe."""
    from std_msgs.msg import String

    sequence = 0
    metrics = bridge.metrics
    qos = telemetry_qos() if bridge.source == 'nav2' else status_qos(publisher=True)
    status_pub = node.create_publisher(String, status_topic, qos)

    def observe():
        bridge.observe_publishers(inspect_publishers(node, ground_topic, metrics),
                                  inspect_publishers(node, control_topic, metrics))

    def on_control(msg, info):
        if metrics:
            started = time.monotonic()
            metrics.received('control', info)
        try:
            observe()
            try:
                data = json.loads(msg.data) if len(msg.data) <= 512 else None
            except ValueError:
                data = None
            if isinstance(data, dict):
                bridge.control(data, info)
            else:
                bridge.clock_result('ignored', 'malformed_control')
                if bridge.activation:
                    bridge.latch('invalid ground control message')
        finally:
            if metrics:
                metrics.sample('control_callback_ms', (time.monotonic() - started) * 1000)

    def before_tick():
        observe()
        age = bridge.clock_proof_age()
        if bridge.state != State.FAULT_LATCHED and age is not None and age >= CLOCK_PROOF_S:
            if metrics:
                metrics.event('pending_control_checks')
            executor.take_pending(control_sub)

    def publish():
        nonlocal sequence
        sequence += 1
        payload = bridge.status()
        before = bridge.state
        probe = bridge.issue_challenge()
        if bridge.state != before:
            payload = bridge.status()
        payload['status_sequence'] = sequence
        payload['clock_probe_ns'] = probe
        msg = String(data=json.dumps(payload, allow_nan=False))
        started = time.monotonic() if metrics else None
        status_pub.publish(msg)
        if metrics:
            metrics.sample('status_publish_ms', (time.monotonic() - started) * 1000)
            metrics.sequence(sequence)
            metrics.event('status_published')
            metrics.latest['last_clock_proof_age_s'] = bridge.clock_proof_age()
        return payload

    if bridge.commissioning:
        control_sub = executor.subscribe(node, String, control_topic, on_control, command_qos(),
                                         metrics=metrics, channel='control')
    return before_tick, publish


def run_ros(port, commissioning, source, duration, limits, diagnostics=False, *, bridge_class=BridgeV32,
            observer=None, stationary_shutdown=False):
    from amr_nav2_peer import public_domain
    domain = public_domain()
    if domain is not None and commissioning and source != 'nav2':
        raise AuthorityError('private motor domain requires Nav2')
    separate_telemetry = source == 'nav2' or domain is not None
    if separate_telemetry:
        # Let Fast DDS send telemetry on its worker. A Python publishing
        # thread would still hold the GIL inside Humble's rcl_publish binding.
        os.environ.setdefault('RMW_FASTRTPS_PUBLICATION_MODE', 'ASYNCHRONOUS')
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from sensor_msgs.msg import JointState

    node = executor = joint_telemetry = None
    result = 0
    metrics = None
    rclpy.init(args=[])
    try:
        executor = metadata_executor() if commissioning else SingleThreadedExecutor()
        node = rclpy.create_node('amr_serial_bridge')
        if separate_telemetry:
            joint_telemetry = JointTelemetry(diagnostics)
        else:
            joint_pub = node.create_publisher(JointState, '/joint_states', 10)
        metrics = LinkMetrics('bridge') if diagnostics else None
        reporter = PeriodicLinkReport(metrics) if metrics else None
        with exclusive_uart(port) as uart:
            bridge = bridge_class(uart, commissioning, source, duration, limits, metrics=metrics)
            bridge.pi_nav2 = domain is not None
            before_tick, publish = attach_bridge_link(node, executor, bridge)
            try:
                bridge.start()

                def tick():
                    if metrics:
                        started = time.monotonic()
                        metrics.timer(LOOP_PERIOD_S, started)
                    if commissioning:
                        before_tick()
                    if joint_telemetry is not None and not joint_telemetry.process.is_alive():
                        bridge.latch('joint telemetry process exited')
                    previous = bridge.feedback
                    bridge.tick()
                    feedback = bridge.feedback
                    fresh_encoders = bridge.encoder_valid and feedback is not previous
                    stamp = (node.get_clock().now().to_msg()
                             if fresh_encoders and joint_telemetry is None else None)
                    status = publish()
                    if observer is not None:
                        observer(status)
                    if fresh_encoders and joint_telemetry is not None:
                        handed_off = joint_telemetry.slot.submit(
                            bridge.feedback_stamp_ns, bridge.feedback_at, feedback.ticks, feedback.measured)
                        if metrics and not handed_off:
                            metrics.event('joint_handoff_busy')
                    elif fresh_encoders:
                        msg = JointState()
                        msg.header.stamp = stamp
                        msg.name = ['left_wheel_joint', 'right_wheel_joint']
                        msg.position = [ticks * 2 * math.pi / CPR for ticks in feedback.ticks]
                        msg.velocity = [rpm * 2 * math.pi / 60 for rpm in feedback.measured]
                        joint_started = time.monotonic() if metrics else None
                        joint_pub.publish(msg)
                        if metrics:
                            metrics.sample('joint_publish_ms', (time.monotonic() - joint_started) * 1000)
                    if metrics:
                        reporter.poll()
                        metrics.sample('timer_execution_ms', (time.monotonic() - started) * 1000)

                def on_command(msg, info):
                    before_tick()
                    try:
                        stamp = CommandStamp.from_header(msg.header)
                    except AuthorityError as exc:
                        if bridge.activation:
                            bridge.latch(str(exc))
                    else:
                        bridge.command(twist_components(msg.twist), stamp, info)

                if commissioning:
                    executor.subscribe(node, TwistStamped, GROUND_TOPIC, on_command, command_qos())
                    if observer is None:
                        print(f'Session {bridge.session}, selected source {source}', flush=True)
                node.create_timer(LOOP_PERIOD_S, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
                executor.add_node(node)
                executor.spin()
            finally:
                verified = bridge.shutdown(stationary=True) if stationary_shutdown else bridge.shutdown()
                if not verified:
                    result = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (AuthorityError, RuntimeError, OSError, ImportError) as exc:
        print(f'Bridge stopped: {exc}', file=sys.stderr, flush=True)
        result = 1
    finally:
        if joint_telemetry is not None:
            joint_telemetry.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        if metrics:
            print(json.dumps(summarize(metrics.snapshot())), file=sys.stderr, flush=True)
    return result


def main(*, runner=None, description=__doc__):
    if runner is None:
        runner = run_ros
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--port', default='/dev/serial0')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--ground-control', action='store_true')
    modes.add_argument('--raised-commissioning', action='store_true')
    parser.add_argument('--source', choices=SOURCE_TOPICS)
    parser.add_argument('--session-seconds', type=float)
    parser.add_argument('--max-linear', type=float, default=MAX_LINEAR_MPS)
    parser.add_argument('--max-angular', type=float, default=MAX_ANGULAR_RPS)
    parser.add_argument('--max-wheel-rpm', type=float, default=MAX_WHEEL_RPM)
    parser.add_argument('--link-diagnostics', action='store_true', help='emit bounded link summaries')
    args = parser.parse_args()
    commissioning = 'ground' if args.ground_control else 'raised' if args.raised_commissioning else None
    if bool(commissioning) != bool(args.source):
        parser.error('commissioning requires one explicitly selected --source')
    if args.session_seconds is not None and (
            commissioning != 'ground' or not 1 <= args.session_seconds <= GROUND_MAX_S):
        parser.error('--session-seconds is for ground control and must be between 1 and 120')
    try:
        limits = Limits(args.max_linear, args.max_angular, args.max_wheel_rpm)
    except ValueError as exc:
        parser.error(str(exc))
    if commissioning:
        token = 'CLEAR_AREA_CUTOFF_READY' if commissioning == 'ground' else 'WHEELS_UP_CUTOFF_READY'
        setup = 'clear the floor area' if commissioning == 'ground' else 'securely raise both wheels'
        if not sys.stdin.isatty():
            return 1
        try:
            entered = input(f'Verify the physical motor cutoff and {setup}. '
                            f'Confirm the isolated trusted LAN and synchronized Pi/laptop clocks, '
                            f'then type {token}: ')
        except (EOFError, KeyboardInterrupt):
            return 1
        if entered.strip() != token:
            return 1
    try:
        return runner(args.port, commissioning, args.source, args.session_seconds, limits, args.link_diagnostics)
    except ImportError as exc:
        print(f'Bridge dependency unavailable: {exc}', file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
