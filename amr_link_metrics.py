"""Bounded, opt-in link instrumentation. Output runs outside the safety loop."""
from collections import Counter, deque
import json
import math
from queue import Full, Queue
import sys
from threading import Thread
import time


class LinkMetrics:
    def __init__(self, role):
        self.role = role
        self.counts = Counter()
        self.latest = {}
        self.values = {}
        self.maxima = {}
        self.previous = {}

    def event(self, name):
        self.counts[name] += 1

    def sample(self, name, value):
        self.values.setdefault(name, deque(maxlen=4096)).append(value)
        self.maxima[name] = max(value, self.maxima.get(name, value))

    def interval(self, name, when):
        previous = self.previous.get(name)
        self.previous[name] = when
        if previous is not None:
            self.sample(name, (when - previous) * 1000)

    def timer(self, period, now):
        previous = self.previous.get('timer_interval_ms')
        self.interval('timer_interval_ms', now)
        due = self.previous.get('timer_next_due', now)
        delay = max(0., now - due)
        self.previous['timer_next_due'] = due + (math.floor(delay / period) + 1) * period
        if previous is not None:
            self.sample('timer_delay_ms', delay * 1000)

    def received(self, channel, info, callback_ns=None):
        wall = time.time_ns() if callback_ns is None else callback_ns
        self.event(channel + '_callbacks')
        self.interval(channel + '_callback_interval_ms', time.monotonic())
        source = info.get('source_timestamp') if isinstance(info, dict) else None
        receipt = info.get('received_timestamp') if isinstance(info, dict) else None
        source = source if type(source) is int and source > 0 else None
        receipt = receipt if type(receipt) is int and receipt > 0 else None
        self.latest[channel + '_timestamps'] = dict(dds_source_ns=source, dds_received_ns=receipt,
                                                     callback_processing_ns=wall)
        if receipt is not None:
            self.interval(channel + '_dds_interval_ms', receipt / 1e9)
            self.sample(channel + '_processing_delay_ms', (wall - receipt) / 1e6)

    def sequence(self, sequence):
        if type(sequence) is int and sequence > 0:
            previous = self.latest.get('status_sequence')
            if previous is None or sequence > previous:
                if previous is not None and sequence > previous + 1:
                    self.counts['status_sequence_skipped'] += sequence - previous - 1
                self.latest['status_sequence'] = sequence
            else:
                self.event('status_sequence_out_of_order')

    def snapshot(self):
        return dict(role=self.role, counts=dict(self.counts), latest=dict(self.latest),
                    values={key: list(vals) for key, vals in self.values.items()},
                    maxima=dict(self.maxima))


def summarize(snapshot):
    aggregates = {}
    for name, samples in snapshot.pop('values').items():
        samples.sort()
        def percentile(p):
            return samples[math.ceil(len(samples) * p) - 1]
        aggregates[name] = dict(samples=len(samples), max=snapshot['maxima'][name],
                                p50=percentile(.5), p95=percentile(.95), p99=percentile(.99))
    snapshot.pop('maxima')
    snapshot['intervals_ms'] = aggregates
    return snapshot


class PeriodicLinkReport:
    def __init__(self, metrics):
        self.metrics = metrics
        self.next = time.monotonic() + 10
        self.queue = Queue(maxsize=1)
        Thread(target=self._print, daemon=True).start()

    def poll(self):
        now = time.monotonic()
        if now >= self.next:
            self.next = now + 10
            if not self.queue.full():
                try:
                    self.queue.put_nowait(self.metrics.snapshot())
                except Full:
                    pass

    def _print(self):
        while True:
            print(json.dumps(summarize(self.queue.get())), file=sys.stderr, flush=True)
