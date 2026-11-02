"""Deterministic, bounded timing faults for an isolated classroom Gazebo trial.

The relay keeps original sensor stamps and contents.  In particular, an old
measurement is never restamped to make a delayed sample look fresh.
"""

from dataclasses import dataclass


TOPICS = ('scan', 'odom', 'tf')


@dataclass(frozen=True)
class Decision:
    drop: bool
    delay_seconds: float
    reason: str


class FaultProfile:
    """Return repeatable faults after an explicit profile switch.

    All schedules use the relay's monotonic clock, not /clock.  The initial
    profile is always clean so Gazebo and AMCL can acquire a real baseline.
    """

    JITTER = {
        'scan': (0., .04, .12, .22, .06, .15),
        'odom': (0., .03, .09, .18, .05),
        'tf': (0., .02, .11, .23, .04),
    }
    OUTAGES = {
        # Longer than sensor_timeout + fault_seconds, and long enough for a
        # separate health tick to observe a sustained loss on every stream.
        'scan': (4., 7.0),
        'odom': (10., 13.0),
        'tf': (16., 19.0),
    }
    TRANSIENT_DELAY = .35
    TRANSIENT_DELAY_END = .8
    TRANSIENT_DROP = (1.55, 2.35)

    def __init__(self):
        self.name = 'clean'
        self.started_at = 0.
        self.sequence = {topic: 0 for topic in TOPICS}

    def set_profile(self, name, now):
        if name not in ('clean', 'jitter', 'outages', 'mixed', 'transient'):
            raise ValueError('unknown fault profile: '+str(name))
        self.name = name
        self.started_at = float(now)
        self.sequence = {topic: 0 for topic in TOPICS}

    def decide(self, topic, now):
        if topic not in TOPICS:
            raise ValueError('unknown stream: '+str(topic))
        index = self.sequence[topic]
        self.sequence[topic] += 1
        if self.name == 'clean':
            return Decision(False, 0., 'clean')
        elapsed = float(now)-self.started_at
        if self.name == 'transient':
            # First model ordinary hundreds-of-ms network latency. Later a
            # bounded 0.8 s gap makes the localization observer temporarily
            # reject stale scan/odom without reaching its 0.5 s latch grace
            # after the 0.7 s sensor-age limit. Original stamps are kept.
            if 0. <= elapsed < self.TRANSIENT_DELAY_END:
                return Decision(False, self.TRANSIENT_DELAY, 'transient_delay')
            if self.TRANSIENT_DROP[0] <= elapsed < self.TRANSIENT_DROP[1]:
                return Decision(True, 0., 'transient')
            return Decision(False, 0., 'clean')
        if self.name in ('outages', 'mixed'):
            start, end = self.OUTAGES[topic]
            if start <= elapsed < end:
                return Decision(True, 0., 'outage')
        if self.name in ('jitter', 'mixed'):
            # Sparse lost packets plus the measured-order latency pattern.  No
            # random seed or probabilistic test flakiness is involved.
            drop_every = {'scan': 31, 'odom': 47, 'tf': 59}[topic]
            if index > 0 and index % drop_every == 0:
                return Decision(True, 0., 'packet_loss')
            return Decision(False, self.JITTER[topic][index % len(self.JITTER[topic])], 'jitter')
        return Decision(False, 0., 'clean')
