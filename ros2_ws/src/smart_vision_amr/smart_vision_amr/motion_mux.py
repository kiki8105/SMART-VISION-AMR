"""One velocity publisher, with leased manual control ahead of navigation.

The core deliberately has no ROS imports so timing and ownership races can be
tested without a robot.  Navigation never publishes directly to /cmd_vel in
the supplied launch files; only this node does.
"""

import json
import math
import time
import uuid


class MotionMuxCore:
    """Arbitrate commands using a monotonic dead-man lease, never ROS time."""

    MODES = frozenset({'manual', 'localization', 'hold', 'auto', 'estop', 'reset'})
    RECOVERY_SAFETY_SOURCE = 'path_follower_recovery_safety'
    EKF_SENSOR_SAFETY_SOURCE = 'ekf_sensor_safety'
    LOCALIZATION_SAFETY_SOURCE = 'localization_safety'
    SAFETY_REASONS = {
        RECOVERY_SAFETY_SOURCE: 'Reverse recovery safety stop',
        EKF_SENSOR_SAFETY_SOURCE: 'EKF wheel, IMU, or LiDAR input lost',
        LOCALIZATION_SAFETY_SOURCE: 'Localization confidence unsafe',
    }

    def __init__(self, clock=time.monotonic, manual_timeout=.5, auto_timeout=.5,
                 max_linear_velocity=.25, max_angular_velocity=1.0,
                 max_auto_age_seconds=.35, auto_future_tolerance_seconds=.25,
                 require_localization_health=False,
                 localization_health_timeout=1.0,
                 localization_warning_speed_factor=.5,
                 localization_rotation_limit=.25, startup_estop=False,
                 marker_alignment_rotation_limit=.15):
        self.clock = clock
        self.manual_timeout = float(manual_timeout)
        self.auto_timeout = float(auto_timeout)
        self.max_linear_velocity = float(max_linear_velocity)
        self.max_angular_velocity = float(max_angular_velocity)
        if any(not math.isfinite(value) or value <= 0 for value in (
                self.manual_timeout, self.auto_timeout, self.max_linear_velocity,
                self.max_angular_velocity)):
            raise ValueError('Velocity limits and timeouts must be finite and positive')
        self.mode = 'auto'
        self.reason = 'Navigation control'
        self.source_id = None
        self.retired_sources = set()
        self.sequences = {}
        self.manual_at = None
        self.manual_velocity = (0.0, 0.0)
        self.auto_at = None
        self.auto_velocity = (0.0, 0.0)
        self.estop_latched = bool(startup_estop)
        if self.estop_latched:
            self.mode, self.reason = 'estop', 'Startup interlock; explicit safety reset required'
        self.max_auto_age_ns = int(max_auto_age_seconds * 1e9)
        self.auto_future_tolerance_ns = int(auto_future_tolerance_seconds*1e9)
        self.auto_source_id = None
        self.retired_auto_sources = set()
        self.auto_sequences = {}
        self.auto_last_stamp = -1
        self.auto_rejected_reason = ''
        self.require_localization_health = bool(require_localization_health)
        self.localization_health_timeout = float(localization_health_timeout)
        self.localization_warning_speed_factor = float(localization_warning_speed_factor)
        self.localization_rotation_limit = float(localization_rotation_limit)
        self.marker_alignment_rotation_limit = float(marker_alignment_rotation_limit)
        if (self.max_auto_age_ns <= 0 or self.auto_future_tolerance_ns < 0
                or self.localization_health_timeout <= 0
                or not 0 < self.localization_warning_speed_factor <= 1
                or not 0 < self.localization_rotation_limit <= self.max_angular_velocity
                or not 0 < self.marker_alignment_rotation_limit <= self.max_angular_velocity):
            raise ValueError('Invalid command age or localization safety limits')
        self.localization_at = None
        self.localization_health = None
        self.manual_operation = None
        self.control_received = self.control_accepted = self.control_rejected = 0
        self.auto_received = self.auto_accepted = self.auto_rejected = 0
        self.last_control = self.last_auto = None
        self.last_control_rejection = self.last_auto_rejection = None
        self.control_rejected_reason = ''
        self.control_by_origin = {}
        self.auto_by_origin = {}

    def _receipt(self, channel, payload, accepted, reason, origin):
        """Record once at the public boundary, including pre-core rejects.

        Nested application helpers do not create receipts. Keep only the most
        recent packet and rejection; counters never enter the publish bypass key.
        """
        origin = origin if origin in {'topic', 'service', 'core', 'legacy'} else 'core'
        origins = getattr(self, channel+'_by_origin')
        counts = origins.setdefault(origin, {'received': 0, 'accepted': 0, 'rejected': 0})
        counts['received'] += 1
        counts['accepted' if accepted else 'rejected'] += 1
        packet = payload if isinstance(payload, dict) else {}
        source, seq, mode = (packet.get('source_id'), packet.get('seq'), packet.get('mode'))
        receipt = {'source_id': source[:128] if isinstance(source, str) else None,
                   'seq': seq if type(seq) is int else None,
                   'mode': mode[:32] if isinstance(mode, str) else None,
                   'accepted': bool(accepted), 'reason': reason, 'origin': origin,
                   'received_at_monotonic_ns': int(self.clock()*1e9)}
        setattr(self, channel+'_received', getattr(self, channel+'_received')+1)
        outcome = channel+('_accepted' if accepted else '_rejected')
        setattr(self, outcome, getattr(self, outcome)+1)
        setattr(self, 'last_'+channel, receipt)
        if not accepted:
            setattr(self, 'last_'+channel+'_rejection', dict(receipt))
        return bool(accepted)

    def reject_control(self, payload, reason, origin='topic'):
        self.control_rejected_reason = reason
        return self._receipt('control', payload, False, reason, origin)

    def reject_auto(self, payload, reason, origin='topic'):
        self.auto_rejected_reason = reason
        return self._receipt('auto', payload, False, reason, origin)

    def _velocity(self, linear, angular):
        if (isinstance(linear, bool) or isinstance(angular, bool)
                or not isinstance(linear, (int, float))
                or not isinstance(angular, (int, float))):
            raise ValueError('Velocity must be numeric')
        try:
            linear, angular = float(linear), float(angular)
        except OverflowError as exc:
            raise ValueError('Velocity must be finite') from exc
        if not math.isfinite(linear) or not math.isfinite(angular):
            raise ValueError('Velocity must be finite')
        return (max(-self.max_linear_velocity, min(self.max_linear_velocity, linear)),
                max(-self.max_angular_velocity, min(self.max_angular_velocity, angular)))

    def _expire(self):
        if (self.mode in {'manual', 'localization'} and self.manual_at is not None
                and self.clock() - self.manual_at >= self.manual_timeout):
            self.mode = 'hold'
            self.reason = 'Operator control lease expired; explicit resume required'
            self.manual_velocity = (0.0, 0.0)
            self.auto_at = None

    def receive_auto(self, linear, angular):
        accepted = self._apply_auto(linear, angular)
        return self._receipt('auto', {'mode': 'auto'}, accepted,
                             '' if accepted else self.auto_rejected_reason, 'legacy')

    def _apply_auto(self, linear, angular):
        """Only a post-transition command may restart automatic motion."""
        self._expire()
        try:
            velocity = self._velocity(linear, angular)
        except ValueError:
            self.auto_at = None
            self.auto_rejected_reason = 'Invalid autonomous velocity'
            return False
        # Drop, rather than retain, navigation output during an override.
        if self.mode != 'auto' or self.estop_latched:
            self.auto_rejected_reason = 'Autonomous command blocked by '+self.mode+' control'
            return False
        self.auto_rejected_reason = ''
        self.auto_velocity, self.auto_at = velocity, self.clock()
        return True

    def receive_stamped_auto(self, payload, ros_now_ns):
        accepted = self._receive_stamped_auto(payload, ros_now_ns)
        return self._receipt('auto', payload, accepted,
                             '' if accepted else self.auto_rejected_reason, 'topic')

    def _receive_stamped_auto(self, payload, ros_now_ns):
        """Validate forwarding age and sequence before renewing the steady lease.

        A new bridge UUID may take over only with a stamp newer than every
        previously accepted source. A retired bridge can never resume. Commands
        rejected during manual/hold also consume their sequence so releasing a
        manual lease cannot replay navigation output buffered during override.
        """
        if not isinstance(payload, dict):
            self.auto_rejected_reason = 'Autonomous packet must be a JSON object'
            return False
        source, seq, stamp = (payload.get('source_id'), payload.get('seq'),
                              payload.get('sent_at_ns'))
        if (not isinstance(source, str) or not source or len(source) > 128
                or type(seq) is not int or seq < 0 or type(stamp) is not int
                or type(ros_now_ns) is not int or stamp < 0
                or stamp > ros_now_ns + self.auto_future_tolerance_ns
                or ros_now_ns-stamp > self.max_auto_age_ns
                or source in self.retired_auto_sources
                or seq <= self.auto_sequences.get(source, -1)
                or stamp < self.auto_last_stamp
                or (source != self.auto_source_id and stamp <= self.auto_last_stamp)):
            self.auto_rejected_reason = 'Delayed, replayed, or invalid autonomous command'
            return False
        try:
            self._velocity(payload.get('linear'), payload.get('angular'))
        except ValueError:
            self.auto_rejected_reason = 'Invalid autonomous velocity'
            self.auto_at = None
            return False
        if source != self.auto_source_id:
            if self.auto_source_id is not None:
                self.retired_auto_sources.add(self.auto_source_id)
            self.auto_source_id = source
        self.auto_sequences[source], self.auto_last_stamp = seq, stamp
        self.auto_rejected_reason = ''
        return self._apply_auto(payload['linear'], payload['angular'])

    def receive_localization_health(self, payload):
        if not isinstance(payload, dict) or payload.get('level') not in {
                'ok', 'healthy', 'warning', 'critical'}:
            return False
        self.localization_health = dict(payload)
        self.localization_at = self.clock()
        return True

    def _localization_policy(self):
        if (self.localization_at is None
                or self.clock()-self.localization_at > self.localization_health_timeout):
            return (0.0, False, False, 'Localization health missing or stale') if (
                self.require_localization_health) else (1.0, True, False, '')
        health = self.localization_health
        level = health['level']
        factor = (0.0 if level == 'critical' else
                  self.localization_warning_speed_factor if level == 'warning' else 1.0)
        return (factor, health.get('autonomous_allowed', True) is True,
                health.get('allow_localization', False) is True,
                '; '.join(str(x) for x in health.get('reasons', [])))

    def receive_control(self, payload, ros_now_ns, origin='core'):
        self.control_rejected_reason = ''
        accepted = self._receive_control(payload, ros_now_ns)
        return self._receipt('control', payload, accepted,
                             '' if accepted else self.control_rejected_reason, origin)

    def _reject_control(self, reason):
        self.control_rejected_reason = reason
        return False

    def _receive_control(self, payload, ros_now_ns):
        """Accept one atomic state/velocity packet from the current web bridge.

        ROS stamps reject delayed DDS packets.  Lease expiry still works while
        /clock is paused because lease receipt is timed with monotonic time.
        """
        self._expire()
        if not isinstance(payload, dict):
            return self._reject_control('Control packet must be a JSON object')
        source, seq, mode = (payload.get('source_id'), payload.get('seq'),
                             payload.get('mode'))
        stamp = payload.get('sent_at_ns')
        if not isinstance(source, str) or not source or len(source) > 128:
            return self._reject_control('Control source_id must be a nonempty string (max 128 characters)')
        if type(seq) is not int or seq < 0:
            return self._reject_control('Control seq must be a nonnegative integer')
        if not isinstance(mode, str) or mode not in self.MODES:
            return self._reject_control('Unknown control mode')
        if source in self.SAFETY_REASONS and mode != 'estop':
            return self._reject_control('Safety watchdog sources may only request estop')
        if type(stamp) is not int or stamp < 0 or type(ros_now_ns) is not int:
            return self._reject_control('Control sent_at_ns must be a nonnegative integer')
        if abs(ros_now_ns-stamp) > 1_000_000_000:
            return self._reject_control('Control timestamp differs by over 1s; check PC/robot clock sync and packet delay')
        if source in self.retired_sources:
            return self._reject_control('Control source was retired; reconnect the current bridge')
        if seq <= self.sequences.get(source, -1):
            return self._reject_control('Replayed or out-of-order control sequence')
        try:
            velocity = self._velocity(payload.get('linear', 0.0),
                                      payload.get('angular', 0.0))
        except ValueError as exc:
            return self._reject_control(str(exc))
        if mode == 'localization' and velocity[0] != 0.0:
            return self._reject_control('Localization control permits yaw only; linear must be zero')
        if source in self.SAFETY_REASONS:
            # Independent safety watchdogs can stop even a live manual lease.
            # Keep the web bridge as owner so its explicit emergency-reset
            # button remains the only way to clear the latched stop.
            self.sequences[source] = seq
            self.estop_latched = True
            self.mode, self.reason = 'estop', self.SAFETY_REASONS[source]
            self.manual_velocity, self.auto_at = (0.0, 0.0), None
            return True
        if self.estop_latched and mode not in {'reset', 'estop'}:
            return self._reject_control('Emergency stop is latched; explicit safety reset required')
        now = self.clock()
        if self.mode == 'manual' and mode == 'localization':
            return self._reject_control('Manual joystick lease has priority; acknowledge hold before localization')
        if (self.source_id is not None and source != self.source_id
                and (self.mode == 'manual' and mode != 'estop'
                     or self.mode == 'localization' and mode not in {'manual', 'estop'})):
            return self._reject_control('Another operator owns a live '+self.mode+' lease; release it or wait for expiry')
        if source != self.source_id:
            # Service stop/reset must not permanently retire a connected UI.
            if (self.source_id is not None
                    and not source.startswith('mux-service:')
                    and not self.source_id.startswith('mux-service:')):
                self.retired_sources.add(self.source_id)
            self.source_id = source
        self.sequences[source] = seq

        if mode == 'estop':
            self.estop_latched = True
            self.mode, self.reason = 'estop', 'Emergency stop latched'
            self.manual_velocity, self.auto_at = (0.0, 0.0), None
            return True
        if self.estop_latched:
            if mode != 'reset':
                return False
            self.estop_latched = False
            self.mode, self.reason = 'hold', 'Emergency reset; explicit resume required'
            self.manual_velocity, self.auto_at = (0.0, 0.0), None
            return True
        if mode in {'hold', 'reset'}:
            self.mode, self.reason = 'hold', 'Motion held'
            self.manual_velocity, self.auto_at = (0.0, 0.0), None
        elif mode in {'manual', 'localization'}:
            self.mode, self.reason = mode, ('Leased localization rotation' if mode == 'localization'
                                           else 'Leased manual control has priority')
            self.manual_velocity, self.manual_at = velocity, now
            self.auto_at = None
            self.manual_operation = payload.get('operation')
        else:
            # Repeated auto heartbeats must not erase fresh navigation commands.
            if self.mode != 'auto':
                self.auto_at = None
            self.mode, self.reason = 'auto', 'Navigation control'
            self.manual_velocity = (0.0, 0.0)
        return True

    def output(self):
        self._expire()
        if self.estop_latched:
            return 0.0, 0.0
        velocity = (0.0, 0.0)
        if self.mode in {'manual', 'localization'}:
            velocity = self.manual_velocity
        elif (self.mode == 'auto' and self.auto_at is not None
                and self.clock()-self.auto_at < self.auto_timeout):
            velocity = self.auto_velocity
        factor, auto_allowed, allow_rotation, _ = self._localization_policy()
        if self.mode == 'manual' and self.manual_operation == 'marker_alignment':
            # This is mission-driven observation, not the operator's bootstrap
            # localization exception. Even compatibility runtimes must have a
            # fresh, autonomous-ready health report before rotating for a tag.
            health_fresh = (self.localization_at is not None
                            and self.clock()-self.localization_at <= self.localization_health_timeout)
            if (not health_fresh or not auto_allowed
                    or self.localization_health.get('autonomous_allowed') is not True or factor <= 0.
                    or velocity[0] != 0.):
                return 0.0, 0.0
            angular = max(-self.marker_alignment_rotation_limit,
                          min(self.marker_alignment_rotation_limit, velocity[1]))
            return 0.0, angular*factor
        if (self.mode == 'localization'
                or self.mode == 'manual' and self.manual_operation == 'localization'):
            if allow_rotation and velocity[0] == 0.0:
                return 0.0, max(-self.localization_rotation_limit,
                                min(self.localization_rotation_limit, velocity[1]))
            return 0.0, 0.0
        if self.mode == 'auto' and not auto_allowed:
            return 0.0, 0.0
        return velocity[0]*factor, velocity[1]*factor

    def status(self):
        self._expire()
        factor, auto_allowed, allow_localization, health_reason = self._localization_policy()
        linear, angular = self.output()
        return {'mode': self.mode, 'available': True, 'reason': self.reason,
                'source_id': self.source_id, 'estop_latched': self.estop_latched,
                'control_seq': self.sequences.get(self.source_id, -1),
                'allow_localization': allow_localization,
                'auto_source_id': self.auto_source_id,
                'auto_rejected_reason': self.auto_rejected_reason,
                'control_rejected_reason': self.control_rejected_reason,
                **{name: getattr(self, name) for name in (
                    'control_received', 'control_accepted', 'control_rejected',
                    'auto_received', 'auto_accepted', 'auto_rejected',
                    'control_by_origin', 'auto_by_origin',
                    'last_control', 'last_control_rejection', 'last_auto', 'last_auto_rejection')},
                'output_owner': 'manual' if self.mode == 'manual' else self.mode,
                'localization_speed_factor': factor,
                'autonomous_allowed': auto_allowed,
                'localization_reason': health_reason,
                'output_linear': linear, 'output_angular': angular}


# Keep the arbitration core importable for non-ROS unit tests as well.
try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
except ImportError:
    Node = object


class MotionMuxNode(Node):
    def __init__(self, context=None, use_sim_time=None):
        overrides = []
        if use_sim_time is not None:
            from rclpy.parameter import Parameter
            overrides.append(Parameter('use_sim_time', value=bool(use_sim_time)))
        super().__init__('motion_mux', context=context, parameter_overrides=overrides)
        for name, default in (
                ('manual_timeout', .5), ('auto_timeout', .5),
                ('max_linear_velocity', .25), ('max_angular_velocity', 1.0),
                ('max_auto_age_seconds', .35),
                ('auto_future_tolerance_seconds', .25),
                ('require_localization_health', False),
                ('localization_health_timeout', 1.0),
                ('localization_warning_speed_factor', .5),
                ('localization_rotation_limit', .25),
                ('marker_alignment_rotation_limit', .15),
                ('startup_estop', False), ('allow_topic_reset', True),
                ('allow_legacy_auto_input', True)):
            self.declare_parameter(name, default)
        self.core = MotionMuxCore(**{
            name: self.get_parameter(name).value for name in (
                'manual_timeout', 'auto_timeout', 'max_linear_velocity',
                'max_angular_velocity', 'max_auto_age_seconds',
                'auto_future_tolerance_seconds',
                'require_localization_health', 'localization_health_timeout',
                'localization_warning_speed_factor', 'localization_rotation_limit',
                'marker_alignment_rotation_limit',
                'startup_estop')})
        self.allow_topic_reset = self.get_parameter('allow_topic_reset').value
        self.output_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.output_available = False
        self.output_conflict = 'Checking exclusive /cmd_vel ownership'
        self.output_publishers = 0
        self._ownership_checked_at = -math.inf
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(
            String, '/smart_vision_amr/motion_status', durable)
        self.control_pub = self.create_publisher(
            String, '/smart_vision_amr/manual_control', 10)
        self.service_source = 'mux-service:'+uuid.uuid4().hex
        self.service_seq = 0
        self.auto_started_ns = self.get_clock().now().nanoseconds
        self._health_sent_ns = -1
        self.status_sequence = 0
        self._status_published_at = -math.inf
        self._status_transition = None
        self.pending_reset = None
        self.last_zero_published_at = None
        if self.get_parameter('allow_legacy_auto_input').value:
            self.create_subscription(Twist, '/smart_vision_amr/navigation_cmd_vel',
                                     self.on_auto, 1)
        self.create_subscription(String, '/smart_vision_amr/navigation_control',
                                 self.on_navigation_control, 1)
        self.create_subscription(String, '/smart_vision_amr/localization_health',
                                 self.on_localization_health, 1)
        self.create_subscription(String, '/smart_vision_amr/manual_control',
                                 self.on_control, 1)
        # ROS timers stop when simulation time pauses. Dead-man watchdogs
        # must continue to run even then, so use a steady clock explicitly.
        from rclpy.clock import Clock, ClockType
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        # Reentrant services yield while steady timers/subscriptions process the
        # reset intent and independent sensor watchdogs can veto it.
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.safety_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '/smart_vision_amr/emergency_stop',
                            self.on_emergency_stop, callback_group=self.safety_group)
        self.create_service(Trigger, '/smart_vision_amr/reset_emergency_stop',
                            self.on_reset_emergency_stop, callback_group=self.safety_group)
        self.create_timer(.025, self.advance_reset, clock=self.steady_clock)
        self.create_timer(.05, self.publish_output, clock=self.steady_clock)
        self.create_timer(.1, self.publish_status, clock=self.steady_clock)
        self.publish_status()

    def on_auto(self, message):
        self.core.receive_auto(message.linear.x, message.angular.z)

    def on_navigation_control(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            self.core.reject_auto(None, 'Malformed autonomous JSON')
            self.publish_status()
            return
        # Do not accept a packet already in transit when this mux restarted.
        if (isinstance(payload, dict) and type(payload.get('sent_at_ns')) is int
                and payload['sent_at_ns'] < self.auto_started_ns):
            self.core.reject_auto(payload, 'Autonomous command predates mux startup')
            self.publish_status()
            return
        self.core.receive_stamped_auto(payload, self.get_clock().now().nanoseconds)
        self.publish_status()

    def on_localization_health(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        stamp = payload.get('sent_at_ns')
        if stamp is not None or self.core.require_localization_health:
            now_ns = self.get_clock().now().nanoseconds
            if (type(stamp) is not int or stamp < self._health_sent_ns
                    or stamp < 0 or stamp > now_ns+50_000_000
                    or now_ns-stamp > int(self.core.localization_health_timeout*1e9)):
                return
            self._health_sent_ns = stamp
        if self.core.receive_localization_health(payload):
            self.publish_output()
            self.publish_status()

    def _service_packet(self, mode):
        self.service_seq += 1
        return {'source_id': self.service_source, 'seq': self.service_seq,
                'sent_at_ns': self.get_clock().now().nanoseconds,
                'mode': mode, 'linear': 0.0, 'angular': 0.0}

    def on_emergency_stop(self, request, response):
        del request
        if self.pending_reset is not None:
            self.pending_reset['future'].set_result(False)
            self.pending_reset = None
        packet = self._service_packet('estop')
        self.core.receive_control(packet, self.get_clock().now().nanoseconds, origin='service')
        self.control_pub.publish(String(data=json.dumps(packet)))
        self.publish_output()
        self.publish_status(force=True)
        response.success = bool(self.output_available and self.core.estop_latched
                                and self.last_zero_published_at is not None
                                and time.monotonic()-self.last_zero_published_at < .1)
        response.message = ('Emergency stop latched; zero velocity published. '
                            'Physical standstill is not measured.' if response.success else
                            'Stop latched but exclusive /cmd_vel output is unavailable: '
                            + self.output_conflict)
        return response

    async def on_reset_emergency_stop(self, request, response):
        del request
        if self.pending_reset is not None:
            response.success, response.message = False, 'A reset is already pending'
            return response
        from rclpy.task import Future
        future = Future()
        packet = self._service_packet('reset')
        # Publish intent before changing state; ekf_sensor_guard correlates this
        # source with the subsequent unlatched motion_status and can re-latch.
        self.control_pub.publish(String(data=json.dumps(packet)))
        self.pending_reset = {'future': future, 'packet': packet,
                              'at': time.monotonic(), 'applied': False}
        accepted = await future
        response.success = bool(accepted)
        response.message = ('Emergency reset acknowledged; motion remains held. '
                            'Explicit resume required.' if accepted else
                            'Emergency reset rejected by safety or output ownership guard')
        return response

    def advance_reset(self):
        reset = self.pending_reset
        if reset is None:
            return
        age = time.monotonic()-reset['at']
        if not reset['applied'] and age >= .1:
            reset['applied'] = True
            packet = reset['packet']
            reset['accepted'] = self.core.receive_control(
                packet, self.get_clock().now().nanoseconds, origin='service')
            self.publish_output()
            self.publish_status(force=True)
        if age >= .5:
            factor, _, _, _ = self.core._localization_policy()
            accepted = (reset.get('accepted', False) and not self.core.estop_latched
                        and self.core.source_id == self.service_source
                        and self.output_available and factor > 0)
            if not accepted:
                self.core.estop_latched = True
                self.core.mode, self.core.reason = 'estop', 'Safety reset rejected'
                self.publish_output()
                self.publish_status()
            else:
                # Late manual/navigation packets may have arrived while guards
                # verified the reset. Completion always returns to explicit hold.
                self.core.mode, self.core.reason = 'hold', 'Safety reset confirmed; explicit resume required'
                self.core.manual_velocity, self.core.auto_at = (0.0, 0.0), None
            self.pending_reset = None
            self.publish_output()
            self.publish_status(force=True)
            reset['future'].set_result(accepted)

    def on_control(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            self.core.reject_control(None, 'Malformed control JSON')
            self.publish_status()
            return
        # Locally created service packets are published for guard observation,
        # never applied by DDS loopback ahead of the reset-intent grace period.
        if isinstance(payload, dict) and payload.get('source_id') == self.service_source:
            return
        if (isinstance(payload, dict) and payload.get('mode') == 'reset'
                and not self.allow_topic_reset):
            self.core.reject_control(payload, 'Topic reset is disabled; use reset_emergency_stop service')
            self.publish_status()
            return
        if self.core.receive_control(payload, self.get_clock().now().nanoseconds, origin='topic'):
            self.publish_output()
        self.publish_status()

    def publish_output(self):
        self.check_output_ownership()
        if not self.output_available:
            # Even a zero command would interfere with an existing, unmanaged
            # controller. Wait for an explicit exclusive handover instead.
            return
        message = Twist()
        # Guards briefly observe an unlatched state to acknowledge/veto the
        # explicit reset. Keep the motor output at zero throughout that window,
        # even if a delayed UI/navigation packet is accepted by the core.
        message.linear.x, message.angular.z = ((0.0, 0.0) if
            getattr(self, 'pending_reset', None) is not None else self.core.output())
        self.output_pub.publish(message)
        self.last_output_was_zero = message.linear.x == 0.0 and message.angular.z == 0.0
        if self.last_output_was_zero:
            self.last_zero_published_at = time.monotonic()

    def check_output_ownership(self):
        """Back off completely if any other publisher can command the robot."""
        now = time.monotonic()
        if now-self._ownership_checked_at < .2:
            return
        self._ownership_checked_at = now
        try:
            endpoints = self.get_publishers_info_by_topic(self.output_pub.topic_name)
            self.output_publishers = len(endpoints)
            own_endpoint = (len(endpoints) == 1
                            and endpoints[0].node_name == self.get_name()
                            and endpoints[0].node_namespace == self.get_namespace())
            self.output_available = own_endpoint
            if own_endpoint:
                self.output_conflict = ''
            elif not endpoints:
                self.output_conflict = 'Waiting for local /cmd_vel publisher discovery'
            else:
                names = [f'{item.node_namespace.rstrip("/")}/{item.node_name}'
                         for item in endpoints]
                self.output_conflict = ('Unmanaged /cmd_vel publisher: '
                                        + ', '.join(names))
        except Exception as exc:
            # Failure to inspect the graph must never imply exclusive access.
            self.output_available = False
            self.output_conflict = 'Cannot verify exclusive /cmd_vel ownership: '+str(exc)

    def publish_status(self, force=False):
        self.check_output_ownership()
        status = self.core.status()
        transition = (status['mode'], status['source_id'], status['estop_latched'],
                      status['reason'], self.output_available, self.output_conflict,
                      status['localization_speed_factor'], status['autonomous_allowed'],
                      getattr(self, 'pending_reset', None) is not None)
        now = time.monotonic()
        if (not force and transition == getattr(self, '_status_transition', None)
                and now-getattr(self, '_status_published_at', -math.inf) < .1):
            return False
        self._status_published_at, self._status_transition = now, transition
        self.status_sequence = getattr(self, 'status_sequence', 0)+1
        status['status_seq'] = self.status_sequence
        status['sent_at_ns'] = self.get_clock().now().nanoseconds
        status['mux_instance_id'] = getattr(self, 'service_source', '')
        status['available'] = self.output_available
        status['output_publishers'] = self.output_publishers
        status['stop_acknowledged'] = bool(
            self.output_available and getattr(self, 'last_output_was_zero', False)
            and (getattr(self, 'pending_reset', None) is not None
                 or self.core.output() == (0.0, 0.0))
            and getattr(self, 'last_zero_published_at', None) is not None
            and time.monotonic()-self.last_zero_published_at < .2)
        status['reset_pending'] = getattr(self, 'pending_reset', None) is not None
        if status['reset_pending']:
            status['output_linear'] = status['output_angular'] = 0.0
            status['output_owner'] = 'safety_reset'
        if not self.output_available:
            status['reason'] = self.output_conflict
        message = String()
        message.data = json.dumps(status, separators=(',', ':'))
        self.status_pub.publish(message)
        return True


def main(args=None):
    import signal
    from rclpy.signals import SignalHandlerOptions
    # Publish the final stop before invalidating the context on normal launch
    # shutdown. SIGKILL/power loss still requires the motor firmware watchdog.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MotionMuxNode()
    stop_requested = [False]

    def stop(_signum, _frame):
        stop_requested[0] = True

    previous = {signum: signal.signal(signum, stop)
                for signum in (signal.SIGINT, signal.SIGTERM)}
    try:
        while rclpy.ok() and not stop_requested[0]:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.core.mode = 'hold'
            node.core.estop_latched = True
            node.publish_output()
            time.sleep(.05)  # Give DDS's transport thread time to flush the stop.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
