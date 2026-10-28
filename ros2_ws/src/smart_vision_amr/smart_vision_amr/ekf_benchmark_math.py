"""Small, ROS-free geometry and statistics helpers for the EKF trial.

An odometry estimate starts in its own coordinate frame.  We align only the
first stationary sample with Gazebo truth and then measure *subsequent drift*.
That prevents a different arbitrary odom origin from counting as an error.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def align_pose(estimate, initial_estimate, initial_truth):
    """Express an odom pose in the fixed Gazebo world using the first pair."""
    rotation = wrap_angle(initial_truth.yaw - initial_estimate.yaw)
    dx = estimate.x - initial_estimate.x
    dy = estimate.y - initial_estimate.y
    return Pose2(
        initial_truth.x + math.cos(rotation) * dx - math.sin(rotation) * dy,
        initial_truth.y + math.sin(rotation) * dx + math.cos(rotation) * dy,
        wrap_angle(estimate.yaw + rotation),
    )


def errors(aligned_estimate, truth):
    return (math.hypot(aligned_estimate.x - truth.x,
                       aligned_estimate.y - truth.y),
            abs(wrap_angle(aligned_estimate.yaw - truth.yaw)))


def interpolate_pose(samples, target_ns):
    """Interpolate stamped SE(2) poses; never extrapolate outside the bracket.

    Returns ``(pose, longest_side_ns, bracket_span_ns)`` or ``None``.  The
    angular interpolation follows the shortest heading change across ±pi.
    """
    previous = None
    for stamp, pose in samples:
        if stamp == target_ns:
            return pose, 0, 0
        if stamp > target_ns:
            if previous is None:
                return None
            before_stamp, before = previous
            span = stamp - before_stamp
            if span <= 0:
                return None
            fraction = (target_ns - before_stamp) / span
            return (Pose2(before.x + fraction * (pose.x - before.x),
                          before.y + fraction * (pose.y - before.y),
                          wrap_angle(before.yaw
                                     + fraction * wrap_angle(pose.yaw - before.yaw))),
                    max(target_ns - before_stamp, stamp - target_ns), span)
        previous = (stamp, pose)
    return None


def rms(values):
    values = list(values)
    return math.sqrt(sum(float(value) ** 2 for value in values) / len(values)) if values else None


def percentile(values, fraction):
    """Linearly interpolated percentile for a short, finite trial series."""
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    index = max(0.0, min(1.0, fraction)) * (len(ordered) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def summarize_error_rows(rows, prefix=''):
    position = [row[prefix + 'position_error_m'] for row in rows
                if row.get(prefix + 'position_error_m') is not None]
    heading = [row[prefix + 'heading_error_rad'] for row in rows
               if row.get(prefix + 'heading_error_rad') is not None]
    return {
        'sample_count': min(len(position), len(heading)),
        'position_rmse_m': rms(position),
        'position_p95_m': percentile(position, .95),
        'heading_rmse_deg': math.degrees(rms(heading)) if heading else None,
        'heading_p95_deg': math.degrees(percentile(heading, .95)) if heading else None,
    }
