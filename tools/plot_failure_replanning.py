#!/usr/bin/env python3
"""Plot saved unseen-obstacle trial records without connecting to ROS/Gazebo.

The plot uses estimated map-frame paths/poses, never Gazebo ground-truth pose.
The injected obstacle geometry is known from its SpawnEntity request. Missing
trajectory samples are omitted, rather than reconstructed from planned paths.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def finite_points(value):
    if not isinstance(value, (list, tuple)):
        return []
    result = []
    for point in value:
        if (isinstance(point, (list, tuple)) and len(point) >= 2
                and all(isinstance(v, (float, int)) and not isinstance(v, bool)
                        and math.isfinite(v) for v in point[:2])):
            result.append((float(point[0]), float(point[1])))
    return result


def plot(trial_file: Path, map_file: Path, output: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.transforms as transforms
    from matplotlib.patches import Rectangle
    import numpy as np
    from PIL import Image
    import yaml

    trial = json.loads(trial_file.read_text(encoding='utf-8'))
    records = trial.get('path_records') or []
    before_records = [r for r in records if not r.get('after_obstacle') and finite_points(r.get('points'))]
    after_records = [r for r in records if r.get('after_obstacle') and finite_points(r.get('points'))]
    if not before_records or not after_records:
        raise ValueError('Trial must contain recorded original and replanned paths; counts alone cannot be plotted')
    original = finite_points(before_records[-1]['points'])
    final = finite_points(after_records[-1]['points'])
    removed = trial.get('obstacle_removed') is True
    resumed_path = removed and after_records[-1].get('after_removal') is True
    metadata = yaml.safe_load(map_file.read_text(encoding='utf-8'))
    image_path = Path(metadata['image'])
    if not image_path.is_absolute():
        image_path = map_file.parent / image_path
    occupancy = np.flipud(np.asarray(Image.open(image_path).convert('L')))
    height, width = occupancy.shape
    resolution = float(metadata['resolution'])
    ox, oy, yaw = map(float, metadata['origin'])
    if not all(math.isfinite(v) for v in (resolution, ox, oy, yaw)) or resolution <= 0:
        raise ValueError('Map geometry must be finite and resolution positive')
    box = trial['obstacle']
    bx, by = map(float, box['xy'])
    sx, sy = map(float, box['size'][:2])
    inflation = float(trial.get('actual_obstacle_inflation_m', .30))
    if not all(math.isfinite(v) and v >= 0 for v in (sx, sy, inflation)):
        raise ValueError('Obstacle geometry must be finite and nonnegative')
    goal = finite_points([trial.get('goal')])
    final_pose = finite_points([trial.get('final_map_xy')])
    # New trial traces contain [elapsed_seconds, map_x, map_y]. Never plot
    # the clock as x or confuse untransformed odometry with the map frame.
    timed_trace = trial.get('estimated_map_xy_trace')
    trace = finite_points([point[1:3] for point in timed_trace
                           if isinstance(point, (list, tuple)) and len(point) >= 3]
                          if isinstance(timed_trace, list) else
                          trial.get('map_xy_trace') or trial.get('estimated_map_trace'))
    events = trial.get('execution_failure_events') or []
    centers = []
    for event in events:
        try:
            heading = float(event['segment']['heading'])
            center = (float(event['pose']['x'])+.20*math.cos(heading),
                      float(event['pose']['y'])+.20*math.sin(heading))
            if all(math.isfinite(v) for v in center):
                centers.append(center)
        except (KeyError, TypeError, ValueError):
            pass

    fig, axes = plt.subplots(1, 2, figsize=(12.8, 7.1), gridspec_kw={'width_ratios': [1, 1.2]})
    outcome = 'PASS' if trial.get('passed') is True else 'FAIL'
    title = ('Obstacle hold, confirmed removal, and automatic resume'
             if removed else 'Unseen obstacle: recorded path replanning')
    fig.suptitle(f'{title} — {outcome}', fontsize=15, fontweight='bold', y=.97)
    plt.rcParams.update({'font.size': 10})

    for ax in axes:
        transform = transforms.Affine2D().rotate(yaw).translate(ox, oy) + ax.transData
        ax.imshow(occupancy, origin='lower', cmap='gray', vmin=0, vmax=255,
                  extent=(0, width*resolution, 0, height*resolution),
                  interpolation='nearest', transform=transform, zorder=0, alpha=.88)
        ax.add_patch(Rectangle((bx-sx/2-inflation, by-sy/2-inflation),
                               sx+2*inflation, sy+2*inflation,
                               facecolor='none' if removed else '#f4b5aa', edgecolor='#be4b3a', alpha=.36,
                               linewidth=1.2, linestyle='--', zorder=2,
                               label=f'Former planning margin ({inflation:.2f} m)' if removed else
                               f'Obstacle + planning margin ({inflation:.2f} m)'))
        ax.add_patch(Rectangle((bx-sx/2, by-sy/2), sx, sy,
                               facecolor='none' if removed else '#b23221', edgecolor='#7c2018', alpha=.90,
                               linewidth=1.1, linestyle='--' if removed else '-', zorder=3,
                               label='Removed obstacle footprint (historical)' if removed else
                               'Injected physical obstacle'))
        ax.plot(*zip(*original), color='#3977b7', linestyle='--', linewidth=2.2,
                zorder=4, label='Original planned path')
        ax.plot(*zip(*final), color='#db8b13', linewidth=2.6,
                zorder=5, label='Final path after confirmed removal' if resumed_path else
                'Last path before removal (not resumed)' if removed else 'Last regenerated path')
        if trace:
            ax.plot(*zip(*trace), color='#24714b', linewidth=1.6, alpha=.9,
                    zorder=6, label='Estimated map TF trajectory')
        ax.scatter([original[0][0]], [original[0][1]], color='#273746', marker='o',
                   s=44, edgecolor='white', linewidth=.8, zorder=7, label='Initial path start')
        if goal:
            ax.scatter(*zip(*goal), color='#24714b', marker='*', s=180,
                       edgecolor='white', linewidth=.7, zorder=8, label='Retained goal')
        if final_pose:
            ax.scatter(*zip(*final_pose), color='#7b328c', marker='D', s=56,
                       edgecolor='white', linewidth=.8, zorder=8, label='Final estimated robot pose')
        if centers:
            ax.scatter(*zip(*centers), color='#b2182b', marker='x', s=60,
                       linewidth=1.5, zorder=8, label='Recorded failed-approach centers')
        ax.set_aspect('equal')
        ax.set_xlabel('Map x (m)')
        ax.set_ylabel('Map y (m)')
        ax.grid(alpha=.18, color='#59636c', linewidth=.6)
        ax.set_facecolor('#f4f4f1')
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)

    # Full map bounds, including maps with a rotated origin.
    corners = [(ox+math.cos(yaw)*x-math.sin(yaw)*y,
                oy+math.sin(yaw)*x+math.cos(yaw)*y)
               for x, y in ((0, 0), (width*resolution, 0),
                             (0, height*resolution), (width*resolution, height*resolution))]
    axes[0].set_xlim(min(x for x, y in corners), max(x for x, y in corners))
    axes[0].set_ylim(min(y for x, y in corners), max(y for x, y in corners))
    axes[0].set_title('Saved occupancy map and mission context', fontsize=11)
    relevant = original + final + final_pose + goal + [(bx-sx/2-inflation, by-sy/2-inflation),
                                                      (bx+sx/2+inflation, by+sy/2+inflation)]
    axes[1].set_xlim(min(x for x, y in relevant)-.30, max(x for x, y in relevant)+.30)
    axes[1].set_ylim(min(y for x, y in relevant)-.35, max(y for x, y in relevant)+.35)
    axes[1].set_title('Obstacle approach and replacement route', fontsize=11)
    handles, labels = axes[1].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(.5, .075), ncol=4,
               fontsize=8, frameon=False, columnspacing=1.5, handlelength=2.5)
    last = after_records[-1]
    raw = last.get('raw_intersection_cells', 'not recorded')
    inflated = last.get('inflated_intersection_cells', 'not recorded')
    distance = trial.get('estimated_goal_distance_m')
    distance_text = f'{distance:.3f} m' if isinstance(distance, (int, float)) and math.isfinite(distance) else 'not recorded'
    overlap_label = 'Former obstacle area crossings' if resumed_path else 'Last path intersections'
    annotation = (f'Regenerated paths: {len(after_records)}    Failure events: {len(events)}\n'
                  f'{overlap_label}: physical={raw}, inflated={inflated}\n'
                  f'Final estimated goal error: {distance_text}')
    if removed:
        hold = trial.get('hold_state_snapshot') or {}
        delay = trial.get('removal_to_motion_seconds')
        delay_text = f'{delay:.2f} s' if isinstance(delay, (int, float)) and math.isfinite(delay) else 'not recorded'
        annotation += (f'\nRetry budget at hold: {hold.get("retry_count", "?")}/{hold.get("retry_limit", "?")}'
                       f'    Removal → motion: {delay_text}')
    axes[0].text(.025, .025, annotation, transform=axes[0].transAxes, fontsize=8.5,
                 va='bottom', bbox={'boxstyle': 'round,pad=.45', 'facecolor': 'white',
                                   'edgecolor': '#c9c9c5', 'alpha': .95})
    trace_note = 'Recorded estimated TF trajectory shown.' if trace else 'No trajectory samples recorded; only the final estimated pose is shown.'
    phase_note = '\nDashed obstacle outlines show its former location after physical removal.' if removed else ''
    fig.text(.03, .027, f'Paths/poses use estimated map TF, not Gazebo ground truth. {trace_note}{phase_note}',
             fontsize=8, color='#4d555b')
    fig.subplots_adjust(left=.06, right=.98, top=.89, bottom=.25, wspace=.20)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, facecolor='white')
    plt.close(fig)
    return output


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--map', type=Path, default=root/'ros2_ws/src/smart_vision_amr/maps/simulation_room.yaml')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(plot(args.trial, args.map, args.output))


if __name__ == '__main__':
    main()
