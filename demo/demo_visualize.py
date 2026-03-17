#!/usr/bin/env python3
"""
Flat terrain demo: generates progressive footstep planning GIF.
Uses real planner data exported by demo_export.cpp.
"""
import os
import sys
import csv
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon
    from matplotlib.lines import Line2D
except ImportError:
    os.system(f"{sys.executable} -m pip install --break-system-packages matplotlib")
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon
    from matplotlib.lines import Line2D


def load_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def main():
    # Load planner data
    footsteps_raw = load_csv("demo/footsteps.csv")
    body_path_raw = load_csv("demo/body_path.csv")
    foot_poly_raw = load_csv("demo/foot_polygons.csv")
    sg_raw = load_csv("demo/start_goal.csv")

    # Parse footsteps
    footsteps = []
    for row in footsteps_raw:
        footsteps.append({
            'x': float(row['x']),
            'y': float(row['y']),
            'yaw': float(row['yaw']),
            'side': row['side'],
            'step': int(row['step']),
        })
    n_steps = len(footsteps)
    print(f"Loaded {n_steps} footsteps")

    # Parse body path
    body_x = [float(r['x']) for r in body_path_raw]
    body_y = [float(r['y']) for r in body_path_raw]

    # Parse foot polygons (group by step)
    foot_poly = {}
    for row in foot_poly_raw:
        sid = int(row['step'])
        if sid not in foot_poly:
            foot_poly[sid] = {'x': [], 'y': []}
        foot_poly[sid]['x'].append(float(row['x']))
        foot_poly[sid]['y'].append(float(row['y']))

    # Load obstacle polygon (optional)
    obstacle_x, obstacle_y = [], []
    if os.path.exists("demo/obstacle.csv"):
        with open("demo/obstacle.csv") as f:
            for row in csv.DictReader(f):
                obstacle_x.append(float(row["x"]))
                obstacle_y.append(float(row["y"]))
        print(f"Loaded obstacle polygon ({len(obstacle_x)} vertices)")

    # Parse start/goal
    start_x, start_y, start_yaw = 0, 0, 0
    goal_x, goal_y, goal_yaw = 0, 0, 0
    for row in sg_raw:
        if row['pose'] == 'start':
            start_x, start_y, start_yaw = float(row['x']), float(row['y']), float(row['yaw'])
        else:
            goal_x, goal_y, goal_yaw = float(row['x']), float(row['y']), float(row['yaw'])

    # Colors
    COLOR_L = '#e74c3c'   # red
    COLOR_R = '#f39c12'   # orange/amber
    COLOR_BODY = '#3498db'
    COLOR_START = '#2ecc71'
    COLOR_GOAL = '#e74c3c'

    outDir = "demo/frames_flat"
    os.makedirs(outDir, exist_ok=True)

    # Determine axis limits from data
    all_x = body_x + [s['x'] for s in footsteps] + [start_x, goal_x]
    all_y = body_y + [s['y'] for s in footsteps] + [start_y, goal_y]
    margin = 0.15
    x_min, x_max = min(all_x) - margin, max(all_x) + margin
    y_min, y_max = min(all_y) - margin, max(all_y) + margin

    total_frames = n_steps + 1  # frame 0 = no footsteps, then add one per frame

    for frame in range(total_frames):
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        ax.set_aspect('equal')
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_axis_off()

        # Draw obstacle polygon
        if obstacle_x and obstacle_y:
            obs_patch = MplPolygon(list(zip(obstacle_x, obstacle_y)),
                                  closed=True, facecolor="#2c3e50", edgecolor="#c0392b",
                                  alpha=0.4, linewidth=2, zorder=2.5, label="Obstacle")
            ax.add_patch(obs_patch)
            obs_cx, obs_cy = sum(obstacle_x)/len(obstacle_x), sum(obstacle_y)/len(obstacle_y)
            ax.text(obs_cx, obs_cy, "✕", fontsize=14, color="white",
                    ha="center", va="center", fontweight="bold", zorder=3)

        # Draw body path (ellipsoid)
        ax.plot(body_x, body_y, color=COLOR_BODY, linewidth=1.5, alpha=0.6, linestyle='-',
                label='Body path (ellipsoid)', zorder=1)

        # Draw body path direction arrows
        arrow_step = max(1, len(body_path_raw) // 8)
        bw = 0.015
        for i in range(0, len(body_path_raw), arrow_step):
            bx = float(body_path_raw[i]['x'])
            by = float(body_path_raw[i]['y'])
            byaw = float(body_path_raw[i]['yaw'])
            ax.plot([bx, bx + np.cos(byaw)*bw],
                    [by, by + np.sin(byaw)*bw],
                    color=COLOR_BODY, lw=1.8, alpha=0.7, zorder=2)
            ax.plot(bx + np.cos(byaw)*bw, by + np.sin(byaw)*bw, '>',
                    color=COLOR_BODY, markersize=5, alpha=0.7, zorder=2)

        # Draw footsteps up to current frame
        for i in range(frame):
            s = footsteps[i]
            color = COLOR_L if s['side'] == 'L' else COLOR_R

            # Draw foot polygon
            if i in foot_poly:
                poly = MplPolygon(list(zip(foot_poly[i]['x'], foot_poly[i]['y'])),
                                  closed=True, facecolor=color, edgecolor='white',
                                  alpha=0.65, linewidth=1.2, zorder=3)
                ax.add_patch(poly)

            # Step number (offset from foot center)
            offset_y = 0.035 if s['side'] == 'L' else -0.035
            ax.text(s['x'] + 0.025, s['y'] + offset_y, str(i), fontsize=8,
                    color='#2c3e50', fontweight='bold', zorder=4)

        # Draw start/goal markers LAST (on top) to prevent jumping
        ax.plot(start_x, start_y, 's', color=COLOR_START, markersize=12, zorder=6)
        ax.annotate('', xy=(start_x + np.cos(start_yaw)*0.04, start_y + np.sin(start_yaw)*0.04),
                    xytext=(start_x, start_y),
                    arrowprops=dict(arrowstyle='->', color=COLOR_START, lw=3),
                    zorder=6)
        ax.text(start_x, start_y + 0.06, 'Start', fontsize=10, color=COLOR_START,
                fontweight='bold', ha='center', zorder=6,
                bbox=dict(boxstyle='round,pad=0.15', facecolor='white', edgecolor='none', alpha=0.85))

        ax.plot(goal_x, goal_y, 'D', color=COLOR_GOAL, markersize=12, zorder=6)
        ax.annotate('', xy=(goal_x + np.cos(goal_yaw)*0.04, goal_y + np.sin(goal_yaw)*0.04),
                    xytext=(goal_x, goal_y),
                    arrowprops=dict(arrowstyle='->', color=COLOR_GOAL, lw=3),
                    zorder=6)
        ax.text(goal_x, goal_y + 0.07, 'Goal', fontsize=10, color=COLOR_GOAL,
                fontweight='bold', ha='center', zorder=6,
                bbox=dict(boxstyle='round,pad=0.15', facecolor='white', edgecolor='none', alpha=0.85))

        # Title
        ax.set_title('A* Footstep Planning — Flat Terrain', fontsize=14, fontweight='bold', pad=10)

        # Legend
        legend_elements = [
            Line2D([0], [0], color=COLOR_BODY, linewidth=1.5, label='Body path (ellipsoid)'),
            Line2D([0], [0], color=COLOR_L, linewidth=2, label='Left foot'),
            Line2D([0], [0], color=COLOR_R, linewidth=2, label='Right foot'),
            Line2D([0], [0], color="#c0392b", linewidth=2, label="Obstacle", fill=True, alpha=0.4),
        ]
        ax.legend(handles=legend_elements, loc='lower left', fontsize=9, framealpha=0.9)

        # Step counter
        step_label = f"{frame} / {n_steps}" if frame > 0 else f"0 / {n_steps}"
        ax.text(0.98, 0.02, f"Step {step_label}", transform=ax.transAxes,
                fontsize=11, color='#7f8c8d', fontfamily='monospace',
                ha='right', va='bottom')

        fname = f"{outDir}/frame_{frame:03d}.png"
        plt.savefig(fname, dpi=100, bbox_inches='tight', facecolor='white')
        plt.close()

        if frame % 3 == 0 or frame == total_frames - 1:
            print(f"  Frame {frame}/{total_frames - 1}")

    # Generate GIF
    from PIL import Image
    frames = sorted([f for f in os.listdir(outDir) if f.endswith('.png')])
    images = [Image.open(os.path.join(outDir, f)) for f in frames]

    durations = [300] * (len(images) - 1) + [2000]
    gif_path = "demo/flat_terrain.gif"
    images[0].save(gif_path, save_all=True, append_images=images[1:],
                   duration=durations, loop=0, optimize=True)

    size_kb = os.path.getsize(gif_path) / 1024
    print(f"\nGIF saved: {gif_path} ({size_kb:.0f} KB)")


if __name__ == "__main__":
    print("=== A* Footstep Planner - Flat Terrain Demo (Real Planner Data) ===")
    main()
