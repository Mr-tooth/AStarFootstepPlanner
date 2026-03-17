#!/usr/bin/env python3
"""
A* Footstep Planner demo visualization — supports flat terrain, obstacle, and stairs demos.
Uses real planner data exported by demo_export.cpp / demo_obstacle.cpp / demo_stairs.cpp.
"""
import os
import sys
import csv
import glob
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


def load_terrain_patches(pattern="demo/terrain_*.csv"):
    """Load all terrain_*.csv files and return list of (x_list, y_list) polygons."""
    patches = []
    files = sorted(glob.glob(pattern))
    for fpath in files:
        with open(fpath) as f:
            reader = csv.DictReader(f)
            x, y = [], []
            for row in reader:
                x.append(float(row['x']))
                y.append(float(row['y']))
            if len(x) >= 3:
                patches.append((x, y))
    return patches


def main(mode="flat"):
    """
    mode: "flat", "obstacle", or "stairs"
    """
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

    # Parse body path (may or may not have yaw column)
    body_x = [float(r['x']) for r in body_path_raw]
    body_y = [float(r['y']) for r in body_path_raw]
    if 'yaw' in body_path_raw[0]:
        body_yaw = [float(r['yaw']) for r in body_path_raw]
    else:
        # Compute yaw from consecutive points
        body_yaw = []
        for i in range(len(body_x)):
            if i < len(body_x) - 1:
                body_yaw.append(np.arctan2(body_y[i+1] - body_y[i], body_x[i+1] - body_x[i]))
            else:
                body_yaw.append(body_yaw[-1] if body_yaw else 0)

    # Parse foot polygons (group by step)
    foot_poly = {}
    for row in foot_poly_raw:
        sid = int(row['step'])
        if sid not in foot_poly:
            foot_poly[sid] = {'x': [], 'y': []}
        foot_poly[sid]['x'].append(float(row['x' if 'x' in row else f'x1']))
        foot_poly[sid]['y'].append(float(row['y' if 'y' in row else 'y1']))
        # For multi-column format (x1,y1,x2,y2,...)
        if 'x4' in row:
            foot_poly[sid]['x'] = [float(row[f'x{i}']) for i in range(1, 5)]
            foot_poly[sid]['y'] = [float(row[f'y{i}']) for i in range(1, 5)]

    # Load obstacle polygon (optional)
    obstacle_x, obstacle_y = [], []
    if os.path.exists("demo/obstacle.csv"):
        with open("demo/obstacle.csv") as f:
            for row in csv.DictReader(f):
                obstacle_x.append(float(row["x"]))
                obstacle_y.append(float(row["y"]))
        print(f"Loaded obstacle polygon ({len(obstacle_x)} vertices)")

    # Load terrain patches (for stairs demo)
    terrain_patches = load_terrain_patches()
    if terrain_patches:
        print(f"Loaded {len(terrain_patches)} terrain patches")

    # Parse start/goal (handle both 'pose' and 'type' column names)
    start_x, start_y, start_yaw = 0, 0, 0
    goal_x, goal_y, goal_yaw = 0, 0, 0
    for row in sg_raw:
        label = row.get('pose', row.get('type', ''))
        if label == 'start':
            start_x, start_y = float(row['x']), float(row['y'])
            start_yaw = float(row.get('yaw', 0))
        elif label == 'goal':
            goal_x, goal_y = float(row['x']), float(row['y'])
            goal_yaw = float(row.get('yaw', 0))

    # Colors
    COLOR_L = '#e74c3c'   # red
    COLOR_R = '#f39c12'   # orange/amber
    COLOR_BODY = '#3498db'
    COLOR_START = '#2ecc71'
    COLOR_GOAL = '#e74c3c'
    COLOR_TERRAIN = '#b8d4e3'    # light blue-grey
    COLOR_TERRAIN_EDGE = '#7f8c8d'

    outDir = "demo/frames_flat"
    os.makedirs(outDir, exist_ok=True)

    # Determine axis limits from data
    all_x = body_x + [s['x'] for s in footsteps] + [start_x, goal_x]
    all_y = body_y + [s['y'] for s in footsteps] + [start_y, goal_y]
    # Include terrain patches in axis limits
    for tx, ty in terrain_patches:
        all_x.extend(tx)
        all_y.extend(ty)
    margin = 0.15
    x_min, x_max = min(all_x) - margin, max(all_x) + margin
    y_min, y_max = min(all_y) - margin, max(all_y) + margin

    # Title per mode
    titles = {
        "flat": "A* Footstep Planning — Flat Terrain",
        "obstacle": "A* Footstep Planning — Obstacle Avoidance",
        "stairs": "A* Footstep Planning — Multi-Level Stair Climbing",
    }
    title = titles.get(mode, titles["flat"])

    total_frames = n_steps + 1  # frame 0 = no footsteps, then add one per frame

    for frame in range(total_frames):
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        ax.set_aspect('equal')
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_axis_off()

        # Draw terrain patches (stairs mode)
        for i, (tx, ty) in enumerate(terrain_patches):
            patch = MplPolygon(list(zip(tx, ty)),
                              closed=True, facecolor=COLOR_TERRAIN,
                              edgecolor=COLOR_TERRAIN_EDGE, alpha=0.45,
                              linewidth=1.5, zorder=1.5, linestyle='--')
            ax.add_patch(patch)

        # Draw obstacle polygon
        if obstacle_x and obstacle_y:
            obs_patch = MplPolygon(list(zip(obstacle_x, obstacle_y)),
                                  closed=True, facecolor="#2c3e50", edgecolor="#c0392b",
                                  alpha=0.4, linewidth=2, zorder=2.5, label="Obstacle")
            ax.add_patch(obs_patch)
            obs_cx, obs_cy = sum(obstacle_x)/len(obstacle_x), sum(obstacle_y)/len(obstacle_y)
            ax.text(obs_cx, obs_cy, "✕", fontsize=14, color="white",
                    ha="center", va="center", fontweight="bold", zorder=3)

        # Draw body path
        ax.plot(body_x, body_y, color=COLOR_BODY, linewidth=1.5, alpha=0.6, linestyle='-',
                label='Body path', zorder=1)

        # Draw body path direction arrows
        arrow_step = max(1, len(body_path_raw) // 8)
        bw = 0.015
        for i in range(0, len(body_path_raw), arrow_step):
            bx = float(body_path_raw[i]['x'])
            by = float(body_path_raw[i]['y'])
            byaw = body_yaw[i]
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
        ax.set_title(title, fontsize=14, fontweight='bold', pad=10)

        # Legend
        legend_elements = [
            Line2D([0], [0], color=COLOR_BODY, linewidth=1.5, label='Body path'),
            Line2D([0], [0], color=COLOR_L, linewidth=2, label='Left foot'),
            Line2D([0], [0], color=COLOR_R, linewidth=2, label='Right foot'),
        ]
        if terrain_patches:
            legend_elements.append(
                Line2D([0], [0], color=COLOR_TERRAIN_EDGE, linewidth=1.5,
                       linestyle='--', label='Terrain patches', fillstyle='bottom', alpha=0.45))
        if obstacle_x:
            legend_elements.append(
                Line2D([0], [0], color="#c0392b", linewidth=2, label="Obstacle", fill=True, alpha=0.4))
        ax.legend(handles=legend_elements, loc='lower left', fontsize=9, framealpha=0.9)

        # Step counter
        step_label = f"{frame} / {n_steps}" if frame > 0 else f"0 / {n_steps}"
        ax.text(0.98, 0.02, f"Step {step_label}", transform=ax.transAxes,
                fontsize=11, color='#7f8c8d', fontfamily='monospace',
                ha='right', va='bottom')

        fname = f"{outDir}/frame_{frame:03d}.png"
        plt.savefig(fname, dpi=100, bbox_inches='tight', facecolor='white')
        plt.close()

        if frame % 5 == 0 or frame == total_frames - 1:
            print(f"  Frame {frame}/{total_frames - 1}")

    # Generate GIF
    from PIL import Image
    frames = sorted([f for f in os.listdir(outDir) if f.endswith('.png')])
    images = [Image.open(os.path.join(outDir, f)) for f in frames]

    durations = [300] * (len(images) - 1) + [2000]
    gif_path = f"demo/{mode}.gif"
    images[0].save(gif_path, save_all=True, append_images=images[1:],
                   duration=durations, loop=0, optimize=True)

    size_kb = os.path.getsize(gif_path) / 1024
    print(f"\nGIF saved: {gif_path} ({size_kb:.0f} KB)")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "stairs"
    print(f"=== A* Footstep Planner - {mode.capitalize()} Demo (Real Planner Data) ===")
    main(mode)
