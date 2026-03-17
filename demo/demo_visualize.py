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
        if 'heights' in fpath:
            continue
        with open(fpath) as f:
            reader = csv.DictReader(f)
            x, y = [], []
            for row in reader:
                x.append(float(row['x']))
                y.append(float(row['y']))
            if len(x) >= 3:
                patches.append((x, y))
    return patches


def load_terrain_heights(path="demo/terrain_heights.csv"):
    """Load height info. Returns list of dicts: {index, label, height_m}."""
    heights = []
    if not os.path.exists(path):
        return heights
    with open(path) as f:
        for row in csv.DictReader(f):
            heights.append({
                'index': int(row['index']),
                'label': row.get('label', ''),
                'height_m': float(row['height_m']),
            })
    return heights


def stone_color(height_m, max_h=0.15):
    """Blue-purple gradient for stepping stones, brighter = taller."""
    t = min(1.0, height_m / max_h)
    # low: deep blue (30,80,180) → high: bright cyan (60,200,220)
    r = int(30 + 30 * t)
    g = int(80 + 120 * t)
    b = int(180 + 40 * t)
    return f'#{r:02x}{g:02x}{b:02x}'


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

    # Parse body path
    body_x = [float(r['x']) for r in body_path_raw]
    body_y = [float(r['y']) for r in body_path_raw]
    if 'yaw' in body_path_raw[0]:
        body_yaw = [float(r['yaw']) for r in body_path_raw]
    else:
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
        # For multi-column format (x1,y1,x2,y2,...)
        if 'x4' in row:
            foot_poly[sid]['x'] = [float(row[f'x{i}']) for i in range(1, 5)]
            foot_poly[sid]['y'] = [float(row[f'y{i}']) for i in range(1, 5)]
        else:
            foot_poly[sid]['x'].append(float(row.get('x', row.get('x1', 0))))
            foot_poly[sid]['y'].append(float(row.get('y', row.get('y1', 0))))

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
    terrain_heights = load_terrain_heights()
    if terrain_patches:
        print(f"Loaded {len(terrain_patches)} terrain patches")
        if terrain_heights:
            for h in terrain_heights:
                print(f"    [{h['index']}] {h['label']}: {h['height_m']*100:.0f}cm")

    # Parse start/goal
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
    COLOR_L = '#e74c3c'       # red
    COLOR_R = '#f39c12'       # amber
    COLOR_BODY = '#3498db'
    COLOR_START = '#27ae60'
    COLOR_GOAL = '#c0392b'

    outDir = "demo/frames_flat"
    os.makedirs(outDir, exist_ok=True)

    # Axis limits
    all_x = body_x + [s['x'] for s in footsteps] + [start_x, goal_x]
    all_y = body_y + [s['y'] for s in footsteps] + [start_y, goal_y]
    for tx, ty in terrain_patches:
        all_x.extend(tx)
        all_y.extend(ty)
    margin = 0.20
    x_min, x_max = min(all_x) - margin, max(all_x) + margin
    y_min, y_max = min(all_y) - margin, max(all_y) + margin

    # Title per mode
    titles = {
        "flat": "A* Footstep Planning — Flat Terrain",
        "obstacle": "A* Footstep Planning — Obstacle Avoidance",
        "stairs": "A* Footstep Planning — Stepping Stones",
    }
    title = titles.get(mode, titles["flat"])

    total_frames = n_steps + 1  # frame 0 = no footsteps, then add one per frame

    for frame in range(total_frames):
        fig, ax = plt.subplots(1, 1, figsize=(12, 7))
        ax.set_aspect('equal')
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_axis_off()

        height_map = {h['index']: h['height_m'] for h in terrain_heights}
        label_map = {h['index']: h['label'] for h in terrain_heights}

        # --- Layer 1: Base plane (zorder=1) ---
        for i, (tx, ty) in enumerate(terrain_patches):
            label = label_map.get(i, '')
            if label == 'base':
                patch = MplPolygon(list(zip(tx, ty)),
                                  closed=True, facecolor='#f0f3f5',
                                  edgecolor='#b0b8c0', alpha=0.6,
                                  linewidth=1.5, linestyle='--', zorder=1)
                ax.add_patch(patch)

        # --- Layer 2: Stepping stones (zorder=2) ---
        stone_positions = []
        for i, (tx, ty) in enumerate(terrain_patches):
            label = label_map.get(i, '')
            if label == 'base':
                continue
            h = height_map.get(i, 0.0)
            color = stone_color(h)
            patch = MplPolygon(list(zip(tx, ty)),
                              closed=True, facecolor=color,
                              edgecolor='white', alpha=0.8,
                              linewidth=2.0, zorder=2)
            ax.add_patch(patch)
            cx = sum(tx) / len(tx)
            cy = sum(ty) / len(ty)
            stone_positions.append((cx, cy, h))

        # --- Layer 3: Obstacle (zorder=3) ---
        if obstacle_x and obstacle_y:
            obs_patch = MplPolygon(list(zip(obstacle_x, obstacle_y)),
                                  closed=True, facecolor="#2c3e50", edgecolor="#c0392b",
                                  alpha=0.5, linewidth=2, zorder=3)
            ax.add_patch(obs_patch)
            obs_cx = sum(obstacle_x) / len(obstacle_x)
            obs_cy = sum(obstacle_y) / len(obstacle_y)
            ax.text(obs_cx, obs_cy, "✕", fontsize=14, color="white",
                    ha="center", va="center", fontweight="bold", zorder=5)

        # --- Layer 4: Body path (zorder=4) — hidden in stairs mode ---
        if mode != "stairs":
            ax.plot(body_x, body_y, color=COLOR_BODY, linewidth=1.5, alpha=0.5, linestyle='-',
                    zorder=4)
            arrow_step = max(1, len(body_path_raw) // 8)
            bw = 0.015
            for i in range(0, len(body_path_raw), arrow_step):
                bx = float(body_path_raw[i]['x'])
                by = float(body_path_raw[i]['y'])
                byaw = body_yaw[i]
                ax.plot(bx + np.cos(byaw)*bw, by + np.sin(byaw)*bw, '>',
                        color=COLOR_BODY, markersize=5, alpha=0.6, zorder=4)

        # --- Layer 5: Footstep polygons (zorder=5) ---
        for i in range(frame):
            s = footsteps[i]
            color = COLOR_L if s['side'] == 'L' else COLOR_R
            if i in foot_poly:
                poly = MplPolygon(list(zip(foot_poly[i]['x'], foot_poly[i]['y'])),
                                  closed=True, facecolor=color, edgecolor='white',
                                  alpha=0.75, linewidth=1.5, zorder=5)
                ax.add_patch(poly)

        # --- Layer 6: Step numbers (zorder=6) ---
        for i in range(frame):
            s = footsteps[i]
            offset_y = 0.04 if s['side'] == 'L' else -0.04
            ax.text(s['x'] + 0.03, s['y'] + offset_y, str(i), fontsize=8,
                    color='#2c3e50', fontweight='bold', zorder=6,
                    bbox=dict(boxstyle='round,pad=0.1', facecolor='white',
                              edgecolor='none', alpha=0.8))

        # --- Layer 7: Stone height labels (zorder=7) ---
        for cx, cy, h in stone_positions:
            ax.text(cx, cy, f'{h*100:.0f}cm', fontsize=7,
                    color='white', fontweight='bold', ha='center', va='center', zorder=7,
                    bbox=dict(boxstyle='round,pad=0.15', facecolor='#000000',
                              edgecolor='none', alpha=0.35))

        # --- Layer 8: Start/Goal (zorder=8, always on top) ---
        ax.plot(start_x, start_y, 's', color=COLOR_START, markersize=14, zorder=8,
                markeredgecolor='white', markeredgewidth=1.5)
        ax.annotate('', xy=(start_x + np.cos(start_yaw)*0.06, start_y + np.sin(start_yaw)*0.06),
                    xytext=(start_x, start_y),
                    arrowprops=dict(arrowstyle='->', color=COLOR_START, lw=2.5),
                    zorder=8)
        ax.text(start_x, start_y + 0.08, 'Start', fontsize=11, color=COLOR_START,
                fontweight='bold', ha='center', zorder=8,
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', edgecolor=COLOR_START, alpha=0.9))

        ax.plot(goal_x, goal_y, 'D', color=COLOR_GOAL, markersize=14, zorder=8,
                markeredgecolor='white', markeredgewidth=1.5)
        ax.annotate('', xy=(goal_x + np.cos(goal_yaw)*0.06, goal_y + np.sin(goal_yaw)*0.06),
                    xytext=(goal_x, goal_y),
                    arrowprops=dict(arrowstyle='->', color=COLOR_GOAL, lw=2.5),
                    zorder=8)
        ax.text(goal_x, goal_y + 0.09, 'Goal', fontsize=11, color=COLOR_GOAL,
                fontweight='bold', ha='center', zorder=8,
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', edgecolor=COLOR_GOAL, alpha=0.9))

        # --- Title ---
        ax.set_title(title, fontsize=15, fontweight='bold', pad=12)

        # --- Legend ---
        legend_elements = [
            Line2D([0], [0], color=COLOR_L, linewidth=3, label='Left foot', solid_capstyle='round'),
            Line2D([0], [0], color=COLOR_R, linewidth=3, label='Right foot', solid_capstyle='round'),
        ]
        if mode != "stairs":
            legend_elements.insert(0, Line2D([0], [0], color=COLOR_BODY, linewidth=1.5, label='Body path'))
        if terrain_patches and mode == "stairs":
            # Height legend: 3 reference stones
            for ref_h, ref_label in [(0.05, '~5cm'), (0.10, '~10cm'), (0.14, '~14cm')]:
                c = stone_color(ref_h)
                legend_elements.append(
                    MplPolygon([[0,0],[1,0],[1,1],[0,1]], closed=True,
                              facecolor=c, edgecolor='white', linewidth=1,
                              label=f'Stone {ref_label}', alpha=0.8))
        if obstacle_x:
            legend_elements.append(
                Line2D([0], [0], color="#c0392b", linewidth=2, label="Obstacle", alpha=0.5))
        ax.legend(handles=legend_elements, loc='lower right', fontsize=9,
                  framealpha=0.95, edgecolor='#ddd')

        # --- Step counter ---
        step_label = f"{frame} / {n_steps}" if frame > 0 else f"0 / {n_steps}"
        ax.text(0.02, 0.98, f"Step {step_label}", transform=ax.transAxes,
                fontsize=11, color='#7f8c8d', fontfamily='monospace',
                ha='left', va='top')

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
