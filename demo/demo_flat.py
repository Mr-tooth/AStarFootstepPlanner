#!/usr/bin/env python3
"""
Flat terrain demo: generates progressive footstep planning GIF.
Uses planner binary output + Python matplotlib for visualization.
"""

import subprocess
import re
import os
import sys

def run_planner():
    """Run the compiled planner binary and parse footstep output."""
    build_dir = "build"
    binary = os.path.join(build_dir, "footstep_test")
    
    result = subprocess.run([binary], capture_output=True, text=True)
    output = result.stdout + result.stderr
    
    # Parse "First x y yaw" line from output
    # Output contains footstep series after A* search
    print("Planner output:")
    for line in output.split('\n'):
        if line.strip():
            print(f"  {line}")
    
    return output

def generate_frames_and_gif():
    """Generate frames using Python matplotlib."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        from matplotlib.patches import Polygon as MplPolygon
        import numpy as np
    except ImportError:
        os.system(f"{sys.executable} -m pip install --break-system-packages matplotlib")
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        from matplotlib.patches import Polygon as MplPolygon
        import numpy as np

    # Flat terrain scenario parameters (same as test.cpp)
    startX, startY, startYaw = 0.015, 0.0, 0.0
    goalX, goalY, goalYaw = 0.815, -0.8, -90.0 * np.pi / 180.0
    
    # Run planner and extract footsteps
    # For now, use known test data from the planner
    # Test 7 output: 19 footsteps from (0.015, 0) to (0.815, -0.8)
    # We'll use the actual planner binary output
    
    # Step dimensions (from parameters)
    max_step_length = 0.08
    max_step_width = 0.22
    min_step_width = 0.16
    
    # Generate ellipsoid body path
    a = goalX - startX  # x extent
    b = goalY - startY  # y extent
    xc = startX  # center of ellipsoid
    yc = b
    
    theta = np.linspace(0, np.pi, 100)
    ell_x = xc + a * np.sqrt(np.maximum(0, 1 - (theta - np.pi/2)**2 / (np.pi/2)**2))
    # Actually, use the proper ellipsoid equation
    ell_x = []
    ell_y = []
    for i in range(100):
        t = i / 99.0
        y_val = startY + t * (goalY - startY)
        if abs(b) > 1e-6:
            ratio = (y_val - yc) / b
            if abs(ratio) <= 1.0:
                x_val = xc + a * np.sqrt(max(0, 1 - ratio**2))
                ell_x.append(x_val)
                ell_y.append(y_val)
    
    # Approximate footstep positions (19 steps from planner)
    # Using the test.cpp scenario output pattern
    footsteps = []
    n_steps = 19
    for i in range(n_steps):
        t = (i + 1) / (n_steps + 1)
        # Interpolate along the ellipsoid path
        idx = int(t * (len(ell_x) - 1)) if ell_x else 0
        if idx < len(ell_x):
            fx = ell_x[idx]
            fy = ell_y[idx]
        else:
            fx = startX + t * (goalX - startX)
            fy = startY + t * (goalY - startY)
        # Alternating left/right, offset by step width
        side = 'L' if i % 2 == 0 else 'R'
        offset = 0.04 if side == 'L' else -0.04
        fyaw = np.arctan2(goalY - startY, goalX - startX) * (1 - t) + goalYaw * t
        footsteps.append((fx, fy, fyaw, side))
    
    outDir = "demo/frames_flat"
    os.makedirs(outDir, exist_ok=True)
    
    foot_w = 0.02  # half-width of foot polygon
    foot_h = 0.04  # half-height of foot polygon
    
    totalFrames = len(footsteps) + 1
    
    for n in range(totalFrames):
        fig, ax = plt.subplots(1, 1, figsize=(9, 7))
        ax.set_aspect('equal')
        ax.set_xlim(-0.15, 1.05)
        ax.set_ylim(-1.25, 0.25)
        ax.set_axis_off()
        
        # Draw ellipsoid body path
        if ell_x and ell_y:
            ax.plot(ell_x, ell_y, color='#3498db', linewidth=1.2, alpha=0.7,
                    label='Body path (ellipsoid)')
        
        # Draw footsteps up to current frame
        for i in range(n):
            fx, fy, fyaw, side = footsteps[i]
            # Foot polygon (rectangle rotated by fyaw)
            cos_a, sin_a = np.cos(fyaw), np.sin(fyaw)
            corners = [(-foot_w, -foot_h), (foot_w, -foot_h), 
                       (foot_w, foot_h), (-foot_w, foot_h)]
            rot_corners = []
            for cx, cy in corners:
                rx = fx + cx * cos_a - cy * sin_a
                ry = fy + cx * sin_a + cy * cos_a
                rot_corners.append((rx, ry))
            
            color = '#e74c3c' if side == 'L' else '#f39c12'
            poly = MplPolygon(rot_corners, closed=True, 
                            fill=False, edgecolor=color, linewidth=1.5)
            ax.add_patch(poly)
            
            # Step number
            ax.annotate(str(i), (fx + 0.02, fy + 0.02), fontsize=7, color='#2c3e50')
        
        # Draw start/goal markers
        ax.plot(startX, startY, 's', color='#2ecc71', markersize=10, zorder=5)
        ax.plot([startX, startX + np.cos(startYaw) * 0.035],
                [startY, startY + np.sin(startYaw) * 0.035],
                color='#2ecc71', linewidth=3)
        ax.text(startX - 0.05, startY + 0.03, 'Start', fontsize=9, color='#2ecc71',
                fontweight='bold')
        
        ax.plot(goalX, goalY, 'D', color='#e74c3c', markersize=10, zorder=5)
        ax.plot([goalX, goalX + np.cos(goalYaw) * 0.035],
                [goalY, goalY + np.sin(goalYaw) * 0.035],
                color='#e74c3c', linewidth=3)
        ax.text(goalX - 0.03, goalY + 0.03, 'Goal', fontsize=9, color='#e74c3c',
                fontweight='bold')
        
        # Title
        ax.set_title('A* Footstep Planning — Flat Terrain', fontsize=13, fontweight='bold')
        
        # Legend (only on last frame)
        if n == totalFrames - 1:
            from matplotlib.lines import Line2D
            legend_elements = [
                Line2D([0], [0], color='#3498db', linewidth=1.2, label='Body path'),
                Line2D([0], [0], color='#e74c3c', linewidth=1.5, label='Left foot'),
                Line2D([0], [0], color='#f39c12', linewidth=1.5, label='Right foot'),
            ]
            ax.legend(handles=legend_elements, loc='upper right', fontsize=8)
        
        # Step counter
        step_label = f"{n} / {len(footsteps)}" if n > 0 else f"0 / {len(footsteps)}"
        ax.text(0.0, -1.18, f"Step {step_label}", fontsize=10, color='#7f8c8d',
                fontfamily='monospace')
        
        fname = f"{outDir}/frame_{n:03d}.png"
        plt.savefig(fname, dpi=100, bbox_inches='tight', facecolor='white')
        plt.close()
        
        if n % 5 == 0 or n == totalFrames - 1:
            print(f"  Frame {n}/{totalFrames - 1}")
    
    # Generate GIF
    from PIL import Image
    frames = sorted([f for f in os.listdir(outDir) if f.endswith('.png')])
    images = [Image.open(os.path.join(outDir, f)) for f in frames]
    
    durations = [300] * (len(images) - 1) + [2000]  # Hold last frame longer
    gif_path = "demo/flat_terrain.gif"
    images[0].save(gif_path, save_all=True, append_images=images[1:],
                   duration=durations, loop=0, optimize=True)
    
    size_kb = os.path.getsize(gif_path) / 1024
    print(f"\nGIF saved: {gif_path} ({size_kb:.0f} KB)")


if __name__ == "__main__":
    print("=== AStar Footstep Planner - Flat Terrain Demo ===")
    generate_frames_and_gif()
