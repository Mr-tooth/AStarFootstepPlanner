#!/usr/bin/env python3
"""Stitch PNG frames into an animated GIF with progressive footstep visualization."""

import glob
import os
import sys

def make_gif(frame_dir, output_path, duration_per_frame=400, final_hold=2000):
    """Create GIF from frames, with longer hold on the last frame."""
    try:
        from PIL import Image
    except ImportError:
        print("Installing Pillow...")
        os.system(f"{sys.executable} -m pip install --break-system-packages Pillow")
        from PIL import Image

    frames = sorted(glob.glob(os.path.join(frame_dir, "frame_*.png")))
    if not frames:
        print(f"No frames found in {frame_dir}")
        sys.exit(1)

    print(f"Loading {len(frames)} frames...")
    images = [Image.open(f) for f in frames]

    # Hold the last frame longer
    durations = [duration_per_frame] * (len(images) - 1) + [final_hold]

    print(f"Saving GIF to {output_path}...")
    images[0].save(
        output_path,
        save_all=True,
        append_images=images[1:],
        duration=durations,
        loop=0,
        optimize=True,
    )
    size_kb = os.path.getsize(output_path) / 1024
    print(f"Done! {output_path} ({size_kb:.0f} KB)")


if __name__ == "__main__":
    frame_dir = sys.argv[1] if len(sys.argv) > 1 else "demo/frames_flat"
    output = sys.argv[2] if len(sys.argv) > 2 else "demo/flat_terrain.gif"
    make_gif(frame_dir, output)
