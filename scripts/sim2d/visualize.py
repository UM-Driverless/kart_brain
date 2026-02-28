#!/usr/bin/env python3
"""Replay the best run with a matplotlib plot (or animation)."""

import argparse
import json
import sys

import numpy as np
import matplotlib.pyplot as plt

from track import (BLUE_CONES, YELLOW_CONES, ORANGE_CONES, CENTERLINE_XY,
                   SPAWN_X, SPAWN_Y, SPAWN_YAW, project_to_centerline)
from kart_model import KartState, step as kart_step
from perception import perceive
from controllers import GeometricController, NeuralNetController

CONTROLLER_CLASSES = {
    "geometric": GeometricController,
    "neural": NeuralNetController,
}


def run_and_record(controller, max_steps=2000):
    """Run one episode, returning an Nx5 array (x, y, yaw, speed, steer)."""
    state = KartState(SPAWN_X, SPAWN_Y, SPAWN_YAW, speed=0.0)
    controller.reset()
    rows = [(state.x, state.y, state.yaw, state.speed, 0.0)]

    for _ in range(max_steps):
        visible = perceive(state, BLUE_CONES, YELLOW_CONES, ORANGE_CONES)
        steer, speed_cmd = controller.control(visible)
        state = kart_step(state, steer, speed_cmd)
        rows.append((state.x, state.y, state.yaw, state.speed, steer))

        _, cte = project_to_centerline(state.x, state.y)
        if cte > 5.0:
            break

    return np.array(rows)


def plot_trajectory(traj, title="2D Sim — Kart Trajectory"):
    fig, ax = plt.subplots(figsize=(10, 10))

    # Centerline
    ax.plot(CENTERLINE_XY[:, 0], CENTERLINE_XY[:, 1],
            "k--", alpha=0.3, lw=1, label="Centerline")

    # Cones
    ax.scatter(*BLUE_CONES.T, c="blue", s=30, marker="^", label="Blue")
    ax.scatter(*YELLOW_CONES.T, c="gold", s=30, marker="^", label="Yellow")
    ax.scatter(*ORANGE_CONES.T, c="orange", s=60, marker="^", label="Orange")

    # Trajectory coloured by speed
    sc = ax.scatter(traj[:, 0], traj[:, 1], c=traj[:, 3],
                    cmap="RdYlGn", s=2, vmin=0, vmax=5)
    plt.colorbar(sc, ax=ax, label="Speed (m/s)")

    ax.plot(traj[0, 0], traj[0, 1], "go", ms=10, label="Start")
    ax.plot(traj[-1, 0], traj[-1, 1], "ro", ms=10, label="End")

    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Visualise best controller")
    parser.add_argument("json_file", nargs="?", default=None,
                        help="Path to best_*.json")
    parser.add_argument("--default", action="store_true",
                        help="Run default geometric parameters")
    args = parser.parse_args()

    if args.default:
        ctrl = GeometricController(GeometricController.DEFAULTS)
        label = "Default geometric"
    elif args.json_file:
        with open(args.json_file) as f:
            data = json.load(f)
        cls = CONTROLLER_CLASSES[data["controller_type"]]
        ctrl = cls(np.array(data["genes"]))
        label = f"Best {data['controller_type']} (fitness {data['fitness']:.1f})"
    else:
        print("Usage: visualize.py <best.json>  or  visualize.py --default")
        sys.exit(1)

    print(f"Running episode: {label} ...")
    traj = run_and_record(ctrl)
    print(f"  {len(traj)} steps recorded")
    plot_trajectory(traj, title=label)


if __name__ == "__main__":
    main()
