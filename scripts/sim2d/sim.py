"""Simulation loop: perceive → control → step physics → check termination."""

import numpy as np

from track import (BLUE_CONES, YELLOW_CONES, ORANGE_CONES,
                   TRACK_LENGTH, SPAWN_X, SPAWN_Y, SPAWN_YAW,
                   project_to_centerline)
from kart_model import KartState, step as kart_step, DT
from perception import perceive

MAX_STEPS = 2000
OFF_TRACK_THRESHOLD = 5.0  # m from centerline


def run_episode(controller, max_steps=MAX_STEPS):
    """Run one full episode.

    Returns
    -------
    dict with keys: fitness, distance, laps, avg_cte, avg_speed, steps, time
    """
    state = KartState(SPAWN_X, SPAWN_Y, SPAWN_YAW, speed=0.0)
    controller.reset()

    s_prev, _ = project_to_centerline(state.x, state.y)
    total_distance = 0.0
    total_cte = 0.0
    total_speed = 0.0
    steps = 0

    for _ in range(max_steps):
        visible = perceive(state, BLUE_CONES, YELLOW_CONES, ORANGE_CONES)
        steer, speed_cmd = controller.control(visible)
        state = kart_step(state, steer, speed_cmd)
        steps += 1

        s_cur, cte = project_to_centerline(state.x, state.y)

        # Incremental distance (handles wrap-around)
        ds = s_cur - s_prev
        if ds > TRACK_LENGTH / 2:
            ds -= TRACK_LENGTH
        elif ds < -TRACK_LENGTH / 2:
            ds += TRACK_LENGTH
        total_distance += ds
        s_prev = s_cur

        total_cte += cte
        total_speed += state.speed

        if cte > OFF_TRACK_THRESHOLD:
            break

    avg_cte = total_cte / steps if steps else 0.0
    avg_speed = total_speed / steps if steps else 0.0
    laps = int(total_distance / TRACK_LENGTH) if total_distance > 0 else 0

    fitness = (total_distance
               + 100.0 * laps
               - 2.0 * avg_cte
               + 0.5 * avg_speed)

    return {
        "fitness": fitness,
        "distance": total_distance,
        "laps": laps,
        "avg_cte": avg_cte,
        "avg_speed": avg_speed,
        "steps": steps,
        "time": steps * DT,
    }
