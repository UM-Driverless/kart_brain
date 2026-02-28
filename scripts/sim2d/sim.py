"""Simulation loop: perceive → control → step physics → check termination."""

import numpy as np

from track import (BLUE_CONES, YELLOW_CONES, ORANGE_CONES,
                   TRACK_LENGTH, SPAWN_X, SPAWN_Y, SPAWN_YAW,
                   project_to_centerline)
from kart_model import KartState, step as kart_step, DT
from perception import perceive

MAX_STEPS = 2000
OFF_TRACK_THRESHOLD = 2.0  # m from centerline (track is 1.5m half-width)
HALF_TRACK_WIDTH = 1.5     # m — same as in controllers.py


def run_episode(controller, max_steps=MAX_STEPS, fitness_mode="v1"):
    """Run one full episode.

    Parameters
    ----------
    fitness_mode : "v1" | "v2" | "v3"
        v1: distance + lap bonus - CTE + speed (original)
        v2: lap-time based — completing laps fast is king
        v3: track-keeping first — nonlinear CTE⁴ penalty, reward progress,
            speed only matters through completing laps

    Returns
    -------
    dict with keys: fitness, distance, laps, avg_cte, avg_speed, steps, time,
                    max_cte, cte_penalty
    """
    state = KartState(SPAWN_X, SPAWN_Y, SPAWN_YAW, speed=0.0)
    controller.reset()

    # Check if controller accepts current_speed (v2)
    wants_speed = hasattr(controller, '_last_speed')

    s_prev, _ = project_to_centerline(state.x, state.y)
    total_distance = 0.0
    total_cte = 0.0
    total_cte4 = 0.0   # sum of (cte / half_track)^4 per step
    max_cte = 0.0
    total_speed = 0.0
    steps = 0

    for _ in range(max_steps):
        visible = perceive(state, BLUE_CONES, YELLOW_CONES, ORANGE_CONES)

        if wants_speed:
            steer, speed_cmd = controller.control(visible,
                                                  current_speed=state.speed)
        else:
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
        if cte > max_cte:
            max_cte = cte

        # Nonlinear: (cte / half_track)^4 — explodes when approaching edges
        cte_norm = cte / HALF_TRACK_WIDTH
        total_cte4 += cte_norm ** 4

        if cte > OFF_TRACK_THRESHOLD:
            break

    avg_cte = total_cte / steps if steps else 0.0
    avg_speed = total_speed / steps if steps else 0.0
    avg_cte4 = total_cte4 / steps if steps else 0.0
    laps = int(total_distance / TRACK_LENGTH) if total_distance > 0 else 0
    time = steps * DT

    if fitness_mode == "v3":
        # Primary: progress along track
        # Penalty: TOTAL (not avg) cte^4 scaled per step — accumulates fast
        #   At cte=0.75m (half of half-width): 0.0625 per step
        #   At cte=1.5m  (at cones):           1.0 per step
        #   At cte=2.0m  (just outside):        3.16 per step
        # Over 2000 steps, even small deviations add up significantly
        cte_penalty = 0.5 * total_cte4  # total, not avg — punishes duration too
        # Extra cliff penalty if kart ever exceeded cone boundary
        if max_cte > HALF_TRACK_WIDTH:
            cte_penalty += 200.0 * ((max_cte / HALF_TRACK_WIDTH) ** 2)
        fitness = total_distance + 200.0 * laps - cte_penalty
    elif fitness_mode == "v2":
        if laps >= 1:
            fitness = 500.0 * laps - time - 5.0 * avg_cte
        else:
            fitness = total_distance - 2.0 * avg_cte
    else:
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
        "time": time,
        "max_cte": max_cte,
        "cte_penalty": cte_penalty if fitness_mode == "v3" else 0.0,
    }
