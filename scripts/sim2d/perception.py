"""Simulated perception: FOV + range filter, output in optical frame."""

import numpy as np
from kart_model import KartState

CAMERA_OFFSET = 0.55          # m forward of chassis center
FOV_HALF = np.radians(60)     # ±60° = 120° total
RANGE_MIN = 0.5               # m
RANGE_MAX = 15.0              # m


def perceive(state: KartState, blue_cones, yellow_cones, orange_cones):
    """Return cones visible from the kart's camera.

    Each entry is ``(class_id, opt_x, opt_y, opt_z)`` in the camera
    *optical* frame (Z = forward, X = right, Y = down) — the same
    convention used by the real perception pipeline.
    """
    cos_yaw = np.cos(state.yaw)
    sin_yaw = np.sin(state.yaw)

    cam_x = state.x + CAMERA_OFFSET * cos_yaw
    cam_y = state.y + CAMERA_OFFSET * sin_yaw

    visible = []

    for class_id, cones in (("blue_cone", blue_cones),
                             ("yellow_cone", yellow_cones),
                             ("orange_cone", orange_cones)):
        if len(cones) == 0:
            continue
        # Vectorised relative position
        dx = cones[:, 0] - cam_x
        dy = cones[:, 1] - cam_y

        # Kart frame: fwd / left
        fwd = dx * cos_yaw + dy * sin_yaw
        left = -dx * sin_yaw + dy * cos_yaw

        dist = np.sqrt(fwd * fwd + left * left)
        angle = np.arctan2(left, fwd)

        mask = (dist >= RANGE_MIN) & (dist <= RANGE_MAX) & (np.abs(angle) <= FOV_HALF)
        for i in np.where(mask)[0]:
            # Optical frame: Z=fwd, X=right (=-left), Y=down (=0 in 2-D)
            visible.append((class_id, -float(left[i]), 0.0, float(fwd[i])))

    return visible
