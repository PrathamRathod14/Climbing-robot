import math
import subprocess
import time
from dataclasses import dataclass


WORLD = "geckogrip_climber"
BODY = "geckogrip_quadruped"
MARKER = "predicted_hold_marker"

BODY_Y = -0.34
FOOT_Y = -0.18
CLIMB_LIMIT = 5.78
FLOOR_BODY_Y = -1.55
FALL_DURATION = 2.6
MOVE_START = 0.24
MOVE_END = 0.90
BODY_SIDE = 0.065
HIP_SIDE = 0.075
KNEE_SIDE_MOVING = 0.012
KNEE_SIDE_REST = 0.008
KNEE_DROP_MOVING = 0.055
KNEE_DROP_REST = 0.038
KNEE_LIFT_MOVING = 0.040
KNEE_LIFT_REST = 0.012
FOOT_YAW = 0.025
YAW_SCALE = 0.04
YAW_LIMIT = 0.08
LIFT_Y = 0.025
LIFT_Z = 0.070
FRONT_FOOT_X = 0.085
REAR_FOOT_X = 0.055

HOLDS = [
    (-0.72, 0.45), (-0.36, 0.45), (0.0, 0.45), (0.36, 0.45), (0.72, 0.45),
    (-0.58, 0.86), (-0.18, 0.86), (0.22, 0.86), (0.62, 0.86),
    (-0.72, 1.27), (-0.36, 1.27), (0.0, 1.27), (0.36, 1.27), (0.72, 1.27),
    (-0.58, 1.68), (-0.18, 1.68), (0.22, 1.68), (0.62, 1.68),
    (-0.72, 2.09), (-0.36, 2.09), (0.0, 2.09), (0.36, 2.09), (0.72, 2.09),
    (-0.58, 2.50), (-0.18, 2.50), (0.22, 2.50), (0.62, 2.50),
    (-0.72, 2.91), (-0.36, 2.91), (0.0, 2.91), (0.36, 2.91), (0.72, 2.91),
    (-0.58, 3.32), (-0.18, 3.32), (0.22, 3.32), (0.62, 3.32),
    (-0.72, 3.73), (-0.36, 3.73), (0.0, 3.73), (0.36, 3.73), (0.72, 3.73),
    (-0.58, 4.14), (-0.18, 4.14), (0.22, 4.14), (0.62, 4.14),
    (-0.72, 4.55), (-0.36, 4.55), (0.0, 4.55), (0.36, 4.55), (0.72, 4.55),
    (-0.58, 4.96), (-0.18, 4.96), (0.22, 4.96), (0.62, 4.96),
    (-0.72, 5.37), (-0.36, 5.37), (0.0, 5.37), (0.36, 5.37), (0.72, 5.37),
    (-0.58, 5.78), (-0.18, 5.78), (0.22, 5.78), (0.62, 5.78),
    (-0.72, 6.19), (-0.36, 6.19), (0.0, 6.19), (0.36, 6.19), (0.72, 6.19),
    (-0.58, 6.60), (-0.18, 6.60), (0.22, 6.60), (0.62, 6.60),
    (-0.72, 7.01), (-0.36, 7.01), (0.0, 7.01), (0.36, 7.01), (0.72, 7.01),
    (-0.58, 7.42), (-0.18, 7.42), (0.22, 7.42), (0.62, 7.42),
    (-0.72, 7.83), (-0.36, 7.83), (0.0, 7.83), (0.36, 7.83), (0.72, 7.83),
    (-0.58, 8.24), (-0.18, 8.24), (0.22, 8.24), (0.62, 8.24),
    (-0.72, 8.65), (-0.36, 8.65), (0.0, 8.65), (0.36, 8.65), (0.72, 8.65),
]

ROUTES = {
    "FL": [1, 6, 10, 15, 19, 24, 28, 33, 37, 42, 46, 51, 55, 60, 64, 69, 73, 78, 82, 87, 91],
    "FR": [3, 7, 12, 16, 21, 25, 30, 34, 39, 43, 48, 52, 57, 61, 66, 70, 75, 79, 84, 88, 93],
    "RL": [6, 10, 15, 19, 24, 28, 33, 37, 42, 46, 51, 55, 60, 64, 69, 73, 78, 82, 87, 91],
    "RR": [7, 12, 16, 21, 25, 30, 34, 39, 43, 48, 52, 57, 61, 66, 70, 75, 79, 84, 88, 93],
}

PENDING_COMMANDS: list[list[str]] = []


@dataclass
class Limb:
    key: str
    side: float
    shoulder_z: float
    route_pos: int
    start: tuple[float, float, float]
    target: tuple[float, float, float]


def hold_point(index: int) -> tuple[float, float, float]:
    x, z = HOLDS[index]
    return (x, FOOT_Y, z)


def narrow_wall_foot(limb: Limb, foot: tuple[float, float, float]) -> tuple[float, float, float]:
    lane = FRONT_FOOT_X if limb.shoulder_z > 0.0 else REAR_FOOT_X
    return (limb.side * lane, foot[1], foot[2])


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def gz_set(model: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> None:
    PENDING_COMMANDS.append([
        "gz", "model", "-w", WORLD, "-m", model,
        "-x", f"{x:.3f}", "-y", f"{y:.3f}", "-z", f"{z:.3f}",
        "-R", f"{roll:.3f}", "-P", f"{pitch:.3f}", "-Y", f"{yaw:.3f}",
    ])


def flush_pose_updates() -> None:
    global PENDING_COMMANDS
    commands = PENDING_COMMANDS
    PENDING_COMMANDS = []
    processes = []
    for command in commands:
        try:
            processes.append(subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        except OSError:
            continue
    for process in processes:
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()


def segment_pose(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float, float, float, float]:
    ax, ay, az = a
    bx, by, bz = b
    dx, dy, dz = bx - ax, by - ay, bz - az
    length = max(0.001, math.sqrt(dx * dx + dy * dy + dz * dz))
    yaw = math.atan2(dy, dx)
    pitch = math.acos(max(-1.0, min(1.0, dz / length)))
    return ((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5, 0.0, pitch, yaw)


def desired_body(limbs: list[Limb]) -> tuple[float, float]:
    x = sum(limb.target[0] - limb.side * BODY_SIDE for limb in limbs) / len(limbs)
    z = sum(limb.target[2] - limb.shoulder_z for limb in limbs) / len(limbs)
    return x, z + 0.11


def shoulder(body_x: float, body_z: float, limb: Limb) -> tuple[float, float, float]:
    return (body_x + limb.side * HIP_SIDE, BODY_Y - 0.01, body_z + limb.shoulder_z)


def foot_position(limb: Limb, active: Limb | None, progress: float) -> tuple[float, float, float]:
    if active is not limb:
        return limb.target

    if progress <= MOVE_START:
        return limb.start
    if progress >= MOVE_END:
        return limb.target

    t = smoothstep((progress - MOVE_START) / (MOVE_END - MOVE_START))
    x = lerp(limb.start[0], limb.target[0], t)
    y = lerp(limb.start[1], limb.target[1], t) - LIFT_Y * math.sin(t * math.pi)
    z = lerp(limb.start[2], limb.target[2], t) + LIFT_Z * math.sin(t * math.pi)
    return x, y, z


def set_body(body_x: float, body_z: float, limbs: list[Limb]) -> None:
    left_z = sum(limb.target[2] for limb in limbs if limb.side < 0) / 2.0
    right_z = sum(limb.target[2] for limb in limbs if limb.side > 0) / 2.0
    yaw = max(-YAW_LIMIT, min(YAW_LIMIT, (left_z - right_z) * YAW_SCALE))
    gz_set(BODY, body_x, BODY_Y, body_z, math.pi / 2, -math.pi / 2, yaw)


def set_falling_body(body_x: float, body_z: float, fall_t: float) -> None:
    y = lerp(BODY_Y, FLOOR_BODY_Y, smoothstep(fall_t))
    roll = lerp(math.pi / 2, 0.0, smoothstep(fall_t))
    pitch = lerp(-math.pi / 2, 0.0, smoothstep(fall_t))
    gz_set(BODY, body_x, y, body_z, roll, pitch, 0.0)


def set_marker_to_hold(index: int) -> None:
    x, z = HOLDS[index]
    gz_set(MARKER, x, -0.12, z, 1.571, 0.0, 0.0)


def set_limb(limb: Limb, body_x: float, body_z: float, active: Limb | None, progress: float) -> None:
    foot = foot_position(limb, active, progress)
    hip = shoulder(body_x, body_z, limb)
    moving = active is limb and progress < 1.0
    bend = limb.side * (KNEE_SIDE_MOVING if moving else KNEE_SIDE_REST)
    knee = (
        lerp(hip[0], foot[0], 0.54) + bend,
        lerp(hip[1], foot[1], 0.52) - (KNEE_DROP_MOVING if moving else KNEE_DROP_REST),
        lerp(hip[2], foot[2], 0.52) + (KNEE_LIFT_MOVING if moving else KNEE_LIFT_REST),
    )

    gz_set(f"{limb.key}_hip_joint", hip[0], hip[1], hip[2], 0.0, 0.0, 0.0)
    gz_set(f"{limb.key}_knee_joint", knee[0], knee[1], knee[2], 0.0, 0.0, 0.0)
    gz_set(f"{limb.key}_upper_leg", *segment_pose(hip, knee))
    gz_set(f"{limb.key}_lower_leg", *segment_pose(knee, foot))
    gz_set(f"{limb.key}_adhesive_foot", foot[0], foot[1], foot[2], 1.571, 0.0, limb.side * FOOT_YAW)


def advance_limb(limb: Limb) -> int:
    route = ROUTES[limb.key]
    limb.route_pos = min(limb.route_pos + 1, len(route) - 1)
    next_hold = route[limb.route_pos]
    limb.start = limb.target
    limb.target = narrow_wall_foot(limb, hold_point(next_hold))
    return next_hold


def make_limb(key: str, side: float, shoulder_z: float) -> Limb:
    limb = Limb(key, side, shoulder_z, 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    start = narrow_wall_foot(limb, hold_point(ROUTES[key][0]))
    limb.start = start
    limb.target = start
    return limb


def main() -> None:
    limbs = [
        make_limb("FL", -1.0, 0.32),
        make_limb("FR", 1.0, 0.32),
        make_limb("RL", -1.0, -0.32),
        make_limb("RR", 1.0, -0.32),
    ]
    gait = ["FL", "FR", "FL", "FR", "RL", "RR"]
    body_x, body_z = desired_body(limbs)
    step_index = 0

    while True:
        active = next(limb for limb in limbs if limb.key == gait[step_index % len(gait)])
        next_hold = advance_limb(active)
        set_marker_to_hold(next_hold)

        target_x, target_z = desired_body(limbs)
        mid_x = lerp(body_x, target_x, 0.35)
        mid_z = lerp(body_z, target_z, 0.35)
        set_body(mid_x, mid_z, limbs)
        set_limb(active, mid_x, mid_z, active, 0.55)
        flush_pose_updates()
        time.sleep(0.080)

        body_x, body_z = desired_body(limbs)
        set_body(body_x, body_z, limbs)
        for limb in limbs:
            set_limb(limb, body_x, body_z, None, 1.0)
        flush_pose_updates()
        time.sleep(0.080)
        step_index += 1

        if body_z >= CLIMB_LIMIT:
            for i in range(24):
                set_falling_body(body_x, body_z, i / 23.0)
                flush_pose_updates()
                time.sleep(FALL_DURATION / 24.0)
            break


if __name__ == "__main__":
    main()
