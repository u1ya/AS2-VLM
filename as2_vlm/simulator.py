"""Kinematic, known-map demo. No SDK or physical robot connection."""
import math

from .contracts import PlanError, validate_plan

WORLD = {"width": 8, "height": 6, "radius": 0.22,
         "targets": {"door": [7.4, 4.8], "wall": [4, 5.8], "person": [1.4, 4.7]},
         "obstacle": [3.2, 2.5, 4.6, 3.4], "start": [1.3, 1.2, 0]}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def free(x, y, obstacle):
    r = WORLD["radius"]
    if not (r <= x <= WORLD["width"] - r and r <= y <= WORLD["height"] - r):
        return False
    a, b, c, d = WORLD["obstacle"]
    return not obstacle or math.hypot(x - min(max(x, a), c), y - min(max(y, b), d)) > r


def simulate(plan, pose=None, obstacle=False):
    validate_plan(plan)
    pose = list(WORLD["start"] if pose is None else pose)
    if (len(pose) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in pose)
            or not free(pose[0], pose[1], obstacle)):
        raise PlanError("Invalid simulated starting pose")
    frames = [{"pose": pose[:], "step": 0, "skill": "idle"}]

    def frame(index, skill):
        frames.append({"pose": pose[:], "step": index, "skill": skill})

    def turn(delta, index, skill):
        count = max(1, math.ceil(abs(delta) / 0.06))
        for _ in range(count):
            pose[2] = wrap(pose[2] + delta / count)
            frame(index, skill)

    def walk(distance, index, skill):
        count = max(1, math.ceil(distance / 0.025))
        for _ in range(count):
            x = pose[0] + math.cos(pose[2]) * distance / count
            y = pose[1] + math.sin(pose[2]) * distance / count
            if not free(x, y, obstacle):
                return False
            pose[0:2] = [x, y]
            frame(index, skill)
        return True

    status = "completed"
    for index, step in enumerate(plan["steps"], 1):
        skill = step["skill"]
        if skill == "stop":
            frame(index, skill)
            status = "stopped"
            break
        if skill == "look_around":
            turn(2 * math.pi, index, skill)
        elif skill == "turn_relative":
            turn(step["angle_rad"], index, skill)
        elif skill == "walk_distance":
            if not walk(step["distance_m"], index, skill):
                status = "blocked"
                break
        else:
            target = step["target"]
            if target.get("color") not in (None, "white") or (target["kind"] != "wall" and target.get("color")):
                raise PlanError("Target not present in simulated map")
            x, y = WORLD["targets"][target["kind"]]
            dx, dy = x - pose[0], y - pose[1]
            distance = math.hypot(dx, dy)
            if distance > 1e-9:
                turn(wrap(math.atan2(dy, dx) - pose[2]), index, skill)
            if skill == "approach_target" and not walk(max(0, distance - 0.65), index, skill):
                status = "blocked"
                break
    return {"plan": plan, "frames": frames, "status": status, "final_pose": pose,
            "robot_connected": False, "perception": "known_map", "physics": "kinematic"}
