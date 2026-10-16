import math


class PlanError(ValueError):
    pass


TARGET_SKILLS = {"face_target", "approach_target"}
SKILLS = TARGET_SKILLS | {"stop", "turn_relative", "walk_distance", "look_around"}


def validate_plan(plan):
    if not isinstance(plan, dict) or set(plan) != {"schema_version", "task_id", "steps"}:
        raise PlanError("Invalid plan fields")
    if type(plan["schema_version"]) is not int or plan["schema_version"] != 1:
        raise PlanError("Unsupported schema version")
    if not isinstance(plan["task_id"], str) or not plan["task_id"].strip():
        raise PlanError("task_id is required")
    steps = plan["steps"]
    if not isinstance(steps, list) or not 1 <= len(steps) <= 2:
        raise PlanError("Expected one or two steps")
    for step in steps:
        if not isinstance(step, dict) or not isinstance(step.get("skill"), str):
            raise PlanError("Invalid step")
        skill = step["skill"]
        if skill not in SKILLS:
            raise PlanError("Unsupported skill")
        fields = {"skill"}
        if skill in TARGET_SKILLS:
            fields.add("target")
            target = step.get("target")
            if not isinstance(target, dict) or set(target) - {"kind", "color", "description"}:
                raise PlanError("Invalid target fields")
            if not isinstance(target.get("kind"), str) or target["kind"] not in {"wall", "door", "person"}:
                raise PlanError("Unsupported target kind")
            if any(not isinstance(v, str) or not v.strip() for v in target.values()):
                raise PlanError("Invalid target value")
        if skill in {"turn_relative", "walk_distance"}:
            key, limit = ("angle_rad", math.pi) if skill == "turn_relative" else ("distance_m", 2.0)
            fields.add(key)
            value = step.get(key)
            if type(value) not in {int, float} or (type(value) is float and not math.isfinite(value)):
                raise PlanError("Expected finite numeric argument")
            if abs(value) > limit or (skill == "walk_distance" and value <= 0):
                raise PlanError("Argument outside MVP limits")
        if set(step) != fields:
            raise PlanError("Unexpected or missing step fields")
    return plan
