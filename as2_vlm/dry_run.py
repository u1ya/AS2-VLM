from .contracts import TARGET_SKILLS, validate_plan


def preview_plan(plan, bridge):
    validate_plan(plan)
    return {
        "task_id": plan["task_id"],
        "status": "PREVIEW_ONLY",
        "robot": bridge.get_state(),
        "steps": [
            {**step, "requires": ["visual_grounding", "target_tracking", "robot_controller"]
             if step["skill"] in TARGET_SKILLS else ["robot_controller"]}
            for step in plan["steps"]
        ],
        "executed": False,
    }
