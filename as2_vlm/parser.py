import math
import re
import uuid

from .contracts import PlanError, validate_plan


TARGETS = {
    "белой стене": {"kind": "wall", "color": "white"},
    "белой стены": {"kind": "wall", "color": "white"},
    "двери": {"kind": "door"},
    "мужчине": {"kind": "person", "description": "мужчина"},
    "мужчины": {"kind": "person", "description": "мужчина"},
}


def parse_instruction(instruction):
    if not isinstance(instruction, str) or not instruction.strip():
        raise PlanError("Empty instruction")
    normalized = re.sub(r"\s+", " ", instruction.lower().replace("ё", "е")).strip().rstrip(".!?")
    parts = re.split(r"\s+(?:и затем|затем|и)\s+", normalized)
    steps = []
    last_target = None
    for part in parts:
        step = None
        if part in {"стоп", "остановись"}:
            step = {"skill": "stop"}
        elif part == "оглянись вокруг":
            step = {"skill": "look_around"}
        elif part in {"повернись направо", "повернись налево"}:
            step = {"skill": "turn_relative", "angle_rad": -math.pi / 2 if part.endswith("направо") else math.pi / 2}
        else:
            movement = re.fullmatch(r"пройди вперед на (метр|\d+(?:[.,]\d+)? мет(?:р|ра|ров))", part)
            target_match = re.fullmatch(r"(дойди до|подойди к|повернись к|повернись в сторону) (.+)", part)
            if movement:
                amount = movement.group(1)
                step = {"skill": "walk_distance", "distance_m": 1.0 if amount == "метр" else float(amount.split()[0].replace(",", "."))}
            elif target_match:
                verb, phrase = target_match.groups()
                target = last_target if phrase in {"ней", "нему"} else TARGETS.get(phrase)
                if target is not None:
                    last_target = dict(target)
                    step = {"skill": "face_target" if verb.startswith("повернись") else "approach_target", "target": dict(target)}
        if step is None:
            raise PlanError("Unsupported or ambiguous instruction: " + part)
        steps.append(step)
    return validate_plan({"schema_version": 1, "task_id": str(uuid.uuid4()), "steps": steps})
