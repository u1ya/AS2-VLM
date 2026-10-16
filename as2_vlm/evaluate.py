"""Live inference smoke tests; no robot access and no benchmark claims."""
import argparse
import json
import time
from pathlib import Path

from .contracts import PlanError
from .vlm import DEFAULT_MODEL, OllamaBackend, VLMError, ground_target, plan_instruction, read_image


CASES = [
    ("Дойди до белой стены", [("approach_target", "wall")]),
    ("Дойди до двери", [("approach_target", "door")]),
    ("Оглянись вокруг", [("look_around", None)]),
    ("Повернись в сторону мужчины", [("face_target", "person")]),
    ("Повернись к двери и подойди к ней", [("face_target", "door"), ("approach_target", "door")]),
    ("Пройди вперед на метр и остановись", [("walk_distance", None), ("stop", None)]),
]


def emit_report(report, output=None):
    serialized = json.dumps(report, ensure_ascii=False, indent=2)
    if output:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(serialized + "\n", encoding="utf-8")
    print(serialized)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--demo-image", help="Official Qwen beach demo image; not an arbitrary frame")
    parser.add_argument("--output", help="Save this smoke-test report as UTF-8 JSON")
    args = parser.parse_args()
    backend = OllamaBackend(model=args.model)
    report = {"model": args.model, "executed": False, "cases": [], "vision_cases": []}
    report["scope"] = "In-sample instruction smoke tests and vision status/schema checks; not a navigation benchmark"
    report["localization_accuracy_evaluated"] = False
    try:
        health = backend.healthcheck()
        if not health["model_installed"]:
            raise VLMError("Model is not installed")
        report["model_digest"] = health["model_digest"]
    except VLMError as exc:
        report.update(status="MODEL_UNAVAILABLE", error=str(exc))
        emit_report(report, args.output)
        return 1
    for instruction, expected in CASES:
        started = time.monotonic()
        row = {"instruction": instruction}
        try:
            plan = plan_instruction(backend, instruction)
            actual = [(s["skill"], s.get("target", {}).get("kind")) for s in plan["steps"]]
            row.update(passed=actual == expected, plan=plan)
        except (VLMError, PlanError) as exc:
            row.update(passed=False, error=str(exc))
        row["seconds"] = round(time.monotonic() - started, 3)
        report["cases"].append(row)
    if args.demo_image:
        image = read_image(args.demo_image)
        for target, expected in [({"kind": "person", "description": "person wearing a plaid shirt"}, "found"),
                                 ({"kind": "door"}, "not_found")]:
            started = time.monotonic()
            row = {"target": target, "expected_status": expected}
            try:
                result = ground_target(backend, target, image)
                row.update(passed=result["status"] == expected, observation=result)
            except VLMError as exc:
                row.update(passed=False, error=str(exc))
            row["seconds"] = round(time.monotonic() - started, 3)
            report["vision_cases"].append(row)
    rows = report["cases"] + report["vision_cases"]
    report["passed"] = sum(r["passed"] for r in rows)
    report["total"] = len(rows)
    emit_report(report, args.output)
    return 0 if all(r["passed"] for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
