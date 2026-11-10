"""Read-only bilingual language evaluation; never creates a robot adapter.

python -m as2_vlm.agent.evaluate_language --output reports/language.json
Add --model-only to test model semantics even for recognized commands.
"""
import argparse
import json
import math
import time
from pathlib import Path

from .config import load_config
from .physical_planner import PhysicalPlanner
from ..vlm import VLMError


def matches(result, case):
    if result.status != case["status"] or len(result.steps) != len(case["steps"]):
        return False
    for step, (tool, amount) in zip(result.steps, case["steps"]):
        value = step.angle_rad if tool == "rotate_relative" else step.distance_m or 0
        expected = math.radians(amount) if tool == "rotate_relative" else amount
        if step.tool != tool or value is None or not math.isclose(value, expected, rel_tol=0, abs_tol=1e-4):
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", default=str(Path(__file__).with_name("language_cases")/"language_commands.json"))
    parser.add_argument("--model-only", action="store_true")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--output")
    args = parser.parse_args()
    cases = json.loads(Path(args.cases).read_text(encoding="utf-8"))
    if args.limit > 0:
        cases = cases[:args.limit]
    planner = PhysicalPlanner(load_config(path="", env={}))
    report = {"executed":False, "mode":"model_with_guards" if args.model_only else "hybrid",
              "model":planner.config.vlm.model, "cases":[]}
    for case in cases:
        started = time.monotonic()
        row = {"text":case["text"], "expected":case}
        try:
            result = planner.plan(case["text"], model_only=args.model_only)
            row.update(passed=matches(result,case), actual=result.model_dump())
        except (VLMError, ValueError) as exc:
            row.update(passed=False,rejected=True,error=str(exc))
        row["seconds"] = round(time.monotonic()-started,3)
        report["cases"].append(row)
        print(f"{'PASS' if row['passed'] else 'FAIL'}: {case['text']}", flush=True)
    report.update(passed=sum(row["passed"] for row in report["cases"]), total=len(cases))
    if args.output:
        path = Path(args.output)
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(f"{report['passed']}/{report['total']} passed; no robot actions sent.")
    return 0 if report["passed"] == report["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
