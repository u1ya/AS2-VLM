import argparse
import json

from .bridge import MockBridge
from .contracts import PlanError
from .dry_run import preview_plan
from .parser import parse_instruction
from .state_bridge import ReadOnlyAS2Bridge, StateReadError
from .vlm import DEFAULT_MODEL, OllamaBackend, VLMError, ground_target, plan_instruction, read_image


def main():
    parser = argparse.ArgumentParser(description="AS2 instruction preview; no hardware control")
    parser.add_argument("instruction", nargs="?")
    parser.add_argument("--backend", choices=["rules", "ollama"], default="rules")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--endpoint", default="http://127.0.0.1:11434")
    parser.add_argument("--image", help="PNG/JPEG frame for local VLM grounding")
    parser.add_argument("--healthcheck", action="store_true")
    parser.add_argument("--robot-state", action="store_true", help="Read AS2 state only; no motion")
    parser.add_argument("--robot-interface", default="eth1")
    parser.add_argument("--wsl-distro", default="Ubuntu-22.04")
    args = parser.parse_args()
    try:
        if args.robot_state:
            if args.instruction or args.healthcheck or args.image:
                parser.error('--robot-state cannot be combined with planning or model healthcheck')
            print(json.dumps(ReadOnlyAS2Bridge(args.robot_interface, args.wsl_distro).get_state(), ensure_ascii=False, indent=2))
            return 0
        if args.healthcheck:
            result = OllamaBackend(args.endpoint, args.model, timeout=5).healthcheck()
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0 if result["model_installed"] else 1
        if not args.instruction:
            parser.error("instruction is required unless --healthcheck is used")
        if args.image and args.backend != "ollama":
            parser.error("--image requires --backend ollama")
        backend = OllamaBackend(args.endpoint, args.model) if args.backend == "ollama" else None
        encoded_image = read_image(args.image) if args.image else None
        plan = plan_instruction(backend, args.instruction) if backend else parse_instruction(args.instruction)
        result = preview_plan(plan, MockBridge())
        result["planning_backend"] = args.backend
        result["model"] = args.model if backend else None
        if encoded_image:
            observations = []
            for step in plan["steps"]:
                target = step.get("target")
                if target and not any(o["target"] == target for o in observations):
                    observations.append(ground_target(backend, target, encoded_image))
            result["observations"] = observations
            result["visual_targets_resolved"] = all(o["status"] == "found" for o in observations)
    except (PlanError, VLMError, StateReadError, OSError) as exc:
        print(json.dumps({"status": "REJECTED", "reason": str(exc), "executed": False}, ensure_ascii=False))
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
