"""Command-line interface for generating and inspecting A3 serve artifacts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .config import default_config_path, load_config
from .csvio import MotionCsv, sha256_file
from .mujoco_scene import A3ServeScene
from .pipeline import generate


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="hope-a3-serve",
        description="Generate an A3 serve CSV through MuJoCo physics and DLS IK.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    inspect_parser = subparsers.add_parser(
        "inspect", help="load the official model and validate configured assets"
    )
    inspect_parser.add_argument("--config", default=str(default_config_path()))
    generate_parser = subparsers.add_parser(
        "generate", help="search, solve IK, replay, and export an A3 artifact"
    )
    generate_parser.add_argument("--config", default=str(default_config_path()))
    generate_parser.add_argument("--output", required=True)
    csv_parser = subparsers.add_parser(
        "validate-csv", help="validate an SDK CSV without publishing robot commands"
    )
    csv_parser.add_argument("csv")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "validate-csv":
        motion = MotionCsv.load(args.csv)
        print(
            json.dumps(
                {
                    "path": str(motion.path),
                    "shape": list(motion.values.shape),
                    "sha256": sha256_file(motion.path),
                },
                indent=2,
            )
        )
        return 0
    if args.command == "inspect":
        config = load_config(args.config)
        template = MotionCsv.load(config["source"]["template_csv"])
        scene = A3ServeScene(config["model"]["xml"], config["physics"], config["model"])
        print(
            json.dumps(
                {
                    "config": str(Path(config["_config_path"])),
                    "model": str(scene.model_xml),
                    "model_name": scene.model.names.decode(errors="ignore").split("\x00")[0]
                    if isinstance(scene.model.names, bytes)
                    else "A3",
                    "nq": int(scene.model.nq),
                    "nv": int(scene.model.nv),
                    "racket_site": scene.racket_site_name,
                    "racket_geom": scene.racket_geom_name,
                    "template_csv": str(template.path),
                    "template_sha256": sha256_file(template.path),
                },
                indent=2,
            )
        )
        return 0
    result = generate(args.config, args.output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

