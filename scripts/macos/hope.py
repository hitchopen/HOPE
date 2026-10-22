#!/usr/bin/env python3
"""Local Docker entry point; does not connect to or start a physical robot."""
import sys

# The sibling operator.py is a CLI, not Python's standard-library operator
# module. Direct script execution otherwise shadows that module while argparse
# imports collections/functools. No imports here require this script directory.
if __name__ == "__main__" and not __package__:
    sys.path.pop(0)

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def source_directory(root, project, marker):
    """Keep build_6 paths when present; accept main's existing layout as well."""
    for parent in ("agi", "a3_deploy"):
        candidate = root / parent / project
        if (candidate / marker).is_file():
            return candidate
    raise FileNotFoundError(f"missing {project} source under {root}/agi or {root}/a3_deploy")


def run(args, **kwargs):
    subprocess.run([str(x) for x in args], check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=[
        "image", "shell", "build-ros", "test-ros", "build-runner",
        "build-sim", "inspect-policy", "build-rockchip", "build-thor", "exec", "build-operator",
    ])
    parser.add_argument("--arch", choices=["arm64", "amd64"], default="arm64")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--policy-dir", type=Path)
    parser.add_argument("--teleop-policy-dir", type=Path)
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--no-tests", action="store_true", help="omit ROS test targets when building a runtime workspace")
    parser.add_argument("--with-lint", action="store_true",
                        help="also run inherited ament linters (test-ros/build-operator); default is functional regression tests")
    parser.add_argument("--clean-cache", action="store_true",
                        help="reset ROS CMake caches after changing checkout layout (build-ros/build-operator)")
    parser.add_argument("--publish-port", type=int, action="append", default=[],
                        help="publish a container TCP port on Mac localhost (shell/exec only)")
    parser.add_argument("--virtual-display", action="store_true",
                        help="run shell/exec under Xvfb with software OpenGL")
    args, extra = parser.parse_known_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    if args.clean_cache and args.operation not in {"build-ros", "build-operator"}:
        parser.error("--clean-cache requires build-ros or build-operator")
    if args.with_lint and (args.operation not in {"test-ros", "build-operator"} or args.no_tests):
        parser.error("--with-lint requires test-ros or build-operator with tests enabled")
    if extra and args.operation != "exec":
        parser.error("unexpected arguments: " + " ".join(extra))
    if args.publish_port and args.operation not in ("shell", "exec"):
        parser.error("--publish-port requires shell or exec")
    if args.virtual_display and args.operation not in ("shell", "exec"):
        parser.error("--virtual-display requires shell or exec")
    if any(not 1 <= port <= 65535 for port in args.publish_port):
        parser.error("ports must be between 1 and 65535")
    if args.operation == "build-operator":
        if args.arch != "arm64":
            parser.error("the Mac operator image requires --arch arm64")
        # Local build only: never invokes operator.py bind/select or robot SSH.
        entry = Path(__file__).resolve()
        run([sys.executable, entry, "image", "--arch", "arm64"])
        build = [sys.executable, entry, "build-ros", "--arch", "arm64", "--jobs", str(args.jobs)]
        if args.no_tests:
            build.append("--no-tests")
        if args.clean_cache:
            build.append("--clean-cache")
        run(build)
        if not args.no_tests:
            tests = [sys.executable, entry, "test-ros", "--arch", "arm64", "--jobs", str(args.jobs)]
            if args.with_lint:
                tests.append("--with-lint")
            run(tests)
        run(["docker", "build", "--provenance=false", "--platform", "linux/arm64", "-t", "hope-operator:jazzy-arm64",
             "-f", ROOT / "scripts/macos/Dockerfile.operator", ROOT / "scripts/macos"])
        print("Mac operator built locally: NatNet + VRPN, operator:hope, schema-5 migration support. "
              "No robot files, profiles or services changed; live network acceptance is still required.")
        return
    image = "hope-dev:jazzy-" + args.arch
    if args.operation == "image":
        run(["docker", "build", "--provenance=false", "--platform", "linux/" + args.arch,
             "-t", image, "-f", ROOT / "scripts/macos/Dockerfile.ros",
             ROOT / "scripts/macos"])
        return
    mounts = []
    for name, value in [("policy-src", args.policy_dir),
                        ("teleop-src", args.teleop_policy_dir)]:
        if value:
            value = value.expanduser().resolve(strict=True)
            mounts += ["-v", f"{value}:/{name}:ro"]
    if args.operation in ("build-rockchip", "build-thor"):
        if not args.build_only and not args.policy_dir:
            parser.error("choose --build-only or provide --policy-dir for packaging")
        deploy = source_directory(ROOT, "a3_deploy_example", "CMakeLists.txt")
        target = args.operation.removeprefix("build-")
        image = f"a3-{target}-builder:1.0"
        archive_rel = f"thirdparty/{target}_sysroot/{target}-1.0-aarch64-sysroot.tar.gz"
        archive = deploy / archive_rel
        if not archive.is_file():
            parser.error(f"missing target sysroot: {archive}")
        present = subprocess.run(["docker", "image", "inspect", image],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if present.returncode:
            with tempfile.TemporaryDirectory(prefix="hope-builder-") as tmp:
                context = Path(tmp)
                (context / archive_rel).parent.mkdir(parents=True)
                try:
                    os.link(archive, context / archive_rel)
                except OSError:
                    shutil.copy2(archive, context / archive_rel)
                dockerfile = deploy / f"docker/Dockerfile.a3-{target}-builder"
                shutil.copy2(dockerfile, context / "Dockerfile")
                run(["docker", "build", "--platform", "linux/amd64",
                     "-t", image, context])
        command = ["bash", "scripts/build_a3_deploy_pkg.sh", "--arch", target,
                   "--inside-docker", "--jobs", str(args.jobs)]
        if args.build_only:
            command += ["--build-only"]
        elif args.policy_dir:
            command += ["--policy-dir", "/policy-src", "--runtime-cfg",
                        "src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.hitter_pingpong.yaml"]
            if args.teleop_policy_dir:
                command += ["--teleop-policy-dir", "/teleop-src"]
        run(["docker", "run", "--rm", "--platform", "linux/amd64",
             "-v", f"{deploy}:/work",
             *mounts, "-w", "/work",
             "-e", "HAS_ROS2=1", image, *command])
        return
    if args.operation in ("build-runner", "build-sim", "inspect-policy") and args.arch != "amd64":
        parser.error("the current native Runner/Gate3 entry requires --arch amd64")
    deploy = source_directory(ROOT, "a3_deploy_example", "CMakeLists.txt")
    sim = source_directory(ROOT, "A3_MuJoCo_Sim/aimrt_mujoco_sim", "CMakeLists.txt")
    deploy_container = "/work/" + deploy.relative_to(ROOT).as_posix()
    sim_container = "/work/" + sim.relative_to(ROOT).as_posix()
    command = ["docker", "run", "--rm", "--init", "--platform", "linux/" + args.arch,
               # This Docker Desktop VM's tmpfs rejects POSIX ACLs. An anonymous
               # Linux volume supports Iceoryx ACLs and is removed with --rm.
               "--mount", "type=volume,destination=/dev/shm",
               "-v", f"{ROOT}:/work",
               "-v", f"hope-macos-{args.arch}:/opt/hope",
               "-v", f"hope-macos-sim-deps:{sim_container}/_deps",
               *mounts, "-w", "/work", "-e", f"HOPE_JOBS={args.jobs}",
               "-e", f"HOPE_DEPLOY_DIR={deploy_container}",
               "-e", f"HOPE_SIM_DIR={sim_container}"]
    for port in args.publish_port:
        command += ["-p", f"127.0.0.1:{port}:{port}"]
    if args.operation == "shell":
        command += ["-it"]
    if args.virtual_display:
        command += ["-e", "HOPE_VIRTUAL_DISPLAY=1", "-e", "LIBGL_ALWAYS_SOFTWARE=1",
                    "-e", f"LP_NUM_THREADS={args.jobs}"]
    if args.policy_dir:
        command += ["-e", "HOPE_POLICY_DIR=/policy-src"]
    if args.teleop_policy_dir:
        command += ["-e", "HOPE_TELEOP_DIR=/teleop-src"]
    if args.build_only:
        command += ["-e", "HOPE_BUILD_ONLY=1"]
    if args.no_tests:
        command += ["-e", "HOPE_ROS_TESTING=OFF"]
    if args.with_lint:
        command += ["-e", "HOPE_ROS_LINT=1"]
    if args.clean_cache:
        command += ["-e", "HOPE_ROS_CLEAN_CACHE=1"]
    # Snapshot the entry script so editing it during a long build cannot move
    # the running shell's file offset into another operation's code.
    inside = (ROOT / "scripts/macos/inside.sh").read_text()
    command += [image, "bash", "-c", inside, "hope-macos", args.operation]
    if args.operation == "exec":
        command += extra[1:] if extra[:1] == ["--"] else extra
    run(command)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        raise SystemExit(exc.returncode)
