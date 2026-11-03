#!/usr/bin/env python3
"""Reproduce E01/E02/E03 against the immutable pre-correction implementation.

An expected failure is checked by test name, with the two positive controls
required to pass. This does not substitute for the full baseline regression run.
"""

import argparse
import importlib.util
import json
import math
from pathlib import Path
import re
import subprocess
import tarfile
import xml.etree.ElementTree as ET

BASE = "e9bf01579123eca35ce315b0f605814f8c952255"
PLANNER = "hope_ws/src/hope_planner_cpp"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    source, build = args.output / "source", args.output / "native"
    source.mkdir()
    archive = args.output / "baseline.tar"
    with archive.open("wb") as stream:
        subprocess.run(
            [
                "git",
                "-C",
                str(args.repo),
                "archive",
                BASE,
                PLANNER,
                "hope_ws/src/hope_bringup/scripts/base_pose_contract.py",
                "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include",
            ],
            stdout=stream,
            check=True,
        )
    with tarfile.open(archive) as stream:
        stream.extractall(source, filter="data")
    archive.unlink()

    def run(label, command, expected=0):
        with (args.output / (label + ".log")).open("w") as log:
            result = subprocess.run(
                list(map(str, command)), stdout=log, stderr=subprocess.STDOUT
            )
        (args.output / (label + ".exit")).write_text(str(result.returncode) + "\n")
        assert result.returncode == expected, (label, result.returncode, expected)

    run(
        "configure",
        [
            "cmake",
            "-S",
            source / PLANNER / "test/native",
            "-B",
            build,
            "-DCMAKE_BUILD_TYPE=Release",
        ],
    )
    run("build", ["cmake", "--build", build, "--target", "hope_native_core", "-j2"])
    flags = [
        "g++",
        "-std=c++17",
        "-O2",
        "-I" + str(source / PLANNER / "include"),
        "-I/usr/include/eigen3",
    ]
    run(
        "counterexamples-build",
        flags
        + [
            args.repo / PLANNER / "test/reacquisition_counterexamples.cpp",
            build / "libhope_native_core.a",
            "-lgtest_main",
            "-lgtest",
            "-lpthread",
            "-o",
            args.output / "before",
        ],
    )
    run(
        "counterexamples",
        [
            args.output / "before",
            "--gtest_output=xml:" + str(args.output / "before.xml"),
        ],
        1,
    )
    cases = ET.parse(args.output / "before.xml").findall(".//testcase")
    failed = {c.attrib["name"] for c in cases if c.find("failure") is not None}
    assert len(cases) == 4 and failed == {
        "E01_RejectThenGapAndOrdinaryBallB",
        "E02_QuaternionOnlyMagnusAcquisition",
    }, failed

    producer_path = source / "hope_ws/src/hope_bringup/scripts/base_pose_contract.py"
    spec = importlib.util.spec_from_file_location(
        "baseline_pose_producer", producer_path
    )
    producer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(producer)
    runtime = (source / PLANNER / "src/integrated_planner_runtime.cpp").read_text()
    expression = re.search(r"base.orientation=(.*);", runtime).group(1)
    cpp = '#include "hope_planner_cpp/target_turn_guard.hpp"\n#include <gtest/gtest.h>\n#include <vector>\nusing namespace hope_planner_cpp;\n'
    cpp += (
        "Eigen::Quaterniond actual_consumer(const std::vector<double>& w){return "
        + expression
        + ";}\n"
    )
    for yaw in [0, 30, -30, 50]:
        q = [math.cos(math.radians(yaw) / 2), 0, 0, math.sin(math.radians(yaw) / 2)]
        wire = producer.pose_to_base_flat(
            [-0.43, -0.7, 0],
            q,
            [0, 0, 0],
            [1, 0, 0, 0],
            0.76,
            sequence=1,
            source_sec=1800000000,
            source_nsec=0,
            tracking_quality=1,
            flags=63,
            calibration_id=1,
            world_frame_id=1,
        )
        cpp += (
            "TEST(BaseBefore,Yaw"
            + str(yaw).replace("-", "Minus")
            + "){std::vector<double> w={"
        )
        cpp += ",".join(map(repr, wire)) + "};auto q=actual_consumer(w);"
        cpp += "auto d=check_target_turn(Vec3(.15,-.7,.3),Vec3(-.43,-.7,0),q,true);"
        cpp += f"EXPECT_NEAR(d.base_yaw_deg,{yaw},1e-9);EXPECT_EQ(d.allowed,{str(abs(yaw) <= 40).lower()});}}\n"
    (args.output / "before_base.cpp").write_text(cpp)
    run(
        "base-build",
        flags
        + [
            args.output / "before_base.cpp",
            "-lgtest_main",
            "-lgtest",
            "-lpthread",
            "-o",
            args.output / "before-base",
        ],
    )
    run(
        "base",
        [
            args.output / "before-base",
            "--gtest_output=xml:" + str(args.output / "before-base.xml"),
        ],
        1,
    )
    cases = ET.parse(args.output / "before-base.xml").findall(".//testcase")
    assert {c.attrib["name"] for c in cases if c.find("failure") is not None} == {
        "Yaw30",
        "YawMinus30",
        "Yaw50",
    }
    (args.output / "receipt.json").write_text(
        json.dumps(
            {
                "status": "EXPECTED_FAILURES_REPRODUCED",
                "base": BASE,
                "E01": "FAIL_BEFORE",
                "E02": "FAIL_BEFORE",
                "E03": "FAIL_BEFORE",
                "controls": "PASS",
                "actual_consumer_expression": expression,
                "producer": str(producer_path),
                "scope": "baseline counterexamples; no qualification claim",
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
