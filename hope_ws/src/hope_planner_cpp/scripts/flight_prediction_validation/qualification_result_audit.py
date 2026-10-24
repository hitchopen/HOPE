#!/usr/bin/env python3
"""Check discovered ROS test completion and exact inherited lint findings."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


def lint_findings(build, name):
    path = build / 'hope_bringup/test_results/hope_bringup' / (name + '.xunit.xml')
    cases = ET.parse(path).getroot().findall('.//testcase')
    # Only locations shift; preserve file, rule, exact source and message, and
    # multiplicity. A new finding cannot hide behind an inherited failure name.
    return Counter(re.sub(r':\d+(?::\d+)?\)', ')', case.attrib['name']) + ' ' +
                   case.find('failure').attrib.get('message', '')
                   for case in cases if case.find('failure') is not None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--baseline-build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    packages, lint = {}, {}
    unexpected = []
    for package in ('hope_msgs', 'hope_planner_cpp', 'hope_bringup'):
        build = args.build / package
        registered = json.loads(subprocess.check_output(
            ['ctest', '--test-dir', str(build), '--show-only=json-v1'], text=True))
        expected = {test['name'] for test in registered['tests']}
        tag = (build / 'Testing/TAG').read_text().splitlines()[0]
        tree = ET.parse(build / 'Testing' / tag / 'Test.xml')
        actual = {test.findtext('Name'): test.attrib['Status']
                  for test in tree.findall('./Testing/Test')}
        missing = sorted(expected - actual.keys())
        failures = {name: status for name, status in actual.items() if status != 'passed'}
        unexpected.extend(f'{package}/{name}: {status}' for name, status in failures.items()
                          if package != 'hope_bringup' or name not in ('flake8', 'pep257'))
        unexpected.extend(f'{package}/{name}: NOT_RUN' for name in missing)
        packages[package] = dict(registered=len(expected), executed=len(actual), missing=missing,
                                 failures=failures, tests=actual)
    for name in ('flake8', 'pep257'):
        baseline = lint_findings(args.baseline_build, name)
        candidate = lint_findings(args.build, name)
        added, removed = candidate - baseline, baseline - candidate
        lint[name] = dict(baseline=sum(baseline.values()), candidate=sum(candidate.values()),
                          added=list(added.elements()), removed=list(removed.elements()))
        unexpected.extend(f'{name}: {finding}' for finding in added.elements())
    receipt = dict(status='PASS' if not unexpected else 'FAIL', packages=packages,
                   inherited_lint=lint, unexpected_failures=unexpected,
                   note='This delta audit does not replace or zero the raw colcon/test aggregate exit.')
    args.output.write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps({k: v for k, v in receipt.items() if k not in ('packages', 'inherited_lint')}))
    return bool(unexpected)


if __name__ == '__main__':
    raise SystemExit(main())
