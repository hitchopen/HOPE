#!/usr/bin/env python3
"""Require the installed forecaster to omit inverse-planning and packing symbols."""
import argparse
import json
import subprocess

parser=argparse.ArgumentParser()
parser.add_argument('--executable',required=True)
args=parser.parse_args()
symbols=subprocess.check_output(['nm','-C',args.executable],text=True)
for symbol in ['RacketTargetPlanner::','Schema2Packer::','pack_schema2','planner_locked']:
    assert symbol not in symbols, symbol
print(json.dumps(dict(status='PASS',inverse_and_command_symbols_absent=True)))
