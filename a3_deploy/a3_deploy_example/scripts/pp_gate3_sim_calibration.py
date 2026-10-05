#!/usr/bin/env python3
"""Write an explicit simulation-only marker transform; never a venue receipt."""
import argparse
import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'hope_ws/src/hope_bringup/scripts'))
from p1_marker_layout import layout_metadata


def simulation_receipt():
    return {
        'approved': True,
        'simulation_only': True,
        'source': {'kind': 'mujoco', 'rigid_body_id': 1},
        'cad': {'marker_layout': 'stickers_v3'},
        'marker_layout': layout_metadata(),
        'runtime_world': {'table_side': 'P1'},
        'p1_to_pelvis': {
            'parent_frame': 'UCB_P1', 'child_frame': 'pelvis_link',
            'translation_m': [0.0, 0.0, 0.0],
            'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
        },
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(simulation_receipt(), indent=2) + '\n')
