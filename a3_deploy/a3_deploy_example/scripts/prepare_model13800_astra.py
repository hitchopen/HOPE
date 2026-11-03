#!/usr/bin/env python3
"""Reproduce the exact experimental Astra policy/deploy bytes; never install."""
import argparse
import json
from pathlib import Path
import shutil
import tempfile

import onnx
from onnx import helper
import yaml

from prepare_fusion_weight_edit import POLICY_SHA, digest, prepare

POSITION = [0.03604027350063854, -0.011837893722118357, 0.015332482418976515]
VELOCITY = [0.09511607918972412, 0.013321114908349047, -0.07313963800412887]
ASTRA_POLICY_SHA = 'fa9f8470fab43da30d9884afee0a207f6e1eb6dedfdf8b98b7f2d2e5302a5a9d'
ASTRA_DEPLOY_SHA = '0570621dbb1ba92c77aa8429d4eaec752dc51d962a264eecd1cd04876cc5e10f'


def reproduce(source, output):
    if output.exists():
        raise ValueError('output must be a new directory')
    with tempfile.TemporaryDirectory() as tmp:
        candidate = Path(tmp) / 'candidate'
        receipt = prepare(source, candidate, 'joint', .045, 1.3, POSITION, VELOCITY)
        policy = candidate / 'exported/policy.onnx'
        model = onnx.load(policy)
        metadata = {v.key: v.value for v in model.metadata_props}
        metadata.update(weight_edit='iter1_candidate',
                        weight_edit_status='experimental_joint_position_velocity_normal_search')
        helper.set_model_props(model, metadata)
        onnx.save(model, policy)
        deploy = candidate / 'params/deploy.yaml'
        config = yaml.safe_load(deploy.read_text())
        edit = dict(name='iter1_candidate', parent_sha256=POLICY_SHA,
                    method='clock_conditioned_position_velocity_direction',
                    position_delta_xyz_m=POSITION, velocity_delta_xyz_mps=VELOCITY,
                    wait_tts=.82,
                    normal_supervision='Planner face plane evaluated jointly; current contract n=unit(v), no independent normal input')
        config['provenance']['policy_sha256'] = digest(policy)
        config['provenance']['weight_edit'] = edit
        deploy.write_text(yaml.safe_dump(config, sort_keys=False))
        if (digest(policy), digest(deploy)) != (ASTRA_POLICY_SHA, ASTRA_DEPLOY_SHA):
            raise ValueError('Astra byte reproduction failed; inspect ONNX/PyYAML serialization versions')
        receipt.update(parameters=edit, policy_sha256=ASTRA_POLICY_SHA,
                       deploy_sha256=ASTRA_DEPLOY_SHA, exact_astra_artifact=True)
        (candidate / 'weight-edit-receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
        shutil.copytree(candidate, output)
        return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source13800', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(reproduce(args.source13800, args.output), indent=2))


if __name__ == '__main__':
    main()
