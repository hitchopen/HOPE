"""Build correlated CompleteHold questions, with parent-disjoint train/dev/test.

Augment complete legacy contact tuples, then re-run the existing training ball
law, incoming-bounce and outgoing-net/landing checks. Keep the deployed station
inverse exactly: contact_xy = station_xy + (0.58, side_reach). No independent
Cartesian sampling of contact position, velocity, normal and spin.

This qualifies ball geometry, not A3 balance/torque/collision feasibility.
Run with a torch Python; it never creates a simulator or starts training.
"""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import sys

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
FIELDS = ('clip', 'station_xy', 'target_position_offset', 'target_velocity', 'target_normal',
          'incoming_velocity', 'incoming_spin', 'intended_landing_xy', 'predicted_landing_xy',
          'outgoing_velocity')


def load_physics():
    path = ROOT/'source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/virtual_ball.py'
    spec = importlib.util.spec_from_file_location('complete_hold_ball_physics', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module, path


def legacy_physics_params(vb):
    """Keep recipe-19/20 provenance tied to its original venue contact/flight law."""
    return vb.load_venue_params(str(ROOT.parents[1] / 'configs/ball_physics_venue.yaml'))


def split_ids(ids):
    # All mutations of the same legacy source row remain in the same split.
    # The legacy deployed model has seen this original bank: these splits are
    # held out from the NEW training run, not claimed unseen by model11700.
    values = np.array([int.from_bytes(hashlib.sha256(f'complete-hold:{int(i)}'.encode()).digest()[:4],
                                    'little') % 10 for i in ids])
    return np.where(values < 8, 0, np.where(values == 8, 1, 2))


def qualify(a, vb, params):
    contact = a['target_position_offset'].clone()
    contact[:, :2] += a['station_xy']
    velocity, incoming, spin = a['target_velocity'], a['incoming_velocity'], a['incoming_spin']
    normal = velocity / velocity.norm(dim=-1, keepdim=True).clamp_min(1e-9)
    outgoing, out_spin = vb.predict_paddle_contact(incoming, velocity, normal, spin, params)
    land = vb.coarse_landing(contact, outgoing, out_spin, params, .76, 1.87, h=.005, n_steps=240)
    bounce = vb.coarse_landing(contact, incoming, spin, params, .76, 100., h=-.005, n_steps=160)
    xy, bxy = land['land_xy'], bounce['land_xy']
    relative_speed = ((incoming-velocity)*normal).sum(-1).abs()
    good = (land['land_valid'] & land['net_valid'] & (land['net_z'] > .9675)
            & (xy[:, 0] > 2.20) & (xy[:, 0] < 3.10) & (xy[:, 1].abs() < .65)
            & bounce['land_valid'] & (bxy[:, 0] > .52) & (bxy[:, 0] < 1.85)
            & (bxy[:, 1].abs() < .7425) & (incoming.norm(dim=-1) <= 7.)
            & (outgoing.norm(dim=-1) <= 7.) & (spin.norm(dim=-1) <= 15*2*np.pi)
            & (relative_speed >= 1.4) & (relative_speed <= 7.2)
            & (velocity.norm(dim=-1) <= 3.5))
    a.update(target_normal=normal, outgoing_velocity=outgoing,
             predicted_landing_xy=xy, intended_landing_xy=xy)
    return good


def summary(a):
    result = {}
    for side in (0, 1):
        mask = a['clip'] == side
        result[str(side)] = dict(count=int(mask.sum()))
        for key in ('station_xy', 'target_position_offset', 'target_velocity', 'incoming_velocity'):
            result[str(side)][key] = dict(min=a[key][mask].min(0).tolist(), max=a[key][mask].max(0).tolist())
        wide = mask & (a['source_kind'] == 1)
        result[str(side)]['wide_rows'] = int(wide.sum())
        result[str(side)]['station_y_histogram'] = np.histogram(
            a['station_xy'][wide, 1], [-.50, -.25, 0, .25, .50])[0].tolist()
    return result


def build(output, count=30000, seed=20261016, device='cpu'):
    if count < 1000:
        raise ValueError('at least 1000 augmented questions per side are required')
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f'refusing to overwrite bank evidence: {output}')
    source = ROOT/'assets/venue_tuple/v17_r12_physical_tuple_bank.npz'
    core = dict(np.load(source, allow_pickle=False))
    vb, physics_source = load_physics()
    params = legacy_physics_params(vb)
    rng = np.random.default_rng(seed)
    torch.manual_seed(seed)
    device = torch.device(device)
    # The old bank used different qualification margins. Recheck core rows
    # with exactly the same law/margins as the augmentations; being historical
    # is not evidence that a row is still valid for this bank.
    pieces = []
    core_qualified_count = 0
    with torch.inference_mode():
        for ids in np.array_split(np.arange(len(core['clip'])), 8):
            a = {k: torch.as_tensor(core[k][ids], device=device).clone() for k in FIELDS}
            good = qualify(a, vb, params)
            selected = torch.where(good)[0]
            item = {k: a[k][selected].cpu().numpy() for k in FIELDS}
            item.update(source_kind=np.zeros(len(selected), dtype=np.int16),
                        parent_id=ids[selected.cpu().numpy()])
            pieces.append(item)
            core_qualified_count += len(selected)
    attempts_by_side = {}
    with torch.inference_mode():
        for side in (0, 1):
            # Four station bands prevent rejection sampling from leaving a
            # seemingly large bank concentrated on the easy central stance.
            quota = [count//4]*4
            quota[-1] += count-sum(quota)
            have = [0]*4
            parents = np.where(core['clip'] == side)[0]
            attempts = 0
            for round_index in range(100):
                if have == quota:
                    break
                n = 16384
                ids = rng.choice(parents, n)
                a = {k: torch.as_tensor(core[k][ids], device=device).clone() for k in FIELDS}
                a['station_xy'][:, 0] = torch.rand(n, device=device)*.20-.10
                a['station_xy'][:, 1] = torch.rand(n, device=device)-.50
                # Heights mutate jointly around the source trajectory. Flight
                # validity is recomputed below, never inherited from the parent.
                a['target_position_offset'][:, 2] = (a['target_position_offset'][:, 2]
                    + torch.rand(n, device=device)*.24-.12).clamp(.85, 1.30)
                angle = (torch.rand(n, device=device)*2-1)*.18
                c, s = angle.cos(), angle.sin()
                scale = .85 + .30*torch.rand(n, device=device)
                for key in ('target_velocity', 'incoming_velocity', 'incoming_spin'):
                    x, y = a[key][:, 0].clone(), a[key][:, 1].clone()
                    a[key][:, 0], a[key][:, 1] = c*x-s*y, s*x+c*y
                    if key != 'incoming_spin':
                        a[key] *= scale[:, None]
                good = qualify(a, vb, params)
                bins = ((a['station_xy'][:, 1]+.5)*4).long().clamp(0, 3)
                for band in range(4):
                    selected = torch.where(good & (bins == band))[0][:quota[band]-have[band]]
                    if len(selected):
                        item = {k: a[k][selected].cpu().numpy() for k in FIELDS}
                        item.update(source_kind=np.ones(len(selected), dtype=np.int16),
                                    parent_id=ids[selected.cpu().numpy()])
                        pieces.append(item)
                        have[band] += len(selected)
                attempts += n
                if round_index % 5 == 0:
                    print(f'side={side} proposals={attempts} accepted_by_station_band={have}', flush=True)
            if have != quota:
                raise RuntimeError(f'coverage quotas cannot be physically filled: side={side}, {have}/{quota}')
            attempts_by_side[str(side)] = attempts
    arrays = {k: np.concatenate([a[k] for a in pieces]) for k in pieces[0]}
    split = split_ids(arrays['parent_id'])
    output.mkdir(parents=True, exist_ok=True)
    source_meta = dict(legacy_bank_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                       generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                       physics_source_sha256=hashlib.sha256(physics_source.read_bytes()).hexdigest(),
                       physics_yaml_sha256=hashlib.sha256(Path(params.source_path).read_bytes()).hexdigest())
    reports = {}
    for code, name in enumerate(('train', 'dev', 'test')):
        selected = np.where(split == code)[0]
        rng.shuffle(selected)
        data = {k: v[selected] for k, v in arrays.items()}
        path = output/f'{name}.npz'
        np.savez_compressed(path, schema_version=np.array(2), **data)
        receipt = dict(schema_version=2, split=name, seed=seed, source=source_meta,
            bank_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            contract=dict(recipe_revision=18, distribution='complete_hold_robust_v1',
                          qualification='training_contact_flight_and_incoming_bounce_v1',
                          station_inverse='contact_xy-minus-fixed-reach',
                          physics_integration_s=.005, min_net_center_z=.9675,
                          incoming_bounce_x=[.52, 1.85], incoming_bounce_y=[-.7425, .7425],
                          landing_x=[2.20, 3.10], landing_y=[-.65, .65]),
            rows=len(selected), unique_parents=len(np.unique(data['parent_id'])),
            core_input_rows=len(core['clip']), core_rows_requalified=core_qualified_count,
            summary=summary(data), proposal_counts=attempts_by_side,
            limitations=['core and augmented rows both pass the same ball-law qualification',
                         'new-run parent-disjoint splits; legacy model11700 saw the original bank',
                         'ball-law qualification only; dynamic A3 balance and real generalization NOT_PROVEN'])
        path.with_suffix('.receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
        reports[name] = dict(rows=len(selected), summary=receipt['summary'])
    parents = [set(arrays['parent_id'][split == i]) for i in range(3)]
    assert not (parents[0]&parents[1] or parents[0]&parents[2] or parents[1]&parents[2])
    (output/'coverage.json').write_text(json.dumps(reports, indent=2)+'\n')
    print(json.dumps({k: v['rows'] for k, v in reports.items()}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--count', type=int, default=30000, help='augmented rows per side, before splitting')
    parser.add_argument('--seed', type=int, default=20261016)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()
    build(args.output, args.count, args.seed, args.device)
