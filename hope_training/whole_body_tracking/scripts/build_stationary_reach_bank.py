"""Build the stationary-rally question bank: one immutable station, wide arm reach.

Every row commands the session anchor (station_xy == 0). The lateral, height
and depth variety of the incoming ball is expressed as a station-relative
contact offset inside the per-side reach boxes, never as a base step. Rows are
correlated mutations of complete legacy contact tuples, re-qualified with the
same training ball law, incoming bounce and outgoing net/landing checks as
``build_complete_hold_bank.py``. Core rows (source_kind=0) keep the original
reach centers (0.58, -0.44) / (0.58, -0.09); wide rows (source_kind=1) fill
four lateral bands per side so the curriculum cannot hide at the easy center.

This qualifies ball geometry only. Dynamic A3 reach/balance feasibility at the
box edges is NOT proven here; the training metrics per lateral band decide it.
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
DISTRIBUTION = 'stationary_reach_v1'
RECIPE_REVISION = 19
# Station-relative contact boxes, FH then BH: x depth, y lateral, z above ground.
# FH stays strictly on the negative side of BH so the planner split is unambiguous.
DEFAULT_BOXES = {
    0: dict(x=(0.44, 0.70), y=(-0.68, -0.24), z=(0.80, 1.45)),
    1: dict(x=(0.44, 0.70), y=(-0.24, 0.28), z=(0.80, 1.45)),
}
# Final racket-velocity export envelope (must contain every row; the task YAML
# vel_range_per_clip must be at least this wide).
VELOCITY_BOXES = {
    0: dict(x=(1.00, 2.90), y=(-0.80, 1.20), z=(0.30, 2.00)),
    1: dict(x=(1.10, 2.90), y=(-1.00, 0.90), z=(0.00, 1.80)),
}
# Correlated mutation strength around each legacy parent row. v4 adds what the
# deployed planner produces but the legacy venue sampler never did: flat/downward
# racket strokes (independent vz scale), aim-at-table-centre lateral racket
# velocity (vy tied to the lateral contact offset), and faster incoming balls.
MUTATION = dict(rotation_rad=0.25, speed_scale=(0.70, 1.30), incoming_scale=(0.70, 1.50),
                spin_scale=(0.50, 1.50), z_jitter=0.20, vz_scale=(0.0, 1.2),
                aim_probability=0.6, aim_gain=(0.6, 1.2), aim_noise=0.12, aim_depth_m=2.0)
# Jitter applied to real/simulated planner targets used as parents.
TRACE_JITTER = dict(position=0.03, velocity=0.10)


def load_legacy_builder():
    path = ROOT / 'scripts/build_complete_hold_bank.py'
    spec = importlib.util.spec_from_file_location('complete_hold_bank_builder', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module, path


def box_tensor(box, device):
    return torch.tensor([box['x'], box['y'], box['z']], dtype=torch.float32, device=device)


def inside(values, box_t):
    return ((values >= box_t[:, 0] - 1e-6) & (values <= box_t[:, 1] + 1e-6)).all(-1)


def apply_stroke_shape_mutation(a, n, device):
    """Flat strokes and centre-aimed lateral racket velocity, before re-qualification."""
    vlo, vhi = MUTATION['vz_scale']
    a['target_velocity'][:, 2] *= vlo + (vhi - vlo) * torch.rand(n, device=device)
    aim = torch.rand(n, device=device) < MUTATION['aim_probability']
    glo, ghi = MUTATION['aim_gain']
    gain = glo + (ghi - glo) * torch.rand(n, device=device)
    # The outgoing direction follows the racket velocity direction (face normal
    # is the velocity direction): aim the horizontal stroke from the contact
    # point toward the table centre line, aim_depth_m ahead, with noise.
    y_contact = a['target_position_offset'][:, 1]
    aimed_vy = -gain * y_contact * a['target_velocity'][:, 0] / MUTATION['aim_depth_m']
    aimed_vy = aimed_vy + MUTATION['aim_noise'] * torch.randn(n, device=device)
    a['target_velocity'][:, 1] = torch.where(aim, aimed_vy, a['target_velocity'][:, 1])


def load_trace_parents(paths):
    """Planner targets from runner traces: (side, offset xyz, racket velocity xyz)."""
    import csv
    rows, seen = [], set()
    for path in paths:
        for r in csv.DictReader(open(path)):
            try:
                if not r.get('target_x') or float(r['target_x']) == 0.0:
                    continue
                key = (str(path), r.get('frozen_flight_id') or r.get('planner_flight_id'),
                       round(float(r['target_x']), 3), round(float(r['target_y']), 3))
                if key in seen:
                    continue
                seen.add(key)
                side = 0 if float(r.get('swing_sign') or 1.0) > 0 else 1
                rows.append([side, float(r['target_x']) - float(r['base_x']), float(r['target_y']) - float(r['base_y']),
                             float(r['target_z']), float(r['target_vx']), float(r['target_vy']), float(r['target_vz'])])
            except (ValueError, KeyError):
                continue
    return np.array(rows, dtype=np.float32).reshape(-1, 7)


def qualify_rows(a, legacy, vb, params, box, vbox):
    good = legacy.qualify(a, vb, params)
    return good & inside(a['target_position_offset'], box) & inside(a['target_velocity'], vbox)


def build(output, count=40000, seed=20261018, device='cpu', boxes=DEFAULT_BOXES, trace_paths=(),
          trace_count=0):
    if count < 4000:
        raise ValueError('at least 4000 augmented questions per side are required')
    for side in (0, 1):
        for axis in 'xyz':
            lo, hi = boxes[side][axis]
            if not lo < hi:
                raise ValueError(f'invalid reach box side={side} axis={axis}')
    if not boxes[0]['y'][1] <= boxes[1]['y'][0]:
        raise ValueError('forehand lateral band must end where the backhand band starts, never overlap')
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f'refusing to overwrite bank evidence: {output}')
    legacy, builder_path = load_legacy_builder()
    source = ROOT / 'assets/venue_tuple/v17_r12_physical_tuple_bank.npz'
    core = dict(np.load(source, allow_pickle=False))
    vb, physics_source = legacy.load_physics()
    params = legacy.legacy_physics_params(vb)
    rng = np.random.default_rng(seed)
    torch.manual_seed(seed)
    device = torch.device(device)
    FIELDS = legacy.FIELDS
    pieces = []
    core_qualified = {0: 0, 1: 0}
    with torch.inference_mode():
        # Core: the original reach centers, now answered from the anchor. The
        # contact therefore moves by the old station; flight validity is recomputed.
        for ids in np.array_split(np.arange(len(core['clip'])), 8):
            a = {k: torch.as_tensor(core[k][ids], device=device).clone() for k in FIELDS}
            a['station_xy'].zero_()
            good = legacy.qualify(a, vb, params)
            for side in (0, 1):
                good &= ~(a['clip'] == side) | (
                    inside(a['target_position_offset'], box_tensor(boxes[side], device))
                    & inside(a['target_velocity'], box_tensor(VELOCITY_BOXES[side], device)))
            selected = torch.where(good)[0]
            item = {k: a[k][selected].cpu().numpy() for k in FIELDS}
            item.update(source_kind=np.zeros(len(selected), dtype=np.int16),
                        parent_id=ids[selected.cpu().numpy()])
            pieces.append(item)
            for side in (0, 1):
                core_qualified[side] += int((item['clip'] == side).sum())
    attempts_by_side = {}
    with torch.inference_mode():
        for side in (0, 1):
            box = box_tensor(boxes[side], device)
            vbox = box_tensor(VELOCITY_BOXES[side], device)
            quota = [count // 4] * 4
            quota[-1] += count - sum(quota)
            have = [0] * 4
            parents = np.where(core['clip'] == side)[0]
            attempts = 0
            y_lo, y_hi = boxes[side]['y']
            for round_index in range(200):
                if have == quota:
                    break
                n = 16384
                ids = rng.choice(parents, n)
                a = {k: torch.as_tensor(core[k][ids], device=device).clone() for k in FIELDS}
                a['station_xy'].zero_()
                a['target_position_offset'][:, 0] = box[0, 0] + torch.rand(n, device=device) * (box[0, 1] - box[0, 0])
                a['target_position_offset'][:, 1] = box[1, 0] + torch.rand(n, device=device) * (box[1, 1] - box[1, 0])
                zj = MUTATION['z_jitter']
                a['target_position_offset'][:, 2] = (a['target_position_offset'][:, 2]
                    + torch.rand(n, device=device) * 2 * zj - zj).clamp(box[2, 0], box[2, 1])
                angle = (torch.rand(n, device=device) * 2 - 1) * MUTATION['rotation_rad']
                c, s = angle.cos(), angle.sin()
                lo, hi = MUTATION['speed_scale']
                scale = lo + (hi - lo) * torch.rand(n, device=device)
                slo, shi = MUTATION['spin_scale']
                spin_scale = slo + (shi - slo) * torch.rand(n, device=device)
                ilo, ihi = MUTATION['incoming_scale']
                incoming_scale = ilo + (ihi - ilo) * torch.rand(n, device=device)
                for key in ('target_velocity', 'incoming_velocity', 'incoming_spin'):
                    x, y = a[key][:, 0].clone(), a[key][:, 1].clone()
                    a[key][:, 0], a[key][:, 1] = c * x - s * y, s * x + c * y
                    factor = {'target_velocity': scale, 'incoming_velocity': incoming_scale,
                              'incoming_spin': spin_scale}[key]
                    a[key] *= factor[:, None]
                apply_stroke_shape_mutation(a, n, device)
                good = legacy.qualify(a, vb, params) & inside(a['target_velocity'], vbox)
                bins = ((a['target_position_offset'][:, 1] - y_lo) / (y_hi - y_lo) * 4).long().clamp(0, 3)
                for band in range(4):
                    selected = torch.where(good & (bins == band))[0][:quota[band] - have[band]]
                    if len(selected):
                        item = {k: a[k][selected].cpu().numpy() for k in FIELDS}
                        item.update(source_kind=np.ones(len(selected), dtype=np.int16),
                                    parent_id=ids[selected.cpu().numpy()])
                        pieces.append(item)
                        have[band] += len(selected)
                attempts += n
                if round_index % 5 == 0:
                    print(f'side={side} proposals={attempts} accepted_by_lateral_band={have}', flush=True)
            if have != quota:
                raise RuntimeError(f'coverage quotas cannot be physically filled: side={side}, {have}/{quota}')
            attempts_by_side[str(side)] = attempts
    trace_report = {}
    if trace_paths:
        # Real/simulated planner targets as parents: keep their offset and racket
        # velocity (small jitter), borrow a physically consistent incoming ball
        # from the nearest legacy row of the same side, then re-qualify.
        trace = load_trace_parents(trace_paths)
        trace_report['unique_targets'] = int(len(trace))
        with torch.inference_mode():
            for side in (0, 1):
                tr = trace[trace[:, 0] == side]
                if not len(tr) or trace_count <= 0:
                    continue
                box = box_tensor(boxes[side], device)
                vbox = box_tensor(VELOCITY_BOXES[side], device)
                parents = np.where(core['clip'] == side)[0]
                legacy_rows = np.c_[core['target_position_offset'][parents, 2], core['target_velocity'][parents]]
                accepted, attempts = 0, 0
                for _ in range(100):
                    if accepted >= trace_count:
                        break
                    n = 8192
                    pick = rng.integers(len(tr), size=n)
                    t = tr[pick]
                    q = np.c_[t[:, 3], t[:, 4:7]]
                    d = ((legacy_rows[None, :, :] - q[:, None, :]) ** 2 * np.array([4., 1., 1., 1.])).sum(-1)
                    ids = parents[d.argmin(1)]
                    a = {k: torch.as_tensor(core[k][ids], device=device).clone() for k in FIELDS}
                    a['station_xy'].zero_()
                    a['target_position_offset'] = (torch.as_tensor(t[:, 1:4], device=device)
                        + (torch.rand(n, 3, device=device) * 2 - 1) * TRACE_JITTER['position'])
                    a['target_position_offset'][:, 2].clamp_(box[2, 0], box[2, 1])
                    a['target_velocity'] = (torch.as_tensor(t[:, 4:7], device=device)
                        + (torch.rand(n, 3, device=device) * 2 - 1) * TRACE_JITTER['velocity'])
                    good = qualify_rows(a, legacy, vb, params, box, vbox)
                    selected = torch.where(good)[0][:trace_count - accepted]
                    if len(selected):
                        item = {k: a[k][selected].cpu().numpy() for k in FIELDS}
                        item.update(source_kind=np.ones(len(selected), dtype=np.int16),
                                    parent_id=-(1 + pick[selected.cpu().numpy()]).astype(np.int64))
                        pieces.append(item)
                        accepted += len(selected)
                    attempts += n
                trace_report[str(side)] = dict(accepted=int(accepted), attempts=int(attempts))
                print(f'trace side={side} accepted={accepted} attempts={attempts}', flush=True)
    arrays = {k: np.concatenate([a[k] for a in pieces]) for k in pieces[0]}
    assert np.all(arrays['station_xy'] == 0)
    split = legacy.split_ids(arrays['parent_id'])
    output.mkdir(parents=True, exist_ok=True)
    source_meta = dict(legacy_bank_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                       generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                       legacy_generator_sha256=hashlib.sha256(builder_path.read_bytes()).hexdigest(),
                       physics_source_sha256=hashlib.sha256(physics_source.read_bytes()).hexdigest(),
                       physics_yaml_sha256=hashlib.sha256(Path(params.source_path).read_bytes()).hexdigest())
    reports = {}
    for code, name in enumerate(('train', 'dev', 'test')):
        selected = np.where(split == code)[0]
        rng.shuffle(selected)
        data = {k: v[selected] for k, v in arrays.items()}
        path = output / f'{name}.npz'
        np.savez_compressed(path, schema_version=np.array(2), **data)
        summary = {}
        for side in (0, 1):
            mask = data['clip'] == side
            wide = mask & (data['source_kind'] == 1)
            y_lo, y_hi = boxes[side]['y']
            summary[str(side)] = dict(
                count=int(mask.sum()), core_rows=int((mask & (data['source_kind'] == 0)).sum()),
                wide_rows=int(wide.sum()),
                target_position_offset=dict(min=data['target_position_offset'][mask].min(0).tolist(),
                                            max=data['target_position_offset'][mask].max(0).tolist()),
                target_velocity=dict(min=data['target_velocity'][mask].min(0).tolist(),
                                     max=data['target_velocity'][mask].max(0).tolist()),
                lateral_histogram=np.histogram(data['target_position_offset'][wide, 1],
                                               np.linspace(y_lo, y_hi, 5))[0].tolist())
        receipt = dict(schema_version=2, split=name, seed=seed, source=source_meta,
            bank_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            contract=dict(recipe_revision=RECIPE_REVISION, distribution=DISTRIBUTION,
                          station='fixed_session_anchor_xy_zero',
                          qualification='training_contact_flight_and_incoming_bounce_v1',
                          reach_boxes={str(k): v for k, v in boxes.items()},
                          velocity_boxes={str(k): v for k, v in VELOCITY_BOXES.items()},
                          mutation=MUTATION, trace_jitter=TRACE_JITTER,
                          trace_sources=[str(p) for p in trace_paths],
                          physics_integration_s=.005, min_net_center_z=.9675,
                          incoming_bounce_x=[.52, 1.85], incoming_bounce_y=[-.7425, .7425],
                          landing_x=[2.20, 3.10], landing_y=[-.65, .65]),
            rows=len(selected), unique_parents=len(np.unique(data['parent_id'])),
            core_input_rows=len(core['clip']), core_rows_requalified=core_qualified,
            summary=summary, proposal_counts=attempts_by_side, trace_parents=trace_report,
            limitations=['every row commands station_xy == 0; the robot is never asked to step',
                         'ball-law qualification only; A3 reach/balance at the box edges NOT_PROVEN',
                         'new-run parent-disjoint splits; legacy models saw the original centers'])
        path.with_suffix('.receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
        reports[name] = dict(rows=len(selected), summary=summary)
    parents = [set(arrays['parent_id'][split == i]) for i in range(3)]
    assert not (parents[0] & parents[1] or parents[0] & parents[2] or parents[1] & parents[2])
    (output / 'coverage.json').write_text(json.dumps(reports, indent=2) + '\n')
    print(json.dumps({k: v['rows'] for k, v in reports.items()}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--count', type=int, default=40000, help='augmented rows per side, before splitting')
    parser.add_argument('--seed', type=int, default=20261018)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--fh-y', type=float, nargs=2, default=DEFAULT_BOXES[0]['y'], help='forehand lateral band')
    parser.add_argument('--bh-y', type=float, nargs=2, default=DEFAULT_BOXES[1]['y'], help='backhand lateral band')
    parser.add_argument('--x', type=float, nargs=2, default=DEFAULT_BOXES[0]['x'], help='depth band, both sides')
    parser.add_argument('--z', type=float, nargs=2, default=DEFAULT_BOXES[0]['z'], help='height band, both sides')
    parser.add_argument('--parents-trace', type=Path, nargs='*', default=(),
                        help='pp_runner_trace.csv files whose planner targets become extra parents')
    parser.add_argument('--trace-count', type=int, default=0, help='accepted trace-derived rows per side')
    args = parser.parse_args()
    boxes = {0: dict(x=tuple(args.x), y=tuple(args.fh_y), z=tuple(args.z)),
             1: dict(x=tuple(args.x), y=tuple(args.bh_y), z=tuple(args.z))}
    build(args.output, args.count, args.seed, args.device, boxes, tuple(args.parents_trace), args.trace_count)
