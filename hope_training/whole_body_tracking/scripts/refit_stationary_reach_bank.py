"""Regenerate a stationary bank with the YAML-selected full PPQ004 first-bounce model.

At-strike incoming states, contact points, parent IDs and split membership are
retained. Recipe 22 retains intended landings; optional --landing-objective
creates recipe 23 with fixed-centre targets and preserves the input landings.
Both recompute labels with shared paddle spin transfer, drag/gravity/Magnus
flight, and the physical first-bounce plane.
Both outgoing first-bounce and incoming reverse-flight checks use the physical center contact plane.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import yaml

# This offline executable does not use coverage instrumentation. Some Isaac
# environments bundle incompatible optional coverage and Numba versions.
sys.modules['coverage'] = None
from numba import njit, prange, set_num_threads

from ball_physics_profile import load_profile, profile_source_hashes
from sweet_spot_profile import load_objective, objective_sha256, objective_in_training_frame
from full_ball_model import (DT, FLIGHT_TIME, RADIUS, full_contact, first_bounce,
                             params_from_profile, plan, rk4_step)

RECIPE_REVISION = 22
QUALIFICATION = 'full_ppq004_first_bounce_v1'
DISTRIBUTION = 'stationary_reach_ppq004_full_v1'
SWEET_SPOT_RECIPE_REVISION = 23
SWEET_SPOT_DISTRIBUTION = 'stationary_reach_ppq004_sweet_spot_v1'


def params_from_yaml(path):
    """Return coefficients only through the shared YAML expansion and solver."""
    return params_from_profile(load_profile(path))


def table_offset_from_yaml(path):
    profile = load_profile(path)
    frame = profile['training_frame']
    return np.asarray([frame['near_x'], frame['y_center'] + profile['table']['width'] / 2,
                       frame['surface_z']], dtype=np.float64)


@njit
def incoming_bounce(x, v, w, params):
    """Reverse the same free-flight law to the previous center contact plane."""
    x, v = x.copy(), v.copy()
    for _ in range(int(round(.8 / params[DT]))):
        y, u = rk4_step(x, v, w, -params[DT], params)
        if x[2] > params[RADIUS] >= y[2]:
            f = (x[2]-params[RADIUS])/(x[2]-y[2])
            return x[:2]+f*(y[:2]-x[:2]), True
        x, v = y, u
    return np.zeros(2), False


@njit(parallel=True)
def retarget(x, incoming, spin, target, params, offset):
    """Run the shared full-physics inverse without clipping its command."""
    velocities, normals = np.zeros_like(x), np.zeros_like(x)
    outgoing, outgoing_spin = np.zeros_like(x), np.zeros_like(x)
    valid = np.zeros(len(x), dtype=np.bool_)
    for i in prange(len(x)):
        racket, normal, out, omega, bounce, time, ok, clears, bypasses = plan(
            x[i]-offset, incoming[i], spin[i], target[i]-offset[:2], params[FLIGHT_TIME], params)
        velocities[i], normals[i], outgoing[i], outgoing_spin[i] = racket, normal, out, omega
        valid[i] = ok and clears and not bypasses
    return velocities, normals, outgoing, outgoing_spin, valid


@njit(parallel=True)
def check(x, incoming, spin, racket, normals, target, params, offset):
    """Requalify stored float32 contact/flight through its physical first bounce."""
    result = np.empty((len(x), 9))
    outgoing, outgoing_spin = np.empty_like(x), np.empty_like(x)
    for i in prange(len(x)):
        out, omega = full_contact(incoming[i], racket[i], normals[i], spin[i], params)
        bounce, bounce_velocity, time, net, net_valid, valid = first_bounce(x[i]-offset, out, omega, params)
        xy = bounce[:2] + offset[:2]
        prior_bounce, prior_valid = incoming_bounce(x[i]-offset, incoming[i], spin[i], params)
        result[i,:2] = xy
        result[i,2] = net[2] + offset[2] if net_valid else -1e9
        result[i,3] = valid
        result[i,4:6] = prior_bounce + offset[:2]
        result[i,6] = prior_valid
        result[i,7] = np.linalg.norm(out)
        result[i,8] = np.linalg.norm(xy-target[i])
        outgoing[i], outgoing_spin[i] = out, omega
    return result, outgoing, outgoing_spin


def mask_valid(a, result, receipt, planner_valid=None, fit_tolerance=.003, landing_objective=None):
    pos, v, normal = a['target_position_offset'], a['target_velocity'], a['target_normal']
    speed = np.abs(np.sum((a['incoming_velocity']-v)*normal, axis=1))
    contract = receipt['contract']
    def inside(values, bounds):
        return (values > bounds[0]) & (values < bounds[1])
    criteria = {
        'landing': (result[:,3]>0) & inside(result[:,0], contract['landing_x']) & inside(result[:,1], contract['landing_y']),
        'net': result[:,2] > contract['min_net_center_z'],
        'incoming_bounce': (result[:,6]>0) & inside(result[:,4], contract['incoming_bounce_x']) & inside(result[:,5], contract['incoming_bounce_y']),
        'speed': (np.linalg.norm(a['incoming_velocity'],axis=1)<=7) & (result[:,7]<=7) & (np.linalg.norm(v,axis=1)<=3.5),
        'spin': np.linalg.norm(a['incoming_spin'],axis=1)<=15*2*np.pi,
        'paddle_approach': (speed>=1.4) & (speed<=7.2),
        'landing_fit': result[:,8]<=fit_tolerance,
        'finite': np.isfinite(result).all(1) & np.isfinite(v).all(1) & np.isfinite(normal).all(1),
        'reach': np.ones(len(v),dtype=bool),
        'planner': np.ones(len(v),dtype=bool) if planner_valid is None else planner_valid,
    }
    if landing_objective is not None:
        xr, yr = landing_objective['x_range'], landing_objective['y_range']
        # Inclusive rectangle qualification applies to the actual stored-command
        # first bounce, independently of distance from the commanded centre.
        criteria['landing_objective'] = ((result[:,0] >= xr[0]) & (result[:,0] <= xr[1])
                                         & (result[:,1] >= yr[0]) & (result[:,1] <= yr[1]))
    for side in (0,1):
        ids = a['clip']==side
        for key, values in [('reach_boxes',pos), ('velocity_boxes',v)]:
            box = np.array([contract[key][str(side)][axis] for axis in 'xyz'])
            criteria['reach'][ids] &= ((values[ids]>=box[:,0]-1e-6)&(values[ids]<=box[:,1]+1e-6)).all(1)
    return np.logical_and.reduce(list(criteria.values())), {key:int((~value).sum()) for key,value in criteria.items()}



def apply_landing_objective(arrays, objective):
    """Replace only target labels, retaining their original input provenance."""
    if objective is None:
        return arrays
    if 'input_intended_landing_xy' in arrays:
        raise ValueError('Input already contains a refit landing-target chain')
    original = arrays['intended_landing_xy']
    if original.ndim != 2 or original.shape[1] != 2 or original.dtype.kind != 'f':
        raise ValueError('Input intended_landing_xy must be an Nx2 floating-point array')
    result = dict(arrays)
    result['input_intended_landing_xy'] = original.copy()
    result['intended_landing_xy'] = np.broadcast_to(
        np.asarray(objective['target_xy'], dtype=original.dtype), original.shape).copy()
    return result


def retained_rows(arrays, accepted):
    """Preserve source-row provenance while adding the mapping into this input bank."""
    rows = dict(arrays)
    if 'source_row_index' in rows:
        if 'input_source_row_index' in rows:
            raise ValueError('Input already contains a refit source-index chain')
        rows['input_source_row_index'] = rows['source_row_index'].copy()
    rows['source_row_index'] = np.arange(len(accepted), dtype=np.int64)
    return {key: value[accepted] if value.ndim else value for key, value in rows.items()}

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _percentiles(values):
    return np.percentile(values, [50,95,100]).tolist() if len(values) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--physics',type=Path,required=True)
    parser.add_argument('--threads',type=int,default=12)
    parser.add_argument('--landing-objective', type=Path,
                        help='Optional fixed-centre table region YAML; selects sweet-spot recipe 23')
    args = parser.parse_args()
    set_num_threads(args.threads)
    if args.output.exists():
        raise FileExistsError(args.output)
    profile = load_profile(args.physics)
    params = params_from_profile(profile)
    offset = table_offset_from_yaml(args.physics)
    source_hashes = profile_source_hashes(args.physics)
    objective = load_objective(args.landing_objective) if args.landing_objective else None
    objective_hash = objective_sha256(args.landing_objective) if objective is not None else None
    objective_frame = objective_in_training_frame(objective, profile) if objective is not None else None
    recipe_revision = SWEET_SPOT_RECIPE_REVISION if objective is not None else RECIPE_REVISION
    refitter_hash = sha(__file__)
    profile_loader_hash = sha(Path(__file__).with_name("ball_physics_profile.py"))
    return_model_hash = sha(Path(__file__).with_name("full_ball_model.py"))
    args.output.mkdir(parents=True)
    summaries, parent_sets = {}, []
    for split in ('train','dev','test'):
        source = args.source/f'{split}.npz'
        old_receipt = json.loads(source.with_suffix('.receipt.json').read_text())
        if sha(source) != old_receipt['bank_sha256']:
            raise ValueError(f'Source bank checksum differs: {source}')
        with np.load(source,allow_pickle=False) as bank:
            a = {key:bank[key] for key in bank.files}
        a = apply_landing_objective(a, objective_frame)
        n = len(a['clip'])
        x = a['target_position_offset'].astype(np.float64)
        x[:,:2] += a['station_xy']
        incoming, spin = a['incoming_velocity'].astype(np.float64), a['incoming_spin'].astype(np.float64)
        old_velocity = a['target_velocity'].astype(np.float64)
        target = a['intended_landing_xy'].astype(np.float64)
        print(f'{split}: replanning {n} rows with full PPQ004 first-bounce physics',flush=True)
        velocity, normals, planned_outgoing, planned_spin, planner_valid = retarget(x,incoming,spin,target,params,offset)
        a['target_velocity'], a['target_normal'] = velocity.astype(np.float32), normals.astype(np.float32)
        # Validate the stored float32 command, as that is what training consumes.
        result, outgoing, outgoing_spin = check(x,incoming,spin,a['target_velocity'].astype(np.float64),
                                 a['target_normal'].astype(np.float64),target,params,offset)
        good, reasons = mask_valid(a,result,old_receipt,planner_valid,landing_objective=objective_frame)
        a['outgoing_velocity'] = outgoing.astype(np.float32)
        a['outgoing_spin'] = outgoing_spin.astype(np.float32)
        a['predicted_landing_xy'] = result[:,:2].astype(np.float32)
        retained = retained_rows(a, good)
        if not np.all(retained['station_xy']==0):
            raise ValueError('Source bank is not fixed-station')
        delta = np.linalg.norm(velocity-old_velocity,axis=1)
        summary = {'source_rows':n, 'retained_rows':int(good.sum()), 'rejected_rows':int((~good).sum()),
                   'failure_counts_overlapping':reasons,
                   'landing_error_m_p50_p95_max':_percentiles(result[good,8]),
                   'racket_velocity_change_mps_p50_p95_max':_percentiles(delta[good]),
                   'forward_vs_planned_velocity_mps_p50_p95_max':_percentiles(np.linalg.norm(outgoing[good]-planned_outgoing[good],axis=1)),
                   'sides':{}}
        for side in (0,1):
            ids = retained['clip']==side
            wide = ids & (retained['source_kind']==1)
            bounds = old_receipt['contract']['reach_boxes'][str(side)]['y']
            summary['sides'][str(side)] = {'rows':int(ids.sum()), 'core_rows':int((ids&~wide).sum()),
                'wide_rows':int(wide.sum()), 'wide_lateral_histogram':np.histogram(retained['target_position_offset'][wide,1],np.linspace(*bounds,5))[0].tolist()}
        summaries[split] = summary
        (args.output/'coverage.json').write_text(json.dumps(summaries,indent=2)+'\n')
        print(json.dumps(summary),flush=True)
        if split=='train' and any(summary['sides'][str(side)]['rows']<50000 for side in (0,1)):
            raise RuntimeError('Full PPQ004 qualification does not meet the original minimum 50000 train rows per side; bounds were not relaxed. See coverage.json.')
        if (profile_source_hashes(args.physics) != source_hashes
                or sha(__file__) != refitter_hash
                or sha(Path(__file__).with_name("ball_physics_profile.py")) != profile_loader_hash
                or sha(Path(__file__).with_name("full_ball_model.py")) != return_model_hash
                or (objective is not None and objective_sha256(args.landing_objective) != objective_hash)):
            raise RuntimeError("Physics, refitter or objective sources changed during bank generation; regenerate from stable inputs")
        path = args.output/f'{split}.npz'
        np.savez_compressed(path,**retained)
        np.savez_compressed(args.output/f'{split}.refit_audit.npz',accepted=good,source_row_index=np.arange(n),
                            result=result,planner_valid=planner_valid,velocity_delta=delta)
        receipt = {'schema_version':2, 'split':split, 'rows':int(good.sum()), 'bank_sha256':sha(path),
            'source':{'bank':str(source.resolve()), 'bank_sha256':sha(source), 'receipt_sha256':sha(source.with_suffix('.receipt.json')),
                      'bank_recipe_revision':old_receipt['contract']['recipe_revision'],
                      'bank_qualification':old_receipt['contract'].get('qualification'),
                      'physics_yaml':str(args.physics.resolve()), 'physics_yaml_sha256':sha(args.physics),
                      'physics_source_hashes':source_hashes, 'refitter_sha256':refitter_hash,
                      'profile_loader_sha256':profile_loader_hash,
                      'return_model_sha256':return_model_hash},
            'contract':dict(old_receipt['contract'], recipe_revision=recipe_revision, qualification=QUALIFICATION,
                            return_model=profile['return_model']['id'], physics_integration_s=float(params[DT]),
                            first_bounce_center_height_m=float(params[RADIUS]), training_frame=profile['training_frame']),
            'summary':summary, 'limitations':[
                'At-strike incoming velocity/spin held fixed; no observed pre-bounce state is available.',
                'Both directions use PPQ004 drag, gravity and Magnus; paddle contact transfers tangential impulse and spin.',
                'First bounce is descending center contact at table surface plus ball radius; no post-bounce trajectory is used.',
                'Original reach, speed, approach, landing, net and incoming-bounce bounds retained.',
                'Ball geometry only: policy performance, long-horizon balance and hardware qualification NOT_PROVEN.']}
        if objective is not None:
            receipt['source'].update(landing_objective_yaml=str(args.landing_objective.resolve()),
                                     landing_objective_yaml_sha256=objective_hash)
            receipt['contract'].update(landing_objective=objective, landing_objective_sha256=objective_hash,
                                       training_distribution_revision=SWEET_SPOT_DISTRIBUTION)
        path.with_suffix('.receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        parent_sets.append(set(retained['parent_id'].tolist()))
    if parent_sets[0]&parent_sets[1] or parent_sets[0]&parent_sets[2] or parent_sets[1]&parent_sets[2]:
        raise ValueError('Source split parent IDs overlap')
    train = args.output/'train.npz'
    receipt = train.with_suffix('.receipt.json')
    overrides = {'environment':{'HOPE_BALL_PHYSICS_YAML':str(args.physics.resolve())},'task':{'racket':{
        'venue_tuple_bank_path':str(train.resolve()), 'venue_tuple_bank_receipt_path':str(receipt.resolve()),
        'venue_tuple_bank_sha256':sha(train), 'venue_tuple_bank_receipt_sha256':sha(receipt),
        'venue_tuple_bank_recipe_revision':recipe_revision}}}
    if objective is not None:
        overrides['task']['landing_objective'] = {
            'path':str(args.landing_objective.resolve()), 'sha256':objective_hash, 'descriptor':objective}
    (args.output/'training_overrides.yaml').write_text(yaml.safe_dump(overrides,sort_keys=False))


if __name__=='__main__':
    main()
