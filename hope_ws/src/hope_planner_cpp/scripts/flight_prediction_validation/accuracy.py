"""Matched physical-flight scoring and figures; no fabricated one-shot curve."""
from collections import Counter, defaultdict
from pathlib import Path
import csv
import json
import math
import numpy as np
from scipy.stats import chi2

METHODS = ('legacy_one_shot', 'progressive_batch', 'v2_ckf')


def statistics(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return dict(n=0, mean=None, median=None, p90=None, p95=None, maximum=None, rmse=None)
    return dict(n=len(values), mean=float(np.mean(values)), median=float(np.median(values)),
                p90=float(np.quantile(values,.9)), p95=float(np.quantile(values,.95)),
                maximum=float(np.max(values)), rmse=float(np.sqrt(np.mean(values**2))))


def validate(records, synthetic=False):
    """Require comparable source cutoffs, identities and independently registered truth."""
    keys=defaultdict(set)
    seen=set()
    for row in records:
        for field in ('session_id','ball_id','flight_id','event_id','method','spin_mode',
                      'information_cutoff_ns','code_sha','config_id','frame_id','clock_mapping_id'):
            if field not in row:
                raise ValueError('missing physical provenance: '+field)
        if row['method'] not in METHODS:
            raise ValueError('unknown method')
        if not synthetic and not row.get('independent_truth',False):
            raise ValueError('independent truth is required; same-sensor proxy is not qualification')
        key=tuple(row[f] for f in ('session_id','ball_id','flight_id','event_id','information_cutoff_ns'))
        if (row['method'],key) in seen:
            raise ValueError('duplicate method/event/cutoff')
        seen.add((row['method'],key))
        keys[row['method']].add(key)
        reference=row.get('prediction_reference_time_ns')
        if reference is not None and reference>row['information_cutoff_ns']:
            raise ValueError('future source information exceeds matched cutoff')
        if row['method']=='legacy_one_shot' and row.get('event_estimate_valid',row['target_valid']) and not row.get('actual_trigger',False):
            raise ValueError('one-shot has no actual trigger; cannot invent a prediction')
        if row.get('event_estimate_valid',row['target_valid']) and row.get('truth_available',True):
            for field in ('position','truth_position','velocity','truth_velocity','arrival_ns','truth_arrival_ns'):
                if field not in row:
                    raise ValueError('missing eligible truth/prediction: '+field)
            for field in ('position','truth_position','velocity','truth_velocity'):
                if np.asarray(row[field]).shape!=(3,) or not np.isfinite(row[field]).all():
                    raise ValueError('invalid vector '+field)
            if reference is None:
                raise ValueError('prediction reference required')
    if set(keys)!=set(METHODS) or any(keys[m]!=keys[METHODS[0]] for m in METHODS):
        raise ValueError('methods must have identical physical events and information cutoffs, including missing predictions')


def residuals(records):
    result=[]
    for row in records:
        if not row.get('event_estimate_valid',row['target_valid']) or not row.get('truth_available',True):
            continue
        p=np.asarray(row['position'])-row['truth_position']
        v=np.asarray(row['velocity'])-row['truth_velocity']
        reference=row['prediction_reference_time_ns']
        value=dict(row, lead_ms=(row['truth_arrival_ns']-reference)/1e6,
                   error_x_m=float(p[0]),error_y_m=float(p[1]),error_z_m=float(p[2]),
                   error_position_m=float(np.linalg.norm(p)),
                   error_vx_mps=float(v[0]),error_vy_mps=float(v[1]),error_vz_mps=float(v[2]),
                   error_velocity_mps=float(np.linalg.norm(v)),
                   error_time_ms=(row['arrival_ns']-row['truth_arrival_ns'])/1e6)
        if row.get('post_velocity') is not None and row.get('truth_post_velocity') is not None:
            value['error_post_velocity_mps']=float(np.linalg.norm(np.asarray(row['post_velocity'])-row['truth_post_velocity']))
        value['conditional_time_standardized']=None
        covariance=row.get('covariance')
        value['conditional_position_mahalanobis']=None
        value['conditional_velocity_mahalanobis']=None
        if covariance is not None:
            cov=np.asarray(covariance)
            if cov.shape not in ((6,6),(7,7)) or not np.isfinite(cov).all():
                raise ValueError('invalid conditional covariance shape/value')
            if not np.allclose(cov,cov.T,atol=1e-9) or np.linalg.eigvalsh(cov).min() < -1e-10:
                raise ValueError('conditional covariance must be symmetric PSD')
            if cov.shape==(7,7) and cov[6,6]>1e-18:
                value['conditional_time_standardized']=(row['arrival_ns']-row['truth_arrival_ns'])*1e-9/math.sqrt(cov[6,6])
            for offset,delta,name in [(0,p,'position'),(3,v,'velocity')]:
                marginal=cov[offset:offset+3,offset:offset+3]
                if np.linalg.eigvalsh(marginal).min()>1e-15:
                    value['conditional_'+name+'_mahalanobis']=float(delta@np.linalg.solve(marginal,delta))
        result.append(value)
    return result


def group_statistics(records, errors):
    """Keep truth-missing, physical, full-trajectory and route availability separate."""
    result={}
    fields=('error_position_m','error_y_m','error_z_m','error_velocity_mps','error_vx_mps',
            'error_vy_mps','error_vz_mps','error_time_ms','error_post_velocity_mps')
    for method in METHODS:
        for spin in sorted({r['spin_mode'] for r in records if r['method']==method}):
            rows=[r for r in records if r['method']==method and r['spin_mode']==spin]
            scored=[r for r in errors if r['method']==method and r['spin_mode']==spin]
            group=dict(attempts=len(rows),actual_predictions=sum(r.get('attempted',True) for r in rows),
                       scored=len(scored),missing_truth=sum(not r.get('truth_available',True) for r in rows),
                       physical_coverage=sum(r.get('physical_available',False) for r in rows)/len(rows),
                       target_coverage=sum(r['target_valid'] for r in rows)/len(rows),
                       event_estimate_coverage=sum(r.get('event_estimate_valid',r['target_valid']) for r in rows)/len(rows),
                       trajectory_coverage=sum(r.get('trajectory_available',False) for r in rows)/len(rows),
                       rejection_reasons=dict(Counter(r.get('reason','unavailable') for r in rows if not r['target_valid'])))
            group['metrics']={field:dict(signed=statistics([r.get(field,math.nan) for r in scored]),
                                       absolute=statistics([abs(r.get(field,math.nan)) for r in scored])) for field in fields}
            group['compute_ms']=statistics([r['compute_ms'] for r in rows if r.get('compute_ms') is not None])
            for marginal in ('position','velocity'):
                values=[r['conditional_'+marginal+'_mahalanobis'] for r in scored
                        if r['conditional_'+marginal+'_mahalanobis'] is not None]
                group['conditional_'+marginal+'_coverage']={str(level):float(np.mean(np.asarray(values)<=chi2.ppf(level,3)))
                                                           if values else None for level in (.68,.95)}
            standardized=[r['conditional_time_standardized'] for r in scored if r['conditional_time_standardized'] is not None]
            group['conditional_time_coverage']={str(level):float(np.mean(np.square(standardized)<=chi2.ppf(level,1))) if standardized else None for level in (.68,.95)}
            first={}
            for row in rows:
                key=(row['session_id'],row['ball_id'],row['flight_id'],row['event_id'])
                first.setdefault(key,dict(physical=None,target=None,truth=row.get('truth_arrival_ns')))
                for kind,available in [('physical',row.get('physical_available',False)),('target',row['target_valid'])]:
                    reference=row.get('prediction_reference_time_ns')
                    if available and reference is not None:
                        first[key][kind]=reference if first[key][kind] is None else min(first[key][kind],reference)
            for kind in ('physical','target'):
                group['first_'+kind+'_source_lead_ms']=statistics([(r['truth']-r[kind])/1e6 for r in first.values() if r['truth'] is not None and r[kind] is not None])
            result[method+'/'+spin]=group
    return result


def paired_bootstrap(records, repetitions=500, unit='shot'):
    """Use identical whole-shot/session draws for every method, never frame resampling."""
    def identity(row):
        return (row['session_id'],) if unit=='session' else tuple(row[k] for k in ('session_id','ball_id','flight_id'))
    units=sorted({identity(r) for r in records})
    grouped={m:{u:[r for r in records if r['method']==m and identity(r)==u] for u in units} for m in METHODS}
    values={m:[] for m in METHODS}
    rng=np.random.default_rng(0)
    for _ in range(repetitions):
        indices=rng.integers(0,len(units),len(units))
        for method in METHODS:
            rows=[r for i in indices for r in grouped[method][units[i]]]
            values[method].append(np.mean([r['target_valid'] for r in rows]))
    return dict(unit=unit,groups=len(units),repetitions=repetitions,
                target_coverage_95={m:[float(v) for v in np.quantile(x,[.025,.975])] for m,x in values.items()},
                v2_minus_progressive_coverage_95=[float(v) for v in np.quantile(
                    np.asarray(values['v2_ckf'])-values['progressive_batch'],[.025,.975])])


def write_report(records, output, synthetic=False, repetitions=500):
    validate(records,synthetic)
    errors=residuals(records)
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    strata=defaultdict(list)
    for row in errors:
        lead_bin=int(math.floor(row['lead_ms']/50))*50
        b1=row.get('b1_ns')
        phase='unknown' if b1 is None else ('pre_B1' if row['prediction_reference_time_ns']<b1 else
                'B1_B2' if row.get('b2_ns') is None or row['prediction_reference_time_ns']<row['b2_ns'] else 'post_B2')
        key=(row['method'],row['spin_mode'],row.get('event_kind','receiving_plane'),phase,lead_bin)
        strata[key].append(row)
    summary=dict(gate_p='NOT_RUN_SYNTHETIC_TOOLING' if synthetic else 'SCORED_INDEPENDENT_DATA',
                 uncertainty='conditional p/v/event covariance only; auxiliary spin/model/plane/clock omissions',
                 groups=group_statistics(records,errors),paired_bootstrap=paired_bootstrap(records,repetitions),
                 strata=[dict(method=k[0],spin_mode=k[1],event_kind=k[2],phase=k[3],lead_bin_ms=k[4],
                              n=len(v),flights=len({(r['session_id'],r['flight_id']) for r in v}),
                              position=statistics([r['error_position_m'] for r in v]),
                              velocity=statistics([r['error_velocity_mps'] for r in v]),
                              arrival_ms=statistics([abs(r['error_time_ms']) for r in v])) for k,v in strata.items()])
    (output/'tables.json').write_text(json.dumps(summary,indent=2)+'\n')
    fields=['session_id','ball_id','flight_id','method','spin_mode','prediction_reference_time_ns','lead_ms',
            'error_x_m','error_y_m','error_z_m','error_position_m','error_vx_mps','error_vy_mps','error_vz_mps','error_velocity_mps','error_time_ms']
    with (output/'errors.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(errors)
    figure_groups=defaultdict(list)
    for row in records:
        figure_groups[(row['spin_mode'],row.get('event_kind','receiving_plane'),row.get('shot_type','unknown'))].append(row)
    summary['figure_groups']=[]
    for key,rows in figure_groups.items():
        directory=output/('_'.join(str(x).replace('/','_') for x in key))
        directory.mkdir(exist_ok=True)
        selected=[r for r in errors if (r['spin_mode'],r.get('event_kind','receiving_plane'),r.get('shot_type','unknown'))==key]
        figures(rows,selected,directory,synthetic)
        summary['figure_groups'].append(dict(spin=key[0],event_kind=key[1],shot_type=key[2],directory=directory.name))
    (output/'tables.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def figures(records, errors, output, synthetic):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    label='SYNTHETIC TOOLING TEST — NOT REAL-BALL ACCURACY' if synthetic else 'Matched source cutoffs; independent registered truth'
    def scatter(ax, rows, x, y, method):
        ax.scatter([x(r) for r in rows],[y(r) for r in rows],s=10,alpha=.55,
                   label=f'{method} (n={len(rows)}, shots={len({(r["session_id"],r["flight_id"]) for r in rows})})')
    fig,axes=plt.subplots(2,3,figsize=(14,8))
    for method in METHODS:
        rows=[r for r in errors if r['method']==method]
        for ax,field,title in zip(axes.flat,['error_y_m','error_z_m','error_position_m','error_velocity_mps','error_time_ms','error_time_ms'],
                                  ['Y error (m)','Z error (m)','3D position error (m)','Velocity error (m/s)','Signed arrival error (ms)','Absolute arrival error (ms)']):
            scatter(ax,rows,lambda r:r['lead_ms'],lambda r:abs(r[field]) if title.startswith('Absolute') else r[field],method)
            ax.set(xlabel='True event lead at actual prediction (ms)',ylabel=title)
    axes.flat[0].legend(fontsize=7);fig.suptitle(label+'\nOne-shot: actual trigger points only; no interpolated curve');fig.tight_layout()
    fig.savefig(output/'01_plane_error_vs_lead.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for ax,bounce in zip(axes,['b1_ns','b2_ns']):
        for method in METHODS:
            rows=[r for r in errors if r['method']==method and r.get(bounce) is not None]
            scatter(ax,rows,lambda r:(r['prediction_reference_time_ns']-r[bounce])/1e6,lambda r:r['error_position_m'],method)
        ax.axvline(0,color='black',linewidth=.6);ax.set(xlabel='Prediction time relative to '+bounce[:2].upper()+' (ms)',ylabel='Plane position error (m)')
    axes[0].legend(fontsize=7);fig.suptitle(label);fig.tight_layout();fig.savefig(output/'02_bounce_aligned_error.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    for method in METHODS:
        rows=[r for r in records if r['method']==method]
        bins=defaultdict(list)
        for r in rows:
            if 'truth_arrival_ns' in r:
                reference=r.get('prediction_reference_time_ns') or r['information_cutoff_ns']
                bins[round((r['truth_arrival_ns']-reference)/50e6)*50].append(r)
        axes[0].plot(sorted(bins),[np.mean([r['target_valid'] for r in bins[k]]) for k in sorted(bins)],marker='o',label=method)
        latency=sorted(r['compute_ms'] for r in rows if r.get('compute_ms') is not None)
        if latency: axes[1].plot(latency,np.arange(1,len(latency)+1)/len(latency),label=method)
        distances=[r['conditional_position_mahalanobis'] for r in errors if r['method']==method and r['conditional_position_mahalanobis'] is not None]
        if distances:
            levels=np.linspace(.05,.99,20);axes[2].plot(levels,[np.mean(np.asarray(distances)<=chi2.ppf(x,3)) for x in levels],label=method)
    axes[0].set(xlabel='Matched event lead bin (ms)',ylabel='Target coverage (all eligible cutoffs)')
    axes[1].set(xlabel='Offline prediction compute time (ms)',ylabel='Empirical CDF')
    axes[2].plot([0,1],[0,1],color='gray',linestyle='--');axes[2].set(xlabel='Nominal conditional position coverage',ylabel='Empirical coverage')
    axes[0].legend(fontsize=7);fig.suptitle(label+'\nStratified spin/phase/sample counts in tables.json; no full uncertainty calibration claim');fig.tight_layout()
    fig.savefig(output/'03_coverage_latency_uncertainty.png',dpi=150);plt.close(fig)
