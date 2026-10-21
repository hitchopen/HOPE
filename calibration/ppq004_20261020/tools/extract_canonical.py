"""
C3D -> canonical npz extractor for the 2026-07-03 venue dataset.

Reads an Avatar Pro C3D export containing:
  ball   : 15-marker rigid body   (b_Ball_* | b_PPQ-15_* | b_PPQ_1_*)
  paddle1: 4-marker rigid body    (b_PPP1_* | b_PPP_1_*)
  paddle2: 4-marker rigid body    (b_PPP2_* | b_PPP_2_*)
  table  : 6 static markers       (b_PPT_* | b_table_*)  = 4 corners + 2 net posts

Outputs one <take>.npz (SI: meters, seconds) + <take>_manifest.json:
  - canonical TABLE frame built from the 4 corner markers
      origin = table center, X = length (2.740 axis), Y = width, Z = up
  - ball centroid corrected to true sphere center (sphere fit on rigid template),
    orientation quaternion (Kabsch vs template) -- valid only relatively
  - both paddles: centroid + quaternion + face normal per frame
  - trimming: leading/trailing TRIM_S seconds cut, plus leading/trailing
    untracked-ball spans; original frame indices & timestamps preserved
  - below-table mask: ball center z_table < BELOW_TABLE_M => sample invalidated
    (positions NaN'd) but the time axis is preserved

Usage: python extract_canonical.py <take_dir_or_c3d> <out_dir>
  If a directory is given and split per-rigid-body exports exist
  (*_Ball.c3d etc.), they are used instead of the combined file (much faster).
"""
import os, sys, json, glob, warnings
import numpy as np

warnings.filterwarnings("ignore")
import c3d
from scipy.spatial.transform import Rotation as Rot

BALL_PREFIXES = ["b_Ball_", "b_PPQ-15_", "b_PPQ_1_"]
P1_PREFIXES = ["b_PPP1_", "b_PPP_1_"]
P2_PREFIXES = ["b_PPP2_", "b_PPP_2_"]
TAB_PREFIXES = ["b_PPT_", "b_table_"]

TRIM_S = 2.0            # cut this much from each end of the take
BELOW_TABLE_M = 0.0     # ball CENTER below table plane => cut (on-table contact is at +R=+0.020)
MM = 1e-3

ITTF_LEN, ITTF_WID = 2.740, 1.525


def _find_prefix(labels, candidates):
    for p in candidates:
        if any(l.startswith(p) for l in labels):
            return p
    return None


def read_groups(path, want):
    """Read only the wanted label groups from a c3d. want: dict name->prefix list.
    Returns dict name -> (nf, M, 3) array (NaN where absent), plus rate/nf."""
    with open(path, "rb") as f:
        r = c3d.Reader(f)
        labels = [l.split("\x00")[0].strip() for l in r.point_labels]
        rate = float(r.point_rate)
        nf_hdr = r.last_frame - r.first_frame + 1
        groups, idxs = {}, {}
        for name, prefs in want.items():
            p = _find_prefix(labels, prefs)
            if p is None:
                continue
            idx = [i for i, l in enumerate(labels) if l.startswith(p)]
            idx.sort(key=lambda i: int(labels[i].rsplit("_", 1)[1]))
            idxs[name] = np.array(idx, int)
            groups[name] = np.full((nf_hdr, len(idx), 3), np.nan)
        k = 0
        for fn, pts, an in r.read_frames():
            if k >= nf_hdr:
                break
            for name, idx in idxs.items():
                ok = pts[idx, 3] >= 0
                groups[name][k][ok] = pts[idx][ok, :3]
            k += 1
    for name in groups:
        groups[name] = groups[name][:k]
    return groups, rate, k


def load_take(src):
    """src: dir with split exports, or a single combined c3d path."""
    want = dict(ball=BALL_PREFIXES, p1=P1_PREFIXES, p2=P2_PREFIXES, tab=TAB_PREFIXES)
    if os.path.isdir(src):
        splits = {n: g for n, pat in
                  dict(ball="*_Ball.c3d", p1="*_PPP1.c3d", p2="*_PPP2.c3d", tab="*_PPT.c3d").items()
                  for g in [sorted(glob.glob(os.path.join(src, pat)))] if g}
        if len(splits) == 4:
            out, rate, nf = {}, None, None
            for name, files in splits.items():
                g, rate_i, nf_i = read_groups(files[0], {name: want[name]})
                out[name] = g[name]
                if rate is None:
                    rate, nf = rate_i, nf_i
                else:
                    assert rate_i == rate and nf_i == nf, f"split file mismatch: {files[0]}"
            return out, rate, nf, "split"
        c3ds = [p for p in sorted(glob.glob(os.path.join(src, "*.c3d")))
                if not any(s in p for s in ("_Ball", "_PPP", "_PPT"))]
        assert len(c3ds) == 1, f"expected one combined c3d in {src}, got {c3ds}"
        src = c3ds[0]
    g, rate, nf = read_groups(src, want)
    return g, rate, nf, "combined"


def table_frame(tab):
    """Canonical table frame from the static markers (drops net posts by height).
    Returns center (m), R (cols = X=length, Y=width, Z=up), corners (m), extents (m)."""
    pts = np.nanmedian(tab, axis=0) * MM
    c0 = pts.mean(0)
    h = (pts - c0) @ np.linalg.svd(pts - c0)[2][2]
    keep = np.abs(h - np.median(h)) <= 0.080
    corners = pts[keep] if (len(pts) > 4 and keep.sum() >= 4) else pts
    center = corners.mean(0)
    Q = corners - center
    _, _, Vt = np.linalg.svd(Q)
    n = Vt[2]
    n = n if n[2] > 0 else -n
    Qp = Q - np.outer(Q @ n, n)
    evals, evecs = np.linalg.eigh(Qp.T @ Qp)
    length_dir = evecs[:, 2]
    ez = n
    ex = length_dir - (length_dir @ ez) * ez
    ex /= np.linalg.norm(ex)
    # deterministic sign: +X toward increasing world Y (the long axis in this rig)
    if abs(ex[1]) > abs(ex[0]):
        if ex[1] < 0:
            ex = -ex
    elif ex[0] < 0:
        ex = -ex
    ey = np.cross(ez, ex)
    R = np.column_stack([ex, ey, ez])
    return center, R, corners, (float(np.ptp(Qp @ ex)), float(np.ptp(Qp @ ey)))


def sphere_center(template):
    """Algebraic sphere fit; returns center offset (m), radius (m), max residual (m)."""
    A = np.column_stack([2 * template, np.ones(len(template))])
    b = (template ** 2).sum(1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cc = sol[:3]
    r = float(np.sqrt(max(sol[3] + cc @ cc, 0.0)))
    resid = float(np.abs(np.linalg.norm(template - cc, axis=1) - r).max())
    return cc, r, resid


def kabsch(A, B):
    H = A.T @ B
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    return Vt.T @ np.diag([1, 1, d]) @ U.T


def rigid_pose(markers_m, min_markers):
    """Per-frame centroid + quaternion for a rigid marker set (m). All-or-nothing
    frames expected, but subset-Kabsch (>=min_markers) is handled: template indices
    present in the frame are used."""
    nf, M, _ = markers_m.shape
    valid = np.isfinite(markers_m[:, :, 0])
    nval = valid.sum(1)
    present = nval >= min_markers
    cent = np.full((nf, 3), np.nan)
    quat = np.full((nf, 4), np.nan)
    rms = np.full(nf, np.nan)
    # template = first frame with ALL markers
    full_idx = np.where(nval == M)[0]
    if len(full_idx) == 0:
        return present & False, cent, quat, rms, None
    ref_frame = markers_m[full_idx[0]]
    ref_c = ref_frame.mean(0)
    ref = ref_frame - ref_c
    for k in np.where(present)[0]:
        sel = valid[k]
        P = markers_m[k, sel]
        c = P.mean(0)
        A = ref[sel] - ref[sel].mean(0)
        B = P - c
        R = kabsch(A, B)
        quat[k] = Rot.from_matrix(R).as_quat()
        rms[k] = np.sqrt(((B - A @ R.T) ** 2).sum(1).mean())
        # centroid of the FULL template mapped into world (robust to subsets)
        cent[k] = c + R @ (ref.mean(0) - ref[sel].mean(0))  # = c - R @ ref[sel].mean(0) since ref.mean=0
    return present, cent, quat, rms, ref


def main():
    src, out_dir = sys.argv[1], sys.argv[2]
    name = os.path.basename(src.rstrip("/")).replace(".c3d", "")
    os.makedirs(out_dir, exist_ok=True)

    g, rate, nf, mode = load_take(src)
    t = np.arange(nf) / rate
    ball_mm = g["ball"]

    center, Rt, corners, (l_ext, w_ext) = table_frame(g["tab"])
    tilt = float(np.degrees(np.arccos(np.clip(Rt[2, 2], -1, 1))))

    ball = ball_mm * MM
    present, cent_marker, quat, rms, ref = rigid_pose(ball, min_markers=4)
    cs_local, sph_r, sph_resid = sphere_center(ref)
    cent_w = cent_marker.copy()
    good = present & np.isfinite(quat).all(1)
    Rm = Rot.from_quat(quat[good]).as_matrix()
    cent_w[good] = cent_marker[good] + np.einsum("kij,j->ki", Rm, cs_local)
    cent_t = (cent_w - center) @ Rt

    paddles = {}
    for pn in ("p1", "p2"):
        if pn not in g:
            continue
        pm = g[pn] * MM
        pp, pc, pq, prms, pref = rigid_pose(pm, min_markers=4)  # need all 4 for stable normal
        if pref is None:
            continue
        # face normal in template space = plane normal of the 4 markers
        _, _, Vt = np.linalg.svd(pref)
        n0 = Vt[2]
        gq = pp & np.isfinite(pq).all(1)
        nrm = np.full((nf, 3), np.nan)
        Rp = Rot.from_quat(pq[gq]).as_matrix()
        nrm[gq] = np.einsum("kij,j->ki", Rp, n0)
        planarity = float(np.abs(pref @ n0).max())
        paddles[pn] = dict(present=pp, cent_w=pc, quat=pq, rms=prms, normal_w=nrm,
                           template=pref, planarity_m=planarity)

    # ---- trimming ----
    keep = np.ones(nf, bool)
    n_trim = int(round(TRIM_S * rate))
    keep[:n_trim] = False
    keep[nf - n_trim:] = False
    tracked_idx = np.where(present & keep)[0]
    if len(tracked_idx):
        keep[:tracked_idx[0]] = False
        keep[tracked_idx[-1] + 1:] = False
    a, b = np.where(keep)[0][[0, -1]]
    sl = slice(a, b + 1)

    # ---- below-table cut (ball only; keep time axis) ----
    below = np.zeros(nf, bool)
    zt = cent_t[:, 2]
    below[np.isfinite(zt) & (zt < BELOW_TABLE_M)] = True
    ball_valid = present & ~below

    def cut(x):
        y = x[sl].copy()
        return y

    ball_pos_w = cent_w.copy(); ball_pos_w[~ball_valid] = np.nan
    ball_pos_t = cent_t.copy(); ball_pos_t[~ball_valid] = np.nan
    ball_quat = quat.copy(); ball_quat[~ball_valid] = np.nan
    markers_out = ball.copy(); markers_out[~ball_valid] = np.nan

    out = dict(
        t=cut(t), frame=np.arange(a, b + 1).astype(np.int32), rate=rate,
        ball_present=cut(ball_valid), ball_below_table=cut(below),
        ball_pos_w_m=cut(ball_pos_w).astype(np.float32),
        ball_pos_t_m=cut(ball_pos_t).astype(np.float32),
        ball_quat_xyzw=cut(ball_quat).astype(np.float32),
        ball_kabsch_rms_m=cut(rms).astype(np.float32),
        ball_markers_w_m=cut(markers_out).astype(np.float32),
        ball_template_m=ref.astype(np.float32),
        ball_center_offset_local_m=cs_local.astype(np.float32),
        ball_sphere_radius_m=sph_r,
        table_center_w_m=center, table_R=Rt, table_corners_w_m=corners,
        table_length_ext_m=l_ext, table_width_ext_m=w_ext,
    )
    for pn, P in paddles.items():
        pos_t = (P["cent_w"] - center) @ Rt
        nrm_t = P["normal_w"] @ Rt
        out.update({
            f"{pn}_present": cut(P["present"]),
            f"{pn}_pos_w_m": cut(P["cent_w"]).astype(np.float32),
            f"{pn}_pos_t_m": cut(pos_t).astype(np.float32),
            f"{pn}_quat_xyzw": cut(P["quat"]).astype(np.float32),
            f"{pn}_normal_t": cut(nrm_t).astype(np.float32),
            f"{pn}_kabsch_rms_m": cut(P["rms"]).astype(np.float32),
            f"{pn}_template_m": P["template"].astype(np.float32),
        })
    np.savez_compressed(os.path.join(out_dir, f"{name}.npz"), **out)

    # ---- QA stats for the manifest ----
    pk = cut(ball_valid)
    nseg = 0
    gaps_ms = []
    s = None
    for i, v in enumerate(pk):
        if not v and s is None:
            s = i
        if v and s is not None:
            gaps_ms.append((i - s) / rate * 1e3); s = None
    if s is not None:
        gaps_ms.append((len(pk) - s) / rate * 1e3)
    # frozen-position check (identical consecutive ball positions = interpolation artifact)
    bp = cut(ball_pos_w)
    d = np.linalg.norm(np.diff(bp, axis=0), axis=1)
    frozen = int(np.nansum(d < 1e-6))

    man = dict(
        take=name, mode=mode, rate_hz=rate, n_frames_raw=int(nf),
        kept_frames=[int(a), int(b)], kept_dur_s=round(float(t[b] - t[a]), 2),
        trim_s=TRIM_S,
        ball_tracked_pct=round(float(np.mean(cut(present))) * 100, 2),
        ball_valid_pct=round(float(np.mean(pk)) * 100, 2),
        below_table_pct=round(float(np.mean(cut(below))) * 100, 2),
        marker_count_hist={int(v): int(c) for v, c in
                           zip(*np.unique(np.isfinite(ball_mm[sl, :, 0]).sum(1), return_counts=True))},
        kabsch_rms_median_mm=round(float(np.nanmedian(cut(rms))) * 1e3, 4),
        sphere_radius_mm=round(sph_r * 1e3, 3),
        sphere_resid_mm=round(sph_resid * 1e3, 4),
        center_offset_mm=round(float(np.linalg.norm(cs_local)) * 1e3, 3),
        table_length_ext_m=round(l_ext, 4), table_width_ext_m=round(w_ext, 4),
        table_len_err_mm=round((l_ext - ITTF_LEN) * 1e3, 1),
        table_wid_err_mm=round((w_ext - ITTF_WID) * 1e3, 1),
        table_tilt_vs_world_deg=round(tilt, 3),
        table_corner_z_w_mm=[round(float(z) * 1e3, 1) for z in corners[:, 2]],
        n_gaps=len(gaps_ms),
        gap_ms_median=round(float(np.median(gaps_ms)), 1) if gaps_ms else 0,
        gap_ms_p90=round(float(np.percentile(gaps_ms, 90)), 1) if gaps_ms else 0,
        gap_ms_max=round(float(np.max(gaps_ms)), 1) if gaps_ms else 0,
        frozen_consecutive_positions=frozen,
    )
    for pn, P in paddles.items():
        man[f"{pn}_tracked_pct"] = round(float(np.mean(cut(P["present"]))) * 100, 2)
        man[f"{pn}_planarity_mm"] = round(P["planarity_m"] * 1e3, 3)
        man[f"{pn}_kabsch_rms_median_mm"] = round(float(np.nanmedian(cut(P["rms"]))) * 1e3, 4)
    with open(os.path.join(out_dir, f"{name}_manifest.json"), "w") as f:
        json.dump(man, f, indent=2, ensure_ascii=False)
    print(json.dumps(man, ensure_ascii=False))


if __name__ == "__main__":
    main()
