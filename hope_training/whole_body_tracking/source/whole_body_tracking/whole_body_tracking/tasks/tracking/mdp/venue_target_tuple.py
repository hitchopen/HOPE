"""Correlated venue tuples qualified by the shared full PPQ004 first-bounce solve.

The online path transfers sampled tuples to the CPU only when targets resample;
Numba accelerates the shared full-contact solver when available. Normal training
uses its prequalified bank instead. Actor observations retain the 110-D ABI.
"""

from __future__ import annotations

import hashlib
import json
import math
import importlib.util
from pathlib import Path
import sys

import numpy as np

import torch


def _support_module(name):
    key = f'hope_venue_{name}'
    if key not in sys.modules:
        for parent in Path(__file__).resolve().parents:
            path = parent / 'scripts' / f'{name}.py'
            if path.is_file():
                spec = importlib.util.spec_from_file_location(key, path)
                module = importlib.util.module_from_spec(spec)
                sys.modules[key] = module
                spec.loader.exec_module(module)
                break
        else:
            raise FileNotFoundError(f'Cannot find training helper {name}')
    return sys.modules[key]


_profile = _support_module('ball_physics_profile')
_full = _support_module('full_ball_model')


def _solve_batch(position, incoming, spin, target, parameters):
    result = np.zeros((len(position), 18))
    for index in range(len(position)):
        vr, normal, out, omega, bounce, time, valid, clear, bypass = _full.plan(
            position[index], incoming[index], spin[index], target[index],
            parameters[_full.FLIGHT_TIME], parameters)
        result[index, :3] = vr
        result[index, 3:6] = normal
        result[index, 6:9] = out
        result[index, 9:12] = omega
        result[index, 12:15] = bounce
        result[index, 15] = time
        result[index, 16] = valid
        result[index, 17] = clear and not bypass
    return result


try:
    from numba import njit
    _solve_batch = njit(_solve_batch)
except ImportError:
    pass  # Correct, slower NumPy fallback for installations without the offline accelerator.


SCHEMA_VERSION = 2
CORRELATION_LABELS = ("vx", "vy", "vz", "contact_z", "spin_magnitude")
CORRELATION_MATRIX = (
    (1.0, 0.0, -0.44, 0.0, -0.36),
    (0.0, 1.0, 0.0, 0.0, 0.0),
    (-0.44, 0.0, 1.0, 0.30, 0.0),
    (0.0, 0.0, 0.30, 1.0, -0.26),
    (-0.36, 0.0, 0.0, -0.26, 1.0),
)
VENUE_VELOCITY_BOX = (
    (-2.85, -0.82),
    (-0.46, 0.31),
    (-2.02, 0.49),
)
VENUE_CONTACT_Z_RANGE = (0.98, 1.26)
VENUE_SPIN_MAX_RAD_S = 34.0
VENUE_CONTACT_Y_FH = (-0.35, -0.155)
VENUE_CONTACT_Y_BH = (-0.155, 0.18)
DEFAULT_LANDING_X_RANGE = (2.17, 3.04)
DEFAULT_LANDING_Y_RANGE = (-0.50, 0.50)


def sampler_receipt_payload() -> dict:
    """Return the canonical constants whose digest identifies this target generator."""

    return {
        "schema_version": SCHEMA_VERSION,
        "correlation_labels": CORRELATION_LABELS,
        "correlation_matrix": CORRELATION_MATRIX,
        "velocity_box": VENUE_VELOCITY_BOX,
        "contact_z_range": VENUE_CONTACT_Z_RANGE,
        "spin_max_rad_s": VENUE_SPIN_MAX_RAD_S,
        "contact_y_fh": VENUE_CONTACT_Y_FH,
        "contact_y_bh": VENUE_CONTACT_Y_BH,
        "landing_x_range": DEFAULT_LANDING_X_RANGE,
        "landing_y_range": DEFAULT_LANDING_Y_RANGE,
        "inverse": {"kind": "full_ppq004_first_bounce_v1",
                    "profile_source_sha256": _profile.profile_source_hashes()},
        # A paddle plane is invariant under n -> -n, but the task's normal
        # reward is signed. The caller must choose the hemisphere containing
        # the current clip's demonstrated racket-face normal.
        "normal_sign_contract": "per_clip_reference_hemisphere_v1",
    }


def sampler_receipt_sha256() -> str:
    canonical = json.dumps(
        sampler_receipt_payload(), sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def align_normal_to_reference_hemisphere(
    normal: torch.Tensor, reference_normal: torch.Tensor
) -> torch.Tensor:
    """Choose the sign-equivalent paddle normal nearest each clip reference."""

    if normal.shape != reference_normal.shape or normal.shape[-1] != 3:
        raise ValueError(
            "normal and reference_normal must have matching [...,3] shapes"
        )
    sign = torch.where(
        torch.sum(normal * reference_normal, dim=-1, keepdim=True) < 0.0,
        -torch.ones_like(normal[..., :1]),
        torch.ones_like(normal[..., :1]),
    )
    return normal * sign


def _stratified_uniform(
    count: int,
    dimensions: int,
    *,
    device: torch.device | str,
    dtype: torch.dtype,
) -> torch.Tensor:
    """Randomly shifted lattice with exactly one point in every marginal stratum."""

    if count < 1 or dimensions < 1:
        raise ValueError("stratified sampling requires positive count and dimensions")
    base = (
        torch.arange(count, device=device, dtype=dtype).unsqueeze(-1) + 0.5
    ) / float(count)
    shifts = torch.rand(1, dimensions, device=device, dtype=dtype)
    # Dimension-specific irrational strides decorrelate the marginal permutations without a
    # device-to-host sort or a separate generator.
    stride = torch.tensor(
        [math.sqrt(index + 2.0) for index in range(dimensions)],
        device=device,
        dtype=dtype,
    ).unsqueeze(0)
    return torch.remainder(base * stride + shifts, 1.0).clamp(
        1.0e-6, 1.0 - 1.0e-6
    )


def _gaussian_copula_uniforms(
    count: int,
    *,
    device: torch.device | str,
    dtype: torch.dtype,
) -> torch.Tensor:
    correlation = torch.tensor(CORRELATION_MATRIX, device=device, dtype=dtype)
    eigenvalues = torch.linalg.eigvalsh(correlation)
    if not bool((eigenvalues > 0.0).all()):
        raise RuntimeError("venue correlation matrix is not positive definite")
    independent_u = _stratified_uniform(
        count, len(CORRELATION_LABELS), device=device, dtype=dtype
    )
    independent_z = math.sqrt(2.0) * torch.erfinv(2.0 * independent_u - 1.0)
    correlated_z = independent_z @ torch.linalg.cholesky(correlation).T
    return (0.5 * (1.0 + torch.erf(correlated_z / math.sqrt(2.0)))).clamp(
        0.0, 1.0
    )


def full_model_racket_target(
    contact_pos_w: torch.Tensor,
    incoming_velocity_w: torch.Tensor,
    incoming_spin_w: torch.Tensor,
    intended_landing_xy_w: torch.Tensor,
    *,
    table_surface_z: float | None = None,
    table_near_x: float | None = None,
    profile: dict | None = None,
) -> dict[str, torch.Tensor]:
    """Solve env-local contact/landing inputs, including the sampled incoming spin."""

    if contact_pos_w.ndim != 2 or contact_pos_w.shape[-1] != 3:
        raise ValueError("contact_pos_w must have shape [N,3]")
    if incoming_velocity_w.shape != contact_pos_w.shape:
        raise ValueError("incoming velocity shape must match contact position")
    if incoming_spin_w.shape != contact_pos_w.shape:
        raise ValueError("incoming spin shape must match contact position")
    if intended_landing_xy_w.shape != (contact_pos_w.shape[0], 2):
        raise ValueError("intended_landing_xy_w must have shape [N,2]")
    raw = _profile.load_profile() if profile is None else profile
    if raw.get('return_model', {}).get('id') != 'full_ppq004_first_bounce_v1':
        raise ValueError('Online target solving requires the full PPQ004 profile; use the '
                         'published bank mode when selecting an explicit legacy venue recipe')
    parameters = _full.params_from_profile(raw)
    frame = raw['training_frame']
    offset = np.array([frame['near_x'] if table_near_x is None else table_near_x,
                       frame['y_center'] + raw['table']['width'] / 2,
                       frame['surface_z'] if table_surface_z is None else table_surface_z])
    def cpu(tensor):
        return tensor.detach().to(device='cpu', dtype=torch.float64).numpy()
    solved = _solve_batch(cpu(contact_pos_w) - offset, cpu(incoming_velocity_w),
                          cpu(incoming_spin_w), cpu(intended_landing_xy_w) - offset[:2], parameters)
    result = torch.as_tensor(solved, device=contact_pos_w.device, dtype=contact_pos_w.dtype)
    landing = result[:, 12:14] + torch.as_tensor(offset[:2], device=result.device, dtype=result.dtype)
    return {'racket_velocity_w': result[:, :3], 'racket_normal_w': result[:, 3:6],
            'outgoing_velocity_seed_w': result[:, 6:9], 'outgoing_spin_w': result[:, 9:12],
            'predicted_landing_xy_w': landing, 'return_flight_time_s': result[:, 15],
            'model_valid': result[:, 16].bool(), 'model_net_clear': result[:, 17].bool()}


def sample_correlated_venue_tuple(
    clip: torch.Tensor,
    contact_x_w: torch.Tensor,
    *,
    table_surface_z: float | None = None,
    table_near_x: float | None = None,
    landing_x_range: tuple[float, float] = DEFAULT_LANDING_X_RANGE,
    landing_y_range: tuple[float, float] = DEFAULT_LANDING_Y_RANGE,
) -> dict[str, torch.Tensor]:
    """Sample one correlated physical tuple for every requested FH/BH clip."""

    if clip.ndim != 1 or clip.dtype != torch.long:
        raise ValueError("clip must be a 1-D int64 tensor")
    if contact_x_w.shape != clip.shape:
        raise ValueError("contact_x_w must have one value per clip")
    if bool(((clip < 0) | (clip > 1)).any()):
        raise ValueError("venue tuple sampler supports clip ids 0=FH and 1=BH")
    count = int(clip.numel())
    if count < 1:
        raise ValueError("venue tuple sampler requires at least one sample")
    dtype = contact_x_w.dtype
    device = contact_x_w.device
    correlated_u = _gaussian_copula_uniforms(
        count, device=device, dtype=dtype
    )

    velocity_bounds = torch.tensor(
        VENUE_VELOCITY_BOX, device=device, dtype=dtype
    )
    incoming_velocity = velocity_bounds[:, 0] + correlated_u[:, :3] * (
        velocity_bounds[:, 1] - velocity_bounds[:, 0]
    )
    contact_z = (
        VENUE_CONTACT_Z_RANGE[0]
        + correlated_u[:, 3]
        * (VENUE_CONTACT_Z_RANGE[1] - VENUE_CONTACT_Z_RANGE[0])
    )
    spin_magnitude = correlated_u[:, 4] * VENUE_SPIN_MAX_RAD_S

    other_u = _stratified_uniform(
        count, 5, device=device, dtype=dtype
    )
    y_lo = torch.where(
        clip == 0,
        torch.full_like(contact_x_w, VENUE_CONTACT_Y_FH[0]),
        torch.full_like(contact_x_w, VENUE_CONTACT_Y_BH[0]),
    )
    y_hi = torch.where(
        clip == 0,
        torch.full_like(contact_x_w, VENUE_CONTACT_Y_FH[1]),
        torch.full_like(contact_x_w, VENUE_CONTACT_Y_BH[1]),
    )
    contact_y = y_lo + other_u[:, 0] * (y_hi - y_lo)
    contact_pos = torch.stack((contact_x_w, contact_y, contact_z), dim=-1)

    landing_x = float(landing_x_range[0]) + other_u[:, 1] * (
        float(landing_x_range[1]) - float(landing_x_range[0])
    )
    landing_y = float(landing_y_range[0]) + other_u[:, 2] * (
        float(landing_y_range[1]) - float(landing_y_range[0])
    )
    intended_landing = torch.stack((landing_x, landing_y), dim=-1)

    spin_direction = torch.stack(
        (
            torch.cos(2.0 * math.pi * other_u[:, 3])
            * torch.sqrt(1.0 - (2.0 * other_u[:, 4] - 1.0).square()),
            torch.sin(2.0 * math.pi * other_u[:, 3])
            * torch.sqrt(1.0 - (2.0 * other_u[:, 4] - 1.0).square()),
            2.0 * other_u[:, 4] - 1.0,
        ),
        dim=-1,
    )
    incoming_spin = spin_direction * spin_magnitude.unsqueeze(-1)
    solved = full_model_racket_target(
        contact_pos,
        incoming_velocity,
        incoming_spin,
        intended_landing,
        table_surface_z=table_surface_z,
        table_near_x=table_near_x,
    )
    return {
        "contact_pos_w": contact_pos,
        "incoming_velocity_w": incoming_velocity,
        "incoming_spin_w": incoming_spin,
        "intended_landing_xy_w": intended_landing,
        **solved,
    }
