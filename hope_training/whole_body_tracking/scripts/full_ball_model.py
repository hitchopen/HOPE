"""Full PPQ004 paddle-to-first-bounce model shared by training and runtime tests.

All positions use the planner table frame (playing surface z=0). Flight includes
drag and Magnus with constant spin; paddle contact transfers both translation
and spin. The first descending ball-center crossing is z=ball radius. The inverse
solves that actual first-bounce position and time, and rejects nonconvergence.
"""
from __future__ import annotations

import numpy as np

try:
    from numba.extending import register_jitable
except ImportError:
    def register_jitable(function):
        return function


KD, KM, G, RADIUS, INERTIA, PAD_AT, PAD_BT, PAD_MU, PAD_G1, PAD_G2, PAD_E0 = range(11)
TAB_E, TAB_AT, TAB_BT, TAB_MU, DT, HORIZON, NET_X, NET_HEIGHT, WIDTH, LENGTH = range(11, 21)
EMIN, EMAX, SEED_ITER, NEWTON_ITER, FD_STEP, POS_TOL, TIME_TOL = range(21, 28)
MAX_STEP, LINESEARCH, NET_MARGIN, EPS, FLIGHT_TIME, OVERHANG = range(28, 34)
ALT_START = 34


def params_from_profile(raw: dict) -> np.ndarray:
    """Pack the expanded YAML profile into a Numba-compatible numeric vector."""
    model = raw['return_model']
    if model['id'] != 'full_ppq004_first_bounce_v1':
        raise ValueError('Full ball model requires full_ppq004_first_bounce_v1')
    flight, ball = raw['flight'], raw['ball']
    paddle, table = raw['contact']['paddle'], raw['contact']['table']
    params = np.asarray([
        flight['k_d'], flight['k_m'], flight['g'], ball['radius'], ball['inertia_coeff'],
        paddle['a_t'], paddle['b_t'], paddle['mu_safety'], paddle['e_exp_g1'], paddle['e_exp_g2'],
        model['restitution_racket'], table['e_eff'], table['a_t'], table['b_t'], table['mu_safety'],
        model['integrate_dt_s'], model['max_predict_time_s'], raw['net']['x_position'],
        raw['net']['height'], raw['table']['width'], raw['table']['length'],
        model['restitution_min'], model['restitution_max'], model['seed_iterations'],
        model['newton_iterations'], model['fd_relative_step'], model['position_tolerance_m'],
        model['time_tolerance_s'], model['max_step_mps'], model['line_search_iterations'],
        model['net_clearance_margin'], model['numeric_epsilon'], model['flight_time_s'],
        raw['net']['overhang'], *model['alternative_flight_times'],
    ], dtype=np.float64)
    if not np.isfinite(params).all():
        raise ValueError('Full ball parameters must be finite')
    positive = (G, RADIUS, INERTIA, DT, HORIZON, WIDTH, LENGTH, FD_STEP,
                POS_TOL, TIME_TOL, MAX_STEP, EPS, FLIGHT_TIME)
    if any(params[index] <= 0 for index in positive) or np.any(params[ALT_START:] <= 0):
        raise ValueError('Full ball geometry, timing and tolerances must be positive')
    if (params[KD] < 0 or params[PAD_MU] < 0 or params[TAB_MU] < 0
            or params[EMIN] < 0 or params[EMAX] < params[EMIN] or params[PAD_E0] <= -1
            or params[NET_MARGIN] < 0):
        raise ValueError('Invalid full ball drag, contact or clearance parameters')
    for index in (SEED_ITER, NEWTON_ITER, LINESEARCH):
        if params[index] < 0 or params[index] != int(params[index]):
            raise ValueError('Iteration counts must be nonnegative integers')
    if params[NEWTON_ITER] == 0 or params[LINESEARCH] == 0:
        raise ValueError('Newton and line-search iteration counts must be positive')
    return params


@register_jitable
def _clamp(value, lower, upper):
    return min(upper, max(lower, value))


@register_jitable
def _contact(incoming, racket, normal, spin, params, restitution, a_t, b_t, mu):
    n = normal / (np.linalg.norm(normal) + params[EPS])
    if np.dot(incoming - racket, n) > 0:
        n = -n
    relative = incoming + np.cross(spin, -params[RADIUS] * n) - racket
    normal_speed = np.dot(relative, n)
    tangent = relative - normal_speed * n
    tangent_speed = np.linalg.norm(tangent)
    approach = abs(normal_speed)
    raw = (a_t + b_t * approach / (np.hypot(tangent_speed, normal_speed) + params[EPS])) * tangent_speed
    impulse = min(max(raw, 0.0), mu * (1 + restitution) * approach)
    delta_tangent = np.zeros(3)
    if tangent_speed > params[EPS]:
        delta_tangent = -impulse * tangent / (tangent_speed + params[EPS])
    outgoing = incoming - (1 + restitution) * normal_speed * n + delta_tangent
    outgoing_spin = spin - np.cross(n, delta_tangent) / (params[INERTIA] * params[RADIUS])
    return outgoing, outgoing_spin


@register_jitable
def full_contact(incoming, racket, normal, spin, params):
    """Full paddle impulse; raw face normal is normalized once and oriented inward."""
    n = normal / (np.linalg.norm(normal) + params[EPS])
    if np.dot(incoming - racket, n) > 0:
        n = -n
    relative = incoming + np.cross(spin, -params[RADIUS] * n) - racket
    approach = abs(np.dot(relative, n))
    restitution = _clamp(params[PAD_G1] * np.exp(params[PAD_G2] * approach),
                         params[EMIN], params[EMAX])
    return _contact(incoming, racket, normal, spin, params, restitution,
                    params[PAD_AT], params[PAD_BT], params[PAD_MU])


@register_jitable
def table_contact(incoming, spin, params):
    """Optional first table impulse; no post-bounce rollout is required by this model."""
    return _contact(incoming, np.zeros(3), np.array([0., 0., 1.]), spin, params,
                    params[TAB_E], params[TAB_AT], params[TAB_BT], params[TAB_MU])


@register_jitable
def _accel_components(vx, vy, vz, wx, wy, wz, drag, magnus, gravity):
    """Scalar flight kernel: no temporary vectors in the integration loop."""
    factor = -drag * np.sqrt(vx*vx + vy*vy + vz*vz)
    return (factor*vx + magnus*(wy*vz - wz*vy),
            factor*vy + magnus*(wz*vx - wx*vz),
            factor*vz + magnus*(wx*vy - wy*vx) - gravity)


@register_jitable
def flight_accel(velocity, spin, params):
    return np.array(_accel_components(velocity[0], velocity[1], velocity[2],
                    spin[0], spin[1], spin[2], params[KD], params[KM], params[G]))


@register_jitable
def _rk4_components(px, py, pz, vx, vy, vz, wx, wy, wz, h, drag, magnus, gravity):
    # Preserve the RK4 stages and operation grouping of the vector expression.
    # Tuple/scalar state lets Numba keep all stages in registers instead of
    # allocating acceleration and intermediate-position arrays each step.
    ax, ay, az = _accel_components(vx, vy, vz, wx, wy, wz, drag, magnus, gravity)
    bx, by, bz = _accel_components(vx + .5*h*ax, vy + .5*h*ay, vz + .5*h*az,
                                   wx, wy, wz, drag, magnus, gravity)
    cx, cy, cz = _accel_components(vx + .5*h*bx, vy + .5*h*by, vz + .5*h*bz,
                                   wx, wy, wz, drag, magnus, gravity)
    dx, dy, dz = _accel_components(vx + h*cx, vy + h*cy, vz + h*cz,
                                   wx, wy, wz, drag, magnus, gravity)
    return (px + h*(vx + 2*(vx + .5*h*ax) + 2*(vx + .5*h*bx) + vx + h*cx)/6,
            py + h*(vy + 2*(vy + .5*h*ay) + 2*(vy + .5*h*by) + vy + h*cy)/6,
            pz + h*(vz + 2*(vz + .5*h*az) + 2*(vz + .5*h*bz) + vz + h*cz)/6,
            vx + h*(ax + 2*bx + 2*cx + dx)/6,
            vy + h*(ay + 2*by + 2*cy + dy)/6,
            vz + h*(az + 2*bz + 2*cz + dz)/6)


@register_jitable
def _finite_state(px, py, pz, vx, vy, vz):
    return (np.isfinite(px) and np.isfinite(py) and np.isfinite(pz)
            and np.isfinite(vx) and np.isfinite(vy) and np.isfinite(vz))


@register_jitable
def rk4_step(position, velocity, spin, h, params):
    px, py, pz, vx, vy, vz = _rk4_components(
        position[0], position[1], position[2], velocity[0], velocity[1], velocity[2],
        spin[0], spin[1], spin[2], h, params[KD], params[KM], params[G])
    return np.array([px, py, pz]), np.array([vx, vy, vz])


@register_jitable
def integrate_flight(position, velocity, spin, duration, params):
    px, py, pz = position
    vx, vy, vz = velocity
    wx, wy, wz = spin
    if (not _finite_state(px, py, pz, vx, vy, vz) or not np.isfinite(spin).all()
            or not np.isfinite(duration) or duration < 0):
        return position.copy(), velocity.copy(), False
    drag, magnus, gravity, step = params[KD], params[KM], params[G], params[DT]
    remaining = duration
    while remaining > 1e-12:
        h = min(step, remaining)
        px, py, pz, vx, vy, vz = _rk4_components(px, py, pz, vx, vy, vz,
                                                wx, wy, wz, h, drag, magnus, gravity)
        remaining -= h
        if not _finite_state(px, py, pz, vx, vy, vz):
            return np.array([px, py, pz]), np.array([vx, vy, vz]), False
    return np.array([px, py, pz]), np.array([vx, vy, vz]), True


@register_jitable
def first_bounce(position, velocity, spin, params):
    """First descending center crossing: xyz, velocity, time, net xyz, net-valid, landed.

    A crossing outside the table is still a mathematical surface crossing;
    callers can separately require table bounds. Net crossings after that first
    event do not count, and no bounce or net collision is integrated.
    """
    px, py, pz = position
    vx, vy, vz = velocity
    wx, wy, wz = spin
    net = np.zeros(3)
    net_valid = False
    radius, net_x = params[RADIUS], params[NET_X]
    if (not _finite_state(px, py, pz, vx, vy, vz) or not np.isfinite(spin).all()
            or pz <= radius):
        return np.zeros(3), np.zeros(3), 0., net, net_valid, False
    drag, magnus, gravity = params[KD], params[KM], params[G]
    step, horizon = params[DT], params[HORIZON]
    elapsed = 0.0
    while horizon - elapsed > 1e-12:
        h = min(step, horizon - elapsed)
        nx, ny, nz, ux, uy, uz = _rk4_components(px, py, pz, vx, vy, vz,
                                               wx, wy, wz, h, drag, magnus, gravity)
        if not _finite_state(nx, ny, nz, ux, uy, uz):
            return np.zeros(3), np.zeros(3), 0., net, net_valid, False
        landed = pz > radius and nz <= radius
        fraction = (pz - radius) / (pz - nz) if landed else 1.0
        if not net_valid and px < net_x <= nx:
            net_fraction = (net_x - px) / (nx - px)
            if net_fraction <= fraction:
                net[0] = px + net_fraction*(nx - px)
                net[1] = py + net_fraction*(ny - py)
                net[2] = pz + net_fraction*(nz - pz)
                net_valid = True
        if landed:
            return (np.array([px + fraction*(nx - px), py + fraction*(ny - py), pz + fraction*(nz - pz)]),
                    np.array([vx + fraction*(ux - vx), vy + fraction*(uy - vy), vz + fraction*(uz - vz)]),
                    elapsed + fraction * h, net, net_valid, True)
        px, py, pz, vx, vy, vz = nx, ny, nz, ux, uy, uz
        elapsed += h
    return np.zeros(3), np.zeros(3), 0., net, net_valid, False


@register_jitable
def evaluate_contact(position, incoming, spin, racket, params):
    if (not np.isfinite(position).all() or not np.isfinite(incoming).all()
            or not np.isfinite(spin).all() or not np.isfinite(racket).all()
            or np.linalg.norm(racket) <= params[EPS] or position[2] <= params[RADIUS]):
        return np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), 0., np.zeros(3), False, False
    normal = racket / (np.linalg.norm(racket) + params[EPS])
    outgoing, outgoing_spin = full_contact(incoming, racket, racket, spin, params)
    bounce, _, time, net, net_valid, landed = first_bounce(position, outgoing, outgoing_spin, params)
    return normal, outgoing, outgoing_spin, bounce, time, net, net_valid, landed


@register_jitable
def _solve3(matrix, rhs):
    """Full-pivot 3x3 solve, with Eigen's default invertibility threshold."""
    a, b = matrix.copy(), rhs.copy()
    columns = np.arange(3)
    largest = 0.0
    for k in range(3):
        pivot_row, pivot_column, magnitude = k, k, 0.0
        for column in range(k, 3):
            for row in range(k, 3):
                if abs(a[row, column]) > magnitude:
                    pivot_row, pivot_column, magnitude = row, column, abs(a[row, column])
        if magnitude == 0 or not np.isfinite(magnitude):
            return np.zeros(3), False
        largest = max(largest, magnitude)
        for column in range(3):
            temporary = a[k, column]
            a[k, column] = a[pivot_row, column]
            a[pivot_row, column] = temporary
        temporary = b[k]
        b[k] = b[pivot_row]
        b[pivot_row] = temporary
        for row in range(3):
            temporary = a[row, k]
            a[row, k] = a[row, pivot_column]
            a[row, pivot_column] = temporary
        temporary_column = columns[k]
        columns[k] = columns[pivot_column]
        columns[pivot_column] = temporary_column
        for row in range(k + 1, 3):
            factor = a[row, k] / a[k, k]
            a[row, k] = 0.0
            for column in range(k + 1, 3):
                a[row, column] -= factor * a[k, column]
            b[row] -= factor * b[k]
    for axis in range(3):
        if abs(a[axis, axis]) <= 3 * np.finfo(np.float64).eps * largest:
            return np.zeros(3), False
    permuted = np.zeros(3)
    for row in range(2, -1, -1):
        value = b[row]
        for column in range(row + 1, 3):
            value -= a[row, column] * permuted[column]
        permuted[row] = value / a[row, row]
    solution = np.zeros(3)
    for axis in range(3):
        solution[columns[axis]] = permuted[axis]
    return solution, np.isfinite(solution).all()


@register_jitable
def _seed(position, incoming, target, time, params):
    outgoing = np.array([(target[0] - position[0]) / time, (target[1] - position[1]) / time,
                         (params[RADIUS] - position[2]) / time + .5 * params[G] * time])
    delta = outgoing - incoming
    length = np.linalg.norm(delta)
    if not np.isfinite(delta).all() or length < 1e-6:
        return np.zeros(3)
    normal = delta / length
    if normal[0] < 0:
        normal = -normal
    if normal[0] <= 1e-6:
        normal[0] += 1.0
        normal /= np.linalg.norm(normal)
    vin, vout = np.dot(incoming, normal), np.dot(outgoing, normal)
    restitution = params[PAD_E0]
    speed = (vout + restitution * vin) / (1 + restitution)
    for _ in range(int(params[SEED_ITER])):
        restitution = _clamp(params[PAD_G1] * np.exp(params[PAD_G2] * abs(vin - speed)),
                             params[EMIN], params[EMAX])
        speed = (vout + restitution * vin) / (1 + restitution)
    return speed * normal


@register_jitable
def _residual(bounce, time, target, desired_time):
    return np.array([bounce[0] - target[0], bounce[1] - target[1], time - desired_time])


@register_jitable
def _converged(residual, params):
    return np.linalg.norm(residual[:2]) <= params[POS_TOL] and abs(residual[2]) <= params[TIME_TOL]


@register_jitable
def _score(residual, params):
    return np.sqrt((residual[0] / params[POS_TOL])**2 + (residual[1] / params[POS_TOL])**2
                   + (residual[2] / params[TIME_TOL])**2)


@register_jitable
def solve_racket(position, incoming, spin, target, desired_time, params):
    """Full-contact Newton solve; return vr,n,out,outspin,bounce,time,net,net-valid,converged."""
    zero = np.zeros(3)
    if (not np.isfinite(position).all() or not np.isfinite(incoming).all()
            or not np.isfinite(spin).all() or not np.isfinite(target).all()
            or not np.isfinite(desired_time) or desired_time <= 0
            or position[2] <= params[RADIUS]):
        return zero, zero, zero, zero, zero, 0., zero, False, False
    racket = _seed(position, incoming, target, desired_time, params)
    for _ in range(int(params[NEWTON_ITER])):
        normal, outgoing, outgoing_spin, bounce, time, net, crossed, landed = evaluate_contact(
            position, incoming, spin, racket, params)
        if not landed:
            return racket, normal, outgoing, outgoing_spin, bounce, time, net, crossed, False
        residual = _residual(bounce, time, target, desired_time)
        if _converged(residual, params):
            return racket, normal, outgoing, outgoing_spin, bounce, time, net, crossed, True
        jacobian = np.empty((3, 3))
        for axis in range(3):
            epsilon = params[FD_STEP] * max(abs(racket[axis]), 1.0)
            perturbed = racket.copy()
            perturbed[axis] += epsilon
            _, _, _, sample, sample_time, _, _, sample_valid = evaluate_contact(
                position, incoming, spin, perturbed, params)
            if not sample_valid:
                return racket, normal, outgoing, outgoing_spin, bounce, time, net, crossed, False
            jacobian[:, axis] = (_residual(sample, sample_time, target, desired_time) - residual) / epsilon
        step, invertible = _solve3(jacobian, residual)
        if not invertible:
            return racket, normal, outgoing, outgoing_spin, bounce, time, net, crossed, False
        magnitude = np.linalg.norm(step)
        if magnitude > params[MAX_STEP]:
            step *= params[MAX_STEP] / magnitude
        improved = False
        scale = 1.0
        score = _score(residual, params)
        for _ in range(int(params[LINESEARCH])):
            candidate = racket - scale * step
            _, _, _, sample, sample_time, _, _, sample_valid = evaluate_contact(
                position, incoming, spin, candidate, params)
            if sample_valid and _score(_residual(sample, sample_time, target, desired_time), params) < score:
                racket = candidate
                improved = True
                break
            scale *= .5
        if not improved:
            return racket, normal, outgoing, outgoing_spin, bounce, time, net, crossed, False
    normal, outgoing, outgoing_spin, bounce, time, net, crossed, landed = evaluate_contact(
        position, incoming, spin, racket, params)
    converged = landed and _converged(_residual(bounce, time, target, desired_time), params)
    return racket, normal, outgoing, outgoing_spin, bounce, time, net, crossed, converged


@register_jitable
def net_clearance(net, crossed, params):
    if not crossed:
        return False, False
    bypass = net[1] < -params[WIDTH] - params[OVERHANG] or net[1] > params[OVERHANG]
    clear = not bypass and net[2] > params[NET_HEIGHT] + params[RADIUS] + params[NET_MARGIN]
    return clear, bypass


@register_jitable
def plan(position, incoming, spin, target, desired_time, params):
    """Return vr,n,out,outspin,bounce,time,converged,clear,bypass with configured retries."""
    failure = (np.zeros(3), np.array([1., 0., 0.]), np.zeros(3), np.zeros(3),
               np.zeros(3), 0., False, False, False)
    if (not np.isfinite(position).all() or not np.isfinite(incoming).all()
            or not np.isfinite(spin).all() or not np.isfinite(target).all()
            or not np.isfinite(desired_time) or desired_time <= 0):
        return failure
    racket, normal, outgoing, outgoing_spin, bounce, time, net, crossed, valid = solve_racket(
        position, incoming, spin, target, desired_time, params)
    clear, bypass = net_clearance(net, crossed, params)
    result = (racket, normal, outgoing, outgoing_spin, bounce, time, valid, clear, bypass) if valid else failure
    if valid and clear:
        return result
    for index in range(ALT_START, len(params)):
        racket, normal, outgoing, outgoing_spin, bounce, time, net, crossed, converged = solve_racket(
            position, incoming, spin, target, params[index], params)
        clear, bypass = net_clearance(net, crossed, params)
        candidate = (racket, normal, outgoing, outgoing_spin, bounce, time, converged, clear, bypass)
        if converged and clear:
            return candidate
        if converged and not result[6]:
            result = candidate
    return result
