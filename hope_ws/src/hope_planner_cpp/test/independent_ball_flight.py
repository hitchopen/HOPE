"""Independent simulation truth shared by offline and ROS qualification."""
import numpy as np
from scipy.integrate import solve_ivp


def flight(speed=5., spin='unknown', scenario='standard'):
    """Independent DOP853 plant, explicit thin-shell impulse and orientation.

    New fast-flight equivalents have frozen initial geometry below. They are
    not the unavailable original reviewer's exact recordings.
    """
    omega = 30. if spin == 'known' else 0.
    drag = 0. if scenario in ['net_clear', 'net_collision'] else .12861635416914574
    def rhs(w):
        def derivative(_, s):
            v = s[3:6]
            a = np.array([0., 0., -9.81]) - drag*np.linalg.norm(v)*v
            if spin != 'unknown':
                a += .00379260504480258*np.cross([0., w, 0.], v)
            return np.r_[v, a, w]
        return derivative
    def contact(_, s):
        return s[2] - .02
    contact.direction = -1
    contact.terminal = True
    geometry = {5.: (1.8, .4, -.5), 10.: (2.5, .32, -.5),
                12.: (2.6, .3, -.5), 20.: (2.6, .65, -5.)}
    x, z, vz = geometry[speed]
    velocity_x = -speed
    if scenario in ['net_clear', 'net_collision']:
        x, velocity_x = 2.6, -10.
        net_time = (x-1.37)/10
        vz = -4.+9.81*net_time
        z = .1525+.024-vz*net_time+4.905*net_time**2
        if scenario == 'net_collision': z -= .015
    if scenario == 'uncertain_contact': x = 1.2
    before = solve_ivp(rhs(omega), [0, 1], [x, -.7, z, velocity_x, 0., vz, 0.],
                       method='DOP853', events=contact, dense_output=True, rtol=1e-11, atol=1e-13)
    bounce = before.t[-1]
    post = before.y[:, -1].copy()
    slip = post[3] - .02*omega if spin != 'unknown' else post[3]
    impulse = min(.38744443855065164*abs(slip),
                  .16706204822993975*(1+.9313775047019153)*abs(post[5]))
    dv = -np.sign(slip)*impulse
    post[3] += dv
    if spin != 'unknown': omega -= dv/((2./3)*.02)
    post[5] *= -.9313775047019153
    after = solve_ivp(rhs(omega), [bounce, .7], post, method='DOP853', dense_output=True, rtol=1e-11, atol=1e-13)
    return before, after, bounce

