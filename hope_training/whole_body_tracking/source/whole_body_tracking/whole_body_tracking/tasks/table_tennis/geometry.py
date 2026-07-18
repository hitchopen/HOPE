"""Canonical table-tennis geometry and bounce materials, in the world frame.

World frame (ROS 2 REP-103, shared with the planner and the mocap reference):

* **Origin** — the near-side **left** corner of the table *surface*, from Player One's (P1) perspective.
* **X** — forward, toward Player Two (P2), along the table length:  ``x in [0, length] m``.
* **Y** — left, from P1's perspective, along the table width:        ``y in [-width, 0] m``.
* **Z** — up; **z = 0 is the table surface** (the floor is therefore at ``z = -height``).

Landmarks (net center, half centers, floor point) are derived from the ITTF regulation dimensions.
This module is the **single source of truth** for the geometry used to build the Isaac Lab scene; every
dimension, landmark and contact material is read from :mod:`configs/ball_physics.yaml` (see
:mod:`.physics_config`) so the simulator, the planner and the evaluator share one consistent world.

It is pure Python (no Isaac / torch imports) so it can be imported anywhere and unit-tested without a
simulator. Each environment's local origin sits at the table-surface height (z = 0), so an asset's
environment-local position *is* its world-frame position; with multiple environments, every environment
is an independent court anchored at its own origin.
"""

from __future__ import annotations

from dataclasses import dataclass

from .physics_config import load_ball_physics

# Load the shared no-spin physics config once (configs/ball_physics.yaml over documented defaults).
_CFG = load_ball_physics()

##
# ITTF regulation table (meters). Sourced from configs/ball_physics.yaml.
##
TABLE_LENGTH: float = float(_CFG["table"]["length"])       # along +X
TABLE_WIDTH: float = float(_CFG["table"]["width"])          # along -Y
TABLE_HEIGHT: float = float(_CFG["table"]["height"])        # table surface above the floor
TABLE_THICKNESS: float = float(_CFG["table"]["thickness"])  # visual/collision slab thickness of the top

NET_X: float = TABLE_LENGTH / 2.0                           # net plane along X (mid-table)
NET_HEIGHT: float = float(_CFG["net"]["height"])            # net height above the table surface
NET_OVERHANG: float = float(_CFG["net"]["overhang"])        # net extends this far past each Y edge
NET_THICKNESS: float = float(_CFG["net"]["thickness"])      # net slab thickness along X

LINE_WIDTH: float = float(_CFG["lines"]["width"])           # painted boundary / center line width
LINE_THICKNESS: float = float(_CFG["lines"]["thickness"])   # painted line slab thickness

##
# Ball (40 mm competition ball).
##
BALL_RADIUS: float = float(_CFG["ball"]["radius"])          # 20 mm radius (40 mm diameter)
BALL_MASS: float = float(_CFG["ball"]["mass"])              # kg

##
# Gravity magnitude (acts along -Z).
##
GRAVITY: float = float(_CFG["gravity"])

##
# Coordinate landmarks in the world frame.
##
ORIGIN: tuple[float, float, float] = (0.0, 0.0, 0.0)
TABLE_CENTER: tuple[float, float, float] = (TABLE_LENGTH / 2.0, -TABLE_WIDTH / 2.0, 0.0)
NET_CENTER: tuple[float, float, float] = (NET_X, -TABLE_WIDTH / 2.0, 0.0)
P1_HALF_CENTER: tuple[float, float, float] = (TABLE_LENGTH / 4.0, -TABLE_WIDTH / 2.0, 0.0)
P2_HALF_CENTER: tuple[float, float, float] = (3.0 * TABLE_LENGTH / 4.0, -TABLE_WIDTH / 2.0, 0.0)
FLOOR_Z: float = -TABLE_HEIGHT      # the floor surface, in world Z

##
# Robot nominal standing pose, P1 side (world frame). X < 0 puts the robot behind the near table end;
# Y centers it on the table width; the pelvis height is added on top of FLOOR_Z by the robot config.
##
P1_STAND_X: float = -0.5
P1_STAND_Y: float = -TABLE_WIDTH / 2.0   # centered on table width


def table_top_center() -> tuple[float, float, float]:
    """Center of the table-top collision/visual slab (its top face is at world z = 0)."""
    return (TABLE_CENTER[0], TABLE_CENTER[1], -TABLE_THICKNESS / 2.0)


def net_center() -> tuple[float, float, float]:
    """Center of the net slab (spans z in [0, NET_HEIGHT])."""
    return (NET_X, -TABLE_WIDTH / 2.0, NET_HEIGHT / 2.0)


def net_size() -> tuple[float, float, float]:
    """Full extents (x, y, z) of the net slab, including the lateral overhang past the table edges."""
    return (NET_THICKNESS, TABLE_WIDTH + 2.0 * NET_OVERHANG, NET_HEIGHT)


def table_top_size() -> tuple[float, float, float]:
    """Full extents (x, y, z) of the table-top slab."""
    return (TABLE_LENGTH, TABLE_WIDTH, TABLE_THICKNESS)


@dataclass
class ServeConfig:
    """Parameters for the scripted **no-spin** ball serve used to reset / visualize ball flight.

    The ball is spawned over the P2 half of the table and launched toward the P1-side robot with a
    roughly flat, slightly downward arc, so a reset shows a full flight -> table bounce -> return arc.
    Positions and velocities are in the world frame (m, m/s); ranges are sampled uniformly per reset.
    The ball carries no angular velocity (no-spin model).
    """

    # Spawn box over the P2 half. Spawn height is well above the net top so the served arc clears the
    # net at x = NET_X for essentially all resets.
    pos_x_range: tuple[float, float] = (2.0, 2.4)
    pos_y_range: tuple[float, float] = (-1.1, -0.4)
    pos_z_range: tuple[float, float] = (0.55, 0.80)
    # Launch velocity toward P1 (-X), roughly flat / slightly up so it clears the net then arcs down
    # under gravity onto the P1 half, with small lateral spread.
    vel_x_range: tuple[float, float] = (-5.0, -3.5)
    vel_y_range: tuple[float, float] = (-0.4, 0.4)
    vel_z_range: tuple[float, float] = (-0.2, 0.5)


@dataclass
class BounceMaterials:
    """PhysX contact-material parameters for the ball and the static surfaces.

    Values are read from ``configs/ball_physics.yaml`` (``contact.*``). Every material is created with
    ``*_combine_mode="multiply"`` (see ``table_tennis_env_cfg._surface_material``), so the *effective*
    coefficient of a ball<->surface contact is the **product** of the two materials' values. The ball
    material is kept **neutral** (restitution and friction = 1.0), so each surface's coefficient is the
    effective ball<->surface coefficient directly:

        effective ball<->table normal restitution == table_restitution

    Restitutions therefore carry the measured effective ball<->surface normal restitution per surface
    (table ~0.92; floor ~0.40; a deliberately low net ~0.10 so the ball dies on a net touch). Friction
    is the PhysX Coulomb coefficient used for the tangential (horizontal) bounce.
    """

    # Ball (dynamic) — neutral, so each surface value is the effective ball<->surface value.
    ball_restitution: float = float(_CFG["contact"]["ball"]["restitution"])
    ball_static_friction: float = float(_CFG["contact"]["ball"]["static_friction"])
    ball_dynamic_friction: float = float(_CFG["contact"]["ball"]["dynamic_friction"])
    # Table top.
    table_restitution: float = float(_CFG["contact"]["table"]["restitution"])
    table_static_friction: float = float(_CFG["contact"]["table"]["static_friction"])
    table_dynamic_friction: float = float(_CFG["contact"]["table"]["dynamic_friction"])
    # Floor.
    floor_restitution: float = float(_CFG["contact"]["floor"]["restitution"])
    floor_static_friction: float = float(_CFG["contact"]["floor"]["static_friction"])
    floor_dynamic_friction: float = float(_CFG["contact"]["floor"]["dynamic_friction"])
    # Net (low restitution so the ball dies on contact).
    net_restitution: float = float(_CFG["contact"]["net"]["restitution"])
    net_static_friction: float = float(_CFG["contact"]["net"]["static_friction"])
    net_dynamic_friction: float = float(_CFG["contact"]["net"]["dynamic_friction"])
    # Paddle (the racket link on the robot articulation). Applied by the A3 env cfg via a startup
    # material event on the racket body, so the effective ball<->paddle normal restitution equals
    # this measured value (the ball material is neutral and combines multiplicatively).
    paddle_restitution: float = float(_CFG["contact"]["paddle"]["restitution"])
    paddle_static_friction: float = float(_CFG["contact"]["paddle"]["static_friction"])
    paddle_dynamic_friction: float = float(_CFG["contact"]["paddle"]["dynamic_friction"])


@dataclass
class OutOfBoundsBox:
    """Axis-aligned region (world frame) outside which the ball is considered dead / out of play.

    Generous margins around the table so the ball can fly well past either player before the episode is
    reset. Used by the termination / serve-reset logic.
    """

    x: tuple[float, float] = (-2.0, TABLE_LENGTH + 2.0)
    y: tuple[float, float] = (-3.0, 1.5)
    # A ball resting on the floor has center z = FLOOR_Z + BALL_RADIUS; trigger just above that so a
    # grounded/missed ball ends the episode promptly instead of idling until timeout.
    z: tuple[float, float] = (FLOOR_Z + BALL_RADIUS + 0.01, 3.0)

    def as_dict(self) -> dict[str, tuple[float, float]]:
        return {"x": self.x, "y": self.y, "z": self.z}
