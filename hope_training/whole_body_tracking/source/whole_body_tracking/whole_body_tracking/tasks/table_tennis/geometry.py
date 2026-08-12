"""Canonical HOPE table-tennis geometry, expressed in the HOPE world frame.

The HOPE world frame (ROS 2 REP-103, identical to the planner / mocap reference docs):

* **Origin** — the near-side **left** corner of the table *surface*, from Player One's (P1) perspective.
* **X** — forward, toward Player Two (P2), along the table length:  ``x in [0, 2.74] m``.
* **Y** — left, from P1's perspective, along the table width:        ``y in [-1.525, 0] m``.
* **Z** — up; **z = 0 is the table surface** (the floor is therefore at ``z = -0.76 m``).

Key landmarks (see ``mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md`` §2.1
and ``hope_ws/.../hope_world_frame.yaml``)::

    Origin (P1 near-side left corner)   ( 0.0  ,  0.0   ,  0.0 )
    Net center line                     ( 1.37 , -0.7625,  0.0 )
    P1 half center                      ( 0.685, -0.7625,  0.0 )
    P2 half center                      ( 2.055, -0.7625,  0.0 )
    Floor directly below origin         ( 0.0  ,  0.0   , -0.76)

This module is the **single source of truth** for the geometry used to build the Isaac Lab scene.
It is pure Python (no Isaac / torch imports) so it can be imported anywhere and unit-tested without a
simulator. The numbers mirror ``hope_ws/src/hope_planner/hope_planner/constants.py`` (ITTF regulation
table) so the simulator and the model-based planner share one consistent world.

The simulation world frame is set **equal to** this HOPE frame: each environment's local origin sits
at the table-surface height (HOPE z = 0), so an asset's environment-local position *is* its HOPE-frame
position. With multiple environments, every environment is an independent court with its own HOPE
origin anchored at that environment's origin.
"""

from __future__ import annotations

from dataclasses import dataclass, field

##
# ITTF regulation table (meters). Mirrors hope_planner.constants.TableParams.
##
TABLE_LENGTH: float = 2.74          # along +X
TABLE_WIDTH: float = 1.525          # along -Y
TABLE_HEIGHT: float = 0.76          # table surface above the floor
TABLE_THICKNESS: float = 0.05       # visual/collision slab thickness of the table top

NET_X: float = 1.37                 # net plane position along X (== TABLE_LENGTH / 2)
NET_HEIGHT: float = 0.1525          # net height above the table surface
NET_OVERHANG: float = 0.15          # net extends this far past each table edge in Y
NET_THICKNESS: float = 0.01         # net slab thickness along X

LINE_WIDTH: float = 0.02            # painted boundary / center line width
LINE_THICKNESS: float = 0.001       # painted line slab thickness (sits just above the surface)

##
# Ball (40 mm). Radius matches both ITTF and Purdue PACE; mass follows Purdue PACE (see below).
##
BALL_RADIUS: float = 0.02           # 40 mm diameter (20 mm radius)
BALL_MASS: float = 0.0034           # 3.4 g — Purdue PACE value (heavier than the 2.7 g ITTF ball)

##
# Coordinate landmarks in the HOPE frame.
##
ORIGIN: tuple[float, float, float] = (0.0, 0.0, 0.0)
TABLE_CENTER: tuple[float, float, float] = (TABLE_LENGTH / 2.0, -TABLE_WIDTH / 2.0, 0.0)
NET_CENTER: tuple[float, float, float] = (NET_X, -TABLE_WIDTH / 2.0, 0.0)
P1_HALF_CENTER: tuple[float, float, float] = (TABLE_LENGTH / 4.0, -TABLE_WIDTH / 2.0, 0.0)
P2_HALF_CENTER: tuple[float, float, float] = (3.0 * TABLE_LENGTH / 4.0, -TABLE_WIDTH / 2.0, 0.0)
FLOOR_Z: float = -TABLE_HEIGHT      # the floor surface, in HOPE z

##
# Robot nominal standing pose, P1 side (HOPE frame).
# From HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md: "Robot nominal standing position
# ~ (-0.5, -0.7625, floor)". X < 0 puts the robot behind the near table end; Y centers it on the
# table width; the pelvis height is added on top of FLOOR_Z by the robot-specific config.
##
P1_STAND_X: float = -0.5
P1_STAND_Y: float = -TABLE_WIDTH / 2.0   # -0.7625, centered on table width


def table_top_center() -> tuple[float, float, float]:
    """Center of the table-top collision/visual slab (its top face is at HOPE z = 0)."""
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
    """Parameters for the scripted ball serve used to reset / visualize ball flight.

    The ball is spawned over one half of the table and launched toward the other side with a slight
    downward component, so a reset shows a full flight -> table bounce -> rising arc. All positions are
    in the HOPE frame; velocities are in m/s in the HOPE frame. Ranges are sampled uniformly per reset.
    """

    # Spawn box over the P2 half (so the ball travels toward the P1-side robot). Spawn height is well
    # above the net top (0.1525 m) so the served arc clears the net at x = 1.37 for ~all resets.
    pos_x_range: tuple[float, float] = (2.0, 2.4)
    pos_y_range: tuple[float, float] = (-1.1, -0.4)
    pos_z_range: tuple[float, float] = (0.55, 0.80)
    # Launch velocity toward P1 (-X), roughly flat / slightly up so it clears the net then arcs down
    # under gravity onto the P1 half, with small lateral spread.
    vel_x_range: tuple[float, float] = (-5.0, -3.5)
    vel_y_range: tuple[float, float] = (-0.4, 0.4)
    vel_z_range: tuple[float, float] = (-0.2, 0.5)
    # Optional initial spin (rad/s) about each HOPE axis; only matters if Magnus is enabled.
    spin_range: tuple[float, float] = (0.0, 0.0)


@dataclass
class BounceMaterials:
    """PhysX contact-material parameters for the ball and the static surfaces.

    The ball and table values follow the **Purdue PACE** model
    (``legged_lab/assets/table_tennis/{ball,table}.py``): table ``restitution = 0.95`` / ``friction = 0.4``
    and ball ``restitution = 0.9`` / ``friction = 0.1``.

    NOTE on combine modes: every material here is created with ``*_combine_mode="multiply"`` (see
    ``table_tennis_env_cfg._surface_material``), so the *effective* coefficient of a ball<->surface
    contact is the **product** of the two materials' values. That product is what lets the net kill the
    ball. Effective ball<->surface **normal restitutions** are therefore:

        table  0.9 * 0.95 = 0.855      floor  0.9 * 0.4 = 0.36      net  0.9 * 0.1 = 0.09

    (Purdue itself uses a single material for the whole table+net under PhysX-default averaging, so in
    their sim the ball *bounces* off the net. We keep multiply + a separate low-restitution net so the
    ball still dies on a net touch — more correct for match play. Adopting Purdue's net behavior would
    mean raising ``net_restitution`` to 0.95.)

    Friction is likewise multiplicative; with the low Purdue ball friction (0.1) the tangential
    (horizontal) bounce is light. Exact horizontal-/racket-restitution matching to the HOPE planner
    (``C_h = 0.75`` / ``C_r = 0.88``) remains a calibration TODO. All knobs live here so a calibration
    pass only edits this one place.
    """

    # Ball (dynamic) — Purdue PACE values.
    ball_restitution: float = 0.9
    ball_static_friction: float = 0.1
    ball_dynamic_friction: float = 0.1
    # Table top — Purdue PACE TableMaterial.
    table_restitution: float = 0.95
    table_static_friction: float = 0.4
    table_dynamic_friction: float = 0.4
    # Floor.
    floor_restitution: float = 0.4
    floor_static_friction: float = 0.8
    floor_dynamic_friction: float = 0.8
    # Net (low restitution so the ball dies on contact; kept separate from Purdue's single material).
    net_restitution: float = 0.1
    net_static_friction: float = 0.5
    net_dynamic_friction: float = 0.5


@dataclass
class OutOfBoundsBox:
    """Axis-aligned region (HOPE frame) outside which the ball is considered dead / out of play.

    Generous margins around the table so the ball can fly well past either player before the episode
    is reset. Used by the termination / serve-reset logic.
    """

    x: tuple[float, float] = (-2.0, 4.74)
    y: tuple[float, float] = (-3.0, 1.5)
    # A ball resting on the floor has center z = FLOOR_Z + BALL_RADIUS = -0.74; trigger just above that
    # (-0.73) so a grounded/missed ball ends the episode promptly instead of idling until timeout.
    z: tuple[float, float] = (FLOOR_Z + BALL_RADIUS + 0.01, 3.0)

    def as_dict(self) -> dict[str, tuple[float, float]]:
        return {"x": self.x, "y": self.y, "z": self.z}
