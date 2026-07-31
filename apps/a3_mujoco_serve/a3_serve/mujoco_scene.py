"""Official A3 MJCF plus table, net, ball, drag, and racket contacts."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

from .constants import JOINT_NAMES, RIGHT_ARM_JOINTS


def _damping_ratio(restitution: float) -> float:
    value = float(np.clip(restitution, 1.0e-3, 0.999))
    logarithm = math.log(value)
    return -logarithm / math.sqrt(math.pi * math.pi + logarithm * logarithm)


@dataclass(frozen=True)
class BallReplay:
    first_bounce_table: np.ndarray | None
    second_bounce_table: np.ndarray | None
    net_clearance_m: float | None
    net_contact: bool
    racket_contact: bool
    bounces_inside_table: bool
    post_contact_velocity_world: np.ndarray | None = None
    net_x_table: float = 1.37


class A3ServeScene:
    """Kinematic A3 replay around a dynamically simulated ping-pong ball.

    The installed high-level controller owns the real joint servos.  For offline
    generation, the CSV joint samples therefore act as a kinematic position
    source while MuJoCo integrates the ball and resolves its contacts.  This
    deliberately does not pretend to model the proprietary high-level servo.
    """

    def __init__(self, model_xml: str | Path, physics: dict, model_cfg: dict) -> None:
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover - exercised by CLI diagnostics
            raise RuntimeError(
                "MuJoCo is required; install this app with `pip install -e .`"
            ) from exc

        self.mj = mujoco
        self.model_xml = Path(model_xml).expanduser().resolve()
        self.physics = physics
        self.racket_site_name = str(model_cfg["racket_site"])
        self.racket_geom_name = str(model_cfg["racket_geom"])
        self.racket_normal_axis = int(model_cfg["racket_normal_axis"])
        self._build_model()
        self._resolve_ids()

    def _build_model(self) -> None:
        mj = self.mj
        spec = mj.MjSpec.from_file(str(self.model_xml))
        world = spec.worldbody
        table = self.physics["table"]
        net = self.physics["net"]
        ball = self.physics["ball"]
        contact = self.physics["contact"]

        self.length = float(table["length"])
        self.width = float(table["width"])
        self.table_height = float(table["height"])
        self.table_thickness = float(table["thickness"])
        self.net_height = float(net["height"])
        self.net_x_table = float(net["x_position"])
        self.net_thickness = float(net["thickness"])
        self.ball_radius = float(ball["radius"])
        self.ball_mass = float(ball["mass"])
        self.drag_k = float(self.physics["drag"]["k"])
        self.velocity_clip = float(self.physics["drag"]["velocity_clip"])
        self.near_edge_x = float(table["near_edge_x_mujoco"])
        self.offset = np.array(
            [self.near_edge_x, self.width / 2.0, self.table_height],
            dtype=np.float64,
        )

        ball_type, ball_affinity = 8, 15
        surface_type, surface_affinity = 8, 8
        top = world.add_body(
            name="hope_serve_table",
            pos=[
                self.near_edge_x + self.length / 2.0,
                0.0,
                self.table_height - self.table_thickness / 2.0,
            ],
        )
        top.add_geom(
            name="hope_serve_table_geom",
            type=mj.mjtGeom.mjGEOM_BOX,
            size=[self.length / 2.0, self.width / 2.0, self.table_thickness / 2.0],
            contype=surface_type,
            conaffinity=surface_affinity,
            rgba=[0.10, 0.35, 0.55, 1.0],
        )
        net_body = world.add_body(
            name="hope_serve_net",
            pos=[
                self.near_edge_x + self.net_x_table,
                0.0,
                self.table_height + self.net_height / 2.0,
            ],
        )
        net_body.add_geom(
            name="hope_serve_net_geom",
            type=mj.mjtGeom.mjGEOM_BOX,
            size=[
                self.net_thickness / 2.0,
                self.width / 2.0 + float(net["overhang"]),
                self.net_height / 2.0,
            ],
            contype=surface_type,
            conaffinity=surface_affinity,
            rgba=[0.9, 0.9, 0.9, 0.35],
        )
        ball_body = world.add_body(
            name="hope_serve_ball",
            pos=[self.near_edge_x + 0.5, 0.0, self.table_height + 0.5],
        )
        ball_body.add_freejoint(name="hope_serve_ball_free_joint")
        ball_body.add_geom(
            name="hope_serve_ball_geom",
            type=mj.mjtGeom.mjGEOM_SPHERE,
            size=[self.ball_radius, 0.0, 0.0],
            mass=self.ball_mass,
            contype=ball_type,
            conaffinity=ball_affinity,
            rgba=[1.0, 0.55, 0.0, 1.0],
        )

        def add_pair(surface: str, surface_key: str, default_e: float, default_f: float) -> None:
            settings = contact.get(surface_key, {})
            restitution = float(settings.get("restitution", default_e))
            friction = float(settings.get("dynamic_friction", default_f))
            pair = spec.add_pair(geomname1="hope_serve_ball_geom", geomname2=surface)
            pair.solref = [0.03 if restitution > 0.8 else 0.02,
                           _damping_ratio(restitution)]
            pair.friction = [friction, friction, 0.005, 1.0e-4, 1.0e-4]
            pair.condim = 3

        add_pair("hope_serve_table_geom", "table", 0.9215, 0.40)
        add_pair("hope_serve_net_geom", "net", 0.10, 0.50)
        add_pair(self.racket_geom_name, "paddle", 0.654, 0.60)
        add_pair("floor", "floor", 0.40, 0.80)

        if spec.keys:
            key = spec.keys[0]
            key.qpos = list(np.asarray(key.qpos, dtype=np.float64)) + [
                self.near_edge_x + 0.5,
                0.0,
                self.table_height + 0.5,
                1.0,
                0.0,
                0.0,
                0.0,
            ]
        self.model = spec.compile()
        self.data = mj.MjData(self.model)
        self.model.opt.gravity[:] = [0.0, 0.0, -float(self.physics["gravity"])]

    def _id(self, object_type, name: str) -> int:
        value = self.mj.mj_name2id(self.model, object_type, name)
        if value < 0:
            raise ValueError(f"required MuJoCo object not found: {name}")
        return int(value)

    def _resolve_ids(self) -> None:
        mj = self.mj
        m = self.model
        self.joint_ids = np.array(
            [self._id(mj.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES], dtype=int
        )
        self.qpos_addresses = m.jnt_qposadr[self.joint_ids].astype(int)
        self.dof_addresses = m.jnt_dofadr[self.joint_ids].astype(int)
        self.right_indices = np.array([JOINT_NAMES.index(name) for name in RIGHT_ARM_JOINTS])
        self.right_dofs = self.dof_addresses[self.right_indices]
        self.racket_site_id = self._id(mj.mjtObj.mjOBJ_SITE, self.racket_site_name)
        self.racket_geom_id = self._id(mj.mjtObj.mjOBJ_GEOM, self.racket_geom_name)
        self.ball_joint_id = self._id(mj.mjtObj.mjOBJ_JOINT, "hope_serve_ball_free_joint")
        self.ball_qpos_address = int(m.jnt_qposadr[self.ball_joint_id])
        self.ball_dof_address = int(m.jnt_dofadr[self.ball_joint_id])
        self.ball_body_id = self._id(mj.mjtObj.mjOBJ_BODY, "hope_serve_ball")
        self.ball_geom_id = self._id(mj.mjtObj.mjOBJ_GEOM, "hope_serve_ball_geom")
        self.table_geom_id = self._id(mj.mjtObj.mjOBJ_GEOM, "hope_serve_table_geom")
        self.net_geom_id = self._id(mj.mjtObj.mjOBJ_GEOM, "hope_serve_net_geom")
        self.floor_geom_id = self._id(mj.mjtObj.mjOBJ_GEOM, "floor")
        self.base_joint_id = self._id(mj.mjtObj.mjOBJ_JOINT, "pelvis_free_joint")
        self.base_qpos_address = int(m.jnt_qposadr[self.base_joint_id])
        self.base_dof_address = int(m.jnt_dofadr[self.base_joint_id])
        self.substeps_per_source = max(
            1, int(round((1.0 / 200.0) / float(self.model.opt.timestep)))
        )

    def reset(self, joint_positions: np.ndarray) -> None:
        if self.model.nkey:
            self.mj.mj_resetDataKeyframe(self.model, self.data, 0)
        else:
            self.mj.mj_resetData(self.model, self.data)
        self.set_joint_positions(joint_positions)
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.xfrc_applied[:] = 0.0
        self._set_ball(
            [self.near_edge_x + self.length + 2.0, 0.0, self.table_height + 2.0],
            [0.0, 0.0, 0.0],
        )
        self.mj.mj_forward(self.model, self.data)

    def set_joint_positions(self, joint_positions: np.ndarray) -> None:
        positions = np.asarray(joint_positions, dtype=np.float64).reshape(len(JOINT_NAMES))
        self.data.qpos[self.qpos_addresses] = positions

    def set_right_arm(self, right_arm: np.ndarray) -> None:
        values = np.asarray(right_arm, dtype=np.float64).reshape(len(RIGHT_ARM_JOINTS))
        self.data.qpos[self.qpos_addresses[self.right_indices]] = values

    def _pin_robot(self, joint_positions: np.ndarray, joint_velocities: np.ndarray | None = None) -> None:
        self.set_joint_positions(joint_positions)
        self.data.qvel[self.base_dof_address:self.base_dof_address + 6] = 0.0
        if joint_velocities is None:
            self.data.qvel[self.dof_addresses] = 0.0
        else:
            self.data.qvel[self.dof_addresses] = np.asarray(joint_velocities, dtype=np.float64)
        self.data.ctrl[:] = 0.0

    def _set_ball(self, position: Iterable[float], velocity: Iterable[float]) -> None:
        q = self.ball_qpos_address
        v = self.ball_dof_address
        self.data.qpos[q:q + 3] = np.asarray(position, dtype=np.float64)
        self.data.qpos[q + 3:q + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[v:v + 3] = np.asarray(velocity, dtype=np.float64)
        self.data.qvel[v + 3:v + 6] = 0.0

    def _apply_drag(self) -> None:
        velocity = self.data.qvel[self.ball_dof_address:self.ball_dof_address + 3]
        speed = min(float(np.linalg.norm(velocity)), self.velocity_clip)
        self.data.xfrc_applied[self.ball_body_id, :3] = (
            -self.ball_mass * self.drag_k * speed * velocity
        )

    def racket_pose(self) -> tuple[np.ndarray, np.ndarray]:
        self.mj.mj_forward(self.model, self.data)
        position = self.data.site_xpos[self.racket_site_id].copy()
        rotation = self.data.site_xmat[self.racket_site_id].reshape(3, 3).copy()
        return position, rotation

    def racket_jacobian(self) -> np.ndarray:
        jac_pos = np.zeros((3, self.model.nv), dtype=np.float64)
        jac_rot = np.zeros((3, self.model.nv), dtype=np.float64)
        self.mj.mj_jacSite(
            self.model, self.data, jac_pos, jac_rot, self.racket_site_id
        )
        return np.vstack((jac_pos[:, self.right_dofs], jac_rot[:, self.right_dofs]))

    def to_table(self, world_position: np.ndarray) -> np.ndarray:
        return np.asarray(world_position, dtype=np.float64) - self.offset

    def _contact_flags(self) -> tuple[bool, bool]:
        racket = False
        net = False
        for index in range(self.data.ncon):
            pair = {int(self.data.contact[index].geom1), int(self.data.contact[index].geom2)}
            racket = racket or pair == {self.ball_geom_id, self.racket_geom_id}
            net = net or pair == {self.ball_geom_id, self.net_geom_id}
        return racket, net

    def _replay_metrics(
        self,
        fixed_joint_positions: np.ndarray,
        duration_s: float,
        trajectory: np.ndarray | None,
        source_hz: float,
        require_racket_contact: bool,
    ) -> BallReplay:
        bounces: list[np.ndarray] = []
        net_clearance: float | None = None
        net_contact = False
        racket_contact = False
        post_contact_velocity: np.ndarray | None = None
        dt = float(self.model.opt.timestep)
        steps = int(math.ceil(duration_s / dt))
        q_fixed = np.asarray(fixed_joint_positions, dtype=np.float64)

        for step in range(steps):
            time_s = step * dt
            if trajectory is None:
                q = q_fixed
                qd = np.zeros_like(q)
            else:
                source = min(time_s * source_hz, trajectory.shape[0] - 1.0)
                lower = int(math.floor(source))
                upper = min(lower + 1, trajectory.shape[0] - 1)
                fraction = source - lower
                q = trajectory[lower] + fraction * (trajectory[upper] - trajectory[lower])
                qd = (trajectory[upper] - trajectory[lower]) * source_hz
            self._pin_robot(q, qd)
            before = self.data.qpos[
                self.ball_qpos_address:self.ball_qpos_address + 3
            ].copy()
            self._apply_drag()
            self.mj.mj_step(self.model, self.data)
            after = self.data.qpos[
                self.ball_qpos_address:self.ball_qpos_address + 3
            ].copy()
            hit_racket, hit_net = self._contact_flags()
            if hit_racket and not racket_contact:
                racket_contact = True
            if racket_contact and post_contact_velocity is None and not hit_racket:
                post_contact_velocity = self.data.qvel[
                    self.ball_dof_address:self.ball_dof_address + 3
                ].copy()
            net_contact = net_contact or hit_net

            x_before = before[0] - self.offset[0]
            x_after = after[0] - self.offset[0]
            if x_before < self.net_x_table <= x_after:
                fraction = (self.net_x_table - x_before) / max(x_after - x_before, 1.0e-12)
                height = before[2] + fraction * (after[2] - before[2]) - self.offset[2]
                if not require_racket_contact or racket_contact:
                    net_clearance = float(height)

            z_before = before[2] - self.offset[2]
            z_after = after[2] - self.offset[2]
            if z_before > self.ball_radius >= z_after:
                fraction = (self.ball_radius - z_before) / min(z_after - z_before, -1.0e-12)
                crossing = before + fraction * (after - before)
                table_crossing = self.to_table(crossing)
                if not require_racket_contact or racket_contact:
                    if not bounces or float(np.linalg.norm(table_crossing[:2] - bounces[-1][:2])) > 0.02:
                        bounces.append(table_crossing)
            if len(bounces) >= 2:
                break
            if after[2] < -0.25:
                break

        self.data.xfrc_applied[self.ball_body_id, :3] = 0.0
        first = bounces[0] if bounces else None
        second = bounces[1] if len(bounces) > 1 else None

        def inside(value: np.ndarray | None) -> bool:
            return bool(
                value is not None
                and 0.0 <= value[0] <= self.length
                and -self.width <= value[1] <= 0.0
            )

        return BallReplay(
            first_bounce_table=first,
            second_bounce_table=second,
            net_clearance_m=net_clearance,
            net_contact=net_contact,
            racket_contact=racket_contact,
            bounces_inside_table=inside(first) and inside(second),
            post_contact_velocity_world=post_contact_velocity,
            net_x_table=self.net_x_table,
        )

    def simulate_outgoing_ball(
        self,
        fixed_joint_positions: np.ndarray,
        position_world: np.ndarray,
        velocity_world: np.ndarray,
        *,
        duration_s: float,
    ) -> BallReplay:
        self.reset(fixed_joint_positions)
        self._set_ball(position_world, velocity_world)
        self.mj.mj_forward(self.model, self.data)
        return self._replay_metrics(
            fixed_joint_positions,
            duration_s,
            trajectory=None,
            source_hz=200.0,
            require_racket_contact=False,
        )

    def simulate_joint_replay(
        self,
        joint_trajectory: np.ndarray,
        ball_position_world: np.ndarray,
        ball_velocity_world: np.ndarray,
        *,
        source_hz: float,
        duration_s: float,
    ) -> BallReplay:
        trajectory = np.asarray(joint_trajectory, dtype=np.float64)
        self.reset(trajectory[0])
        self._set_ball(ball_position_world, ball_velocity_world)
        self.mj.mj_forward(self.model, self.data)
        return self._replay_metrics(
            trajectory[0],
            duration_s,
            trajectory=trajectory,
            source_hz=source_hz,
            require_racket_contact=True,
        )

    def collision_scan(
        self, joint_trajectory: np.ndarray, *, stride: int = 10
    ) -> list[dict[str, object]]:
        """Return robot-vs-robot contact pairs observed along sampled configurations."""

        output: list[dict[str, object]] = []
        ignored = {
            self.ball_geom_id,
            self.table_geom_id,
            self.net_geom_id,
            self.floor_geom_id,
        }
        for frame in range(0, len(joint_trajectory), max(1, int(stride))):
            self.reset(joint_trajectory[frame])
            seen: set[tuple[str, str]] = set()
            for index in range(self.data.ncon):
                contact = self.data.contact[index]
                geom1, geom2 = int(contact.geom1), int(contact.geom2)
                if geom1 in ignored or geom2 in ignored:
                    continue
                name1 = self.mj.mj_id2name(
                    self.model, self.mj.mjtObj.mjOBJ_GEOM, geom1
                ) or f"geom_{geom1}"
                name2 = self.mj.mj_id2name(
                    self.model, self.mj.mjtObj.mjOBJ_GEOM, geom2
                ) or f"geom_{geom2}"
                pair = tuple(sorted((name1, name2)))
                if pair in seen:
                    continue
                seen.add(pair)
                output.append({"frame": frame, "geoms": list(pair)})
        return output

