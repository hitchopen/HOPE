"""HOPE model-based racket planner.

Pure-Python core (constants, estimator, predictor, target planner, contact
model, quaternion utilities, pipeline) with a thin ROS 2 node wrapper in
``node.py``.
"""

from .ball_contact import orient_normal, predict_paddle_contact
from .ball_state_estimator import BallStateEstimator
from .ball_trajectory_predictor import BallTrajectoryPredictor, StrikeTarget
from .constants import (
    BallPhysics,
    PlannerConfig,
    TableParams,
    load_ball_physics,
    load_paddle_params,
)
from .planner import HOPEPlanner
from .quaternion_utils import normal_to_quaternion
from .racket_target_planner import RacketCommand, RacketTargetPlanner

__all__ = [
    "BallPhysics",
    "PlannerConfig",
    "TableParams",
    "load_ball_physics",
    "load_paddle_params",
    "BallStateEstimator",
    "BallTrajectoryPredictor",
    "StrikeTarget",
    "RacketTargetPlanner",
    "RacketCommand",
    "HOPEPlanner",
    "normal_to_quaternion",
    "orient_normal",
    "predict_paddle_contact",
]
