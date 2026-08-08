"""HOPE model-based racket planner (Stages 1-3).

Pure-Python core (constants, estimator, predictor, target planner, pipeline,
quaternion utils) with a thin ROS 2 node wrapper in node.py.
"""

from .ball_contact import orient_normal, predict_paddle_contact
from .ball_kalman_estimator import BallKalmanEstimator
from .ball_state_estimator import BallStateEstimator
from .ball_trajectory_predictor import BallTrajectoryPredictor, StrikeTarget
from .constants import BallPhysics, PlannerConfig, TableParams
from .planner import HOPEPlanner
from .quaternion_utils import normal_to_quaternion
from .racket_target_planner import RacketCommand, RacketTargetPlanner
from .spin_estimator import SpinFromQuats
from .strike_spec_planner import StrikeSpec, StrikeSpecPlanner

__all__ = [
    "BallPhysics",
    "PlannerConfig",
    "TableParams",
    "BallKalmanEstimator",
    "BallStateEstimator",
    "SpinFromQuats",
    "BallTrajectoryPredictor",
    "StrikeTarget",
    "RacketTargetPlanner",
    "RacketCommand",
    "HOPEPlanner",
    "normal_to_quaternion",
    "orient_normal",
    "predict_paddle_contact",
    "StrikeSpec",
    "StrikeSpecPlanner",
]
