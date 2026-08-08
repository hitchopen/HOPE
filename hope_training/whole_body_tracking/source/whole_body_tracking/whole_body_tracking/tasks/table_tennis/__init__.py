"""Standard table-tennis match simulation environment (HOPE frame).

A modular Isaac Lab task that simulates realistic ping-pong ball flight / bounce / racket+table contact
together with a humanoid robot, in the canonical HOPE world frame. Robot-specific environments and Gym
registrations live under :mod:`.config` (e.g. ``config/agibot_a3``); they are auto-imported by
``whole_body_tracking.tasks`` via ``isaaclab_tasks.utils.import_packages``.
"""
