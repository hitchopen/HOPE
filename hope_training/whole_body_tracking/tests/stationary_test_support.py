"""Offline tests of the actual bank loader, sampler, curriculum and resume guard."""
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import torch
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT/'source/whole_body_tracking/whole_body_tracking'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


robust = load('robust_question_test', PKG/'utils/robust_ball_distribution.py')
transport = load('robust_transport_test', PKG/'tasks/tracking/mdp/mocap_transport.py')


def recipe():
    with initialize_config_dir(config_dir=str(ROOT/'cfg'), version_base=None):
        return OmegaConf.to_container(compose(config_name='train', overrides=['task=HOPEPingPongStationaryRally', 'algo=ppo_balanced_hold']), resolve=True)


def initialize_production_velocity_boxes(racket):
    """Run the real YAML translator and constructor's velocity checks without Kit."""
    source = PKG/'tasks/tracking/mdp/hope_commands.py'
    tree = ast.parse(source.read_text())
    config = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RacketTargetCommandCfg')
    defaults = {n.target.id: ast.literal_eval(n.value) for n in config.body
                if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name)
                and n.target.id.startswith('racket_vel_')}
    obj = NS(cfg=NS(**defaults), device='cpu')
    translator = load('robust_command_overrides_test', PKG/'utils/task_command_overrides.py')
    env_cfg = NS(commands=NS(motion=NS(), racket_target=obj.cfg), actions=NS(joint_pos=NS()))
    translator.apply_command_overrides(env_cfg, {'racket': {
        k: v for k, v in racket.items() if k.startswith('vel_') and k in translator._RACKET_ALIASES}}, [])
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RacketTargetCommand')
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    def assignment_to(node, name):
        return isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Attribute) and t.attr == name for t in node.targets)
    start = next(i for i, n in enumerate(init.body) if assignment_to(n, '_vel_range_per_clip_t'))
    end = next(i for i, n in enumerate(init.body) if assignment_to(n, '_pos_range_per_clip_t'))
    exec(compile(ast.Module(body=init.body[start:end], type_ignores=[]), str(source), 'exec'),
         {'torch': torch, 'self': obj})
    return obj








def loader(cfg):
    source = PKG/'tasks/tracking/mdp/hope_commands.py'
    cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'RacketTargetCommand')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_resolve_venue_tuple_bank_path', '_load_venue_tuple_bank')]
    import os
    namespace = dict(np=np, torch=torch, json=json, Path=Path, os=os)
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(source), 'exec'), namespace)
    obj = NS(cfg=NS(**cfg), _venue_tuple_mix_mode='per_swing_bank_v2', device='cpu', _venue_tuple_bank_by_clip={})
    obj._resolve_venue_tuple_bank_path = lambda p: ROOT/p
    namespace['_load_venue_tuple_bank'](obj)
    return obj
