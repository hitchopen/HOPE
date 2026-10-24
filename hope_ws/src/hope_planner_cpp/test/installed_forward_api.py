#!/usr/bin/env python3
"""Build a separate consumer using only installed exported V2 targets and headers."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--prefix', required=True)
args = parser.parse_args()
with tempfile.TemporaryDirectory(prefix='hope-v2-installed-api-') as temporary:
    root = Path(temporary)
    (root / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(v2_consumer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(hope_planner_cpp REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE hope_planner_cpp::hope_v2_forward_physics)
''')
    (root / 'main.cpp').write_text('''#include <hope_planner_cpp/common_ball_physics.hpp>
#include <cmath>
int main() {
  using namespace hope_planner_cpp;
  ModelState state;
  state.position = Vec3(1., -.7, .6);
  state.velocity = Vec3(-2., 0., 1.);
  auto result = state;
  integrate_ball(result, .01, BallPhysics{});
  return !result.position.allFinite() || result.spin.usable() ||
         !(result.position.x() < state.position.x());
}
''')
    env = dict(os.environ)
    env['CMAKE_PREFIX_PATH'] = args.prefix + os.pathsep + env.get('CMAKE_PREFIX_PATH', '')
    subprocess.run(['cmake', '-S', str(root), '-B', str(root / 'build')], env=env, check=True)
    subprocess.run(['cmake', '--build', str(root / 'build'), '-j2'], env=env, check=True)
    subprocess.run([str(root / 'build/consumer')], check=True)
print('PASS: independent installed forward-API consumer')
