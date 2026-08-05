[![CMake](https://github.com/IMRCLab/librigidbodytracker/actions/workflows/cmake.yml/badge.svg)](https://github.com/IMRCLab/librigidbodytracker/actions/workflows/cmake.yml)

# librigidbodytracker
This library helps to track (i.e. estimate the pose) of rigid-bodies.
It assumes that an initial estimate for the pose of each rigid body is given.
The new poses are estimated using the iterative closest point algorithm (ICP) frame-by-frame.

The library is used in the Crazyswarm project.

## Building

See `cmake.yml` workflow for a detailed list of instructions on how to build on Ubuntu.

## Usage

### Playback of a recording

A pointcloud can be recorded in a binary format, for example using the ROS motion capture package. This can be replayed:

```
./playclouds ../example/cfg_000.yaml ../example/recording_000
```