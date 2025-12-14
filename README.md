# CGI Tools — First Assessment

The project procedurally generates a infinite Möbius-style stair mesh and animates a bouncing ball rig that interacts with the stair surface using face normals for orientation.

## Project Overview

This tool demonstrates procedural modelling, rigging, and animation in Autodesk Maya using:

- `maya.cmds`
- `maya.api.OpenMaya`
- `numpy`

The main goal is to explore geometry-driven animation, where a bouncing ball follows a path derived from mesh face centers and aligns its rotation to face normals to maintain correct orientation during motion.

## Core Features

### Möbius Stair Generator
- Procedurally creates a stair-like strip mesh
- Applies extrusion, twist, and bend deformers
- Identifies walkable / bounceable faces based on face normal direction
- Computes ordered face centers and normals for animation control

### Bouncing Ball System
- Creates a polygon sphere with a simple rig:
  - Global control
  - Rotation control
  - Squash & stretch scale control
- Computes bounce trajectories using face centers and interpolated apex points
- Animates translation, rotation, and scale over time
- Aligns the ball’s orientation to surface normals using quaternion-based rotation.

### Animation Logic
- Bounce arcs are computed using triangle paths `(contact → apex → contact)`
- Squash and stretch is applied on impact and takeoff
- Rotation continuity is preserved across frames
