#!/usr/bin/env -S uv run --script


# Create the sphere
sphere = cmds.polySphere(radius=1, name="bouncingSphere")[0]

# Animation parameters
start_frame = 1
end_frame = 240
bounce_height = 5
num_bounces = 5

for i in range(num_bounces):
    # Calculate keyframe times
    up_frame = start_frame + i * (end_frame // num_bounces)
    down_frame = up_frame + (end_frame // (2 * num_bounces))
    next_frame = start_frame + (i + 1) * (end_frame // num_bounces)

    # Set keyframes for bouncing motion
    cmds.setKeyframe(sphere, attribute="translateY", t=up_frame, v=0)
    cmds.setKeyframe(sphere, attribute="translateY", t=down_frame, v=bounce_height)
    cmds.setKeyframe(sphere, attribute="translateY", t=next_frame, v=0)
