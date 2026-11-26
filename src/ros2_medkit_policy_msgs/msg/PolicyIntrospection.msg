# What a learned robot policy (ACT, diffusion policy, VLA, ...) predicted at
# one inference. A policy publishes one message per inference it runs.

# header.stamp: when the inference ran. ros2_medkit_policy_bridge does not
# read it; it is there for recordings and other consumers.
std_msgs/Header header

# Task attempt counter, set by whoever runs the policy. A new value is a new
# attempt.
uint32 episode

# Control step within the episode at which the inference ran.
uint32 step

# Predicted actions for the next chunk_length control steps, row major
# (chunk_length x action_dim), in the units the robot is commanded in.
# A policy that predicts one action per inference sends chunk_length 1.
uint16 chunk_length
uint16 action_dim
float32[] chunk

# Wall time the inference took, in milliseconds. 0, the default, means it was
# not measured; so does a negative or a non-finite value.
float32 inference_ms
