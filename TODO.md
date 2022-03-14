## TODO

- When transitioning from Feedforward + PI control, there is a noticable droop as the feedforward term disappears. We should have a smoother transition between the two

# 1) Stacking
- Get end points of cubes
- Come up with driver code that string together
  - Path planning from one cube pos to another
  - Trajectory interpolation given a suitable TEnd and the path above
  - Looping over each cube

# 2) Drawing
- Test performance of velocity-based control with pen
- Convert shapes to draw into waypoints or endpoints

# 3) Self-made task



# Refactor via points

Program Flow
-> Bootup initial position
-> Goal waypoints

Calculate A* waypoints from initial position->waypoint_1->...->waypoint_N
-> Create list of via points

for vias in via_point_list:

positions = [[],[],[]]

heavy-lifting calculation
  for each adjacent position:
  - get via point set by A* search
  - Interpolate between via points -> coeffs

for each adjacent position:
- write to servos


# General
[ ] Easy entry point to calibrate each servo with position based offsets (for actual day)