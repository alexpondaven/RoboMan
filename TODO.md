# TODO
## Task 2.1 Cube Translation, Task 2.2 Cube Rotation
- [ ] {Alex} Skeleton code for the entirety of Task 2.1/2, able to specify the orientations and locations of the cubes
  - [ ] Function that takes in initial and desired cube states:
    - `initial_cube_state`:
      - `initial_cube_state = [ [3,-8, 1, 0]; [9, 0, 1, 0]; [6, 6, 1, 0] ].';`
      - (cube grid i, cube grid j, cube stack height, cube orientation)
      - Cube orientation (multiply by 90 degrees to get sensible rotation):
        - `0`: upwards
        - `90`: towards robot
        - `-90`: away from robot
        - `180`: downwards
    - `desired_cube_state`: 
      - Similar in structure to `initial_cube_state`. 
      - Assume that cubes maintain the same index across the two variables
    - Outputs:
    - [ ] Desired intermediate waypoints that result in all cubes facing up
    - [ ] Valid occupancy grids for planning from one waypoint to the next
    - [ ] A set of collision-free via points for the end effector from one waypoint to the next
- [ ] Create a way for us to manually declare path of cubes just in case, so it can be changed on demo day
- [x] {TY} Refactor `inverseKin2` / `inverseKinDynamixel2` to be more elegant - `inverseKinDynamixel2` should be a wrapper around `inverseKin2`
- [x] {TY} Software API for interfacing with Servo 5 (function to open and close gripper)
- [x] {TY} Write cube grabbing/depositing function - pseudocode:
  - We can call `mainServoLoop` within a wrapper function with the relevant arguments.
  - Procedure:
    1. Grabbing Cube
       1. *entry point: End effector at goal point
       2. Move end effector to grasp cube (hard-code downwards 20mm?)
       3. Close Gripper
       4. Move end effector to next start point (hard-code upwards 20mm?)
       5. *exit point: End effector follows the next set of via points
    2. Dropping Cube
       1. *entry point: End effector at goal point
       2. Move end effector to deposit cube (hard-code downwards 20mm?)
       3. Open Gripper
       4. Move end effector to next start point (hard-code upwards 20mm?)
       5. *exit point: End effector follows the next set of via points
- [ ] {TY} Expand A* Search space of theta_g to +- pi/2 (with sensible intervals)
- [ ] {Alex} Tune variable `Tend` generation in `interpViaPoints`
- [ ] Test function with arbitrary cube locations, orientations.


## Task 2.3 Cube Stacking
- [ ] Skeleton code for the entirety of Task 2.3, able to specify the orientations and locations of the cubes
- Similar to Task 2.1, we will take in initial positions of the cubes and a target position to stack the cubes on.
- [ ] Function that takes in initial cube positions and orientations, and a target cube-stacking position. Outputs:
  - [ ] Desired intermediate waypoints

## Task 3 Drawing
- [ ] Figure out starting position of robot and pen
- [ ] Picking up pen with gripper
  1. Get to position of pen
  2. close gripper
- [ ] Change end effector position to end of pen
  - Can be done by increasing L4 (possibly a bool parameter in IK that says whether we are holding pen or not can allow easy switching)
- [ ] Test performance of velocity-based control with pen (may tilt)
  - [ ] Make sure 3d printed gripper can hold the pen and it doesn't tilt (find out coords to pick up vertically)
  - [ ] Make sure it can get to all positions possible on the drawing board
- [x] Convert shapes to draw into waypoints or endpoints
  - [x] HELPER: Translate center of the arc, the radius, the degrees of the arc and the start and end co-ordinates into set of waypoints along this arc
  - [x] HELPER: Translation function for interpolating line between waypoints (should have this already)
- [ ] Plan out entire trajectory
  1. Pick up pen
  2. 3 straight lines
  3. 1 partial circle of 270 degrees with constant radius
  4. Put back pen
- [ ] Tune values (can also be tuned on the day)
  1. Number waypoints
  2. Tend for each segment?
- [ ] implentment functions fo line/arc drawing

- [ ] 
- [ ] making thetaG variable axross the drwaing 

## Task 4 Self-Proposed task
- [ ] CAD the gripper to rotate through translation
- [ ] Extend wire of buzzer
- [ ] Plan trajectory of wire
- [ ] Code structure
  - [ ] Helper: Theta 5 (how open gripper is) -> Rotation of gripper
  - [ ] Helper: Path generation
    1. Take waypoints - corners? and interpolate lines between them
    2. More adaptable - take picture of path to determine waypoints, and then follow these
    3. Determine how to rotate based on direction of path
  - [ ] Main
    1. 
- [ ] Steps
  1. Draw path by hand (or declare waypoints manually)
  e.g.
  ¦
  ¦
   \
    \
    /
   /
  ¦
  ¦
  2. Determine angle of buzzer depending on direction of path
  3. 





## General
- [ ] Easy entry point to calibrate each servo with position based offsets (for actual day)
- [ ] Fix droop when transitioning from Feedforward + PI control
  - [ ] Perhaps, we can monitor servo load before setting all velocities to 0, or fade them to 0 more slowly.
- [ ] Benchmark code to determine performance bottleneck (if any) in control loop


Notes:
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

# TASK LIST Tue 15/03/2022
- General
- [ ] Benchmark code
- Task 2
  - [ ] Tune occupancy grid generation to match the real-world dimensions of each cubes and cube holders
  - [x] Test cube picking and dropping with 1 cube
  - [ ] Test and tune cube translation for 1 cube in 3 locations
  - [ ] Test cube rotation for 1 cube in 3 locations
- Task 3
  - [x] Convert shapes to draw into waypoints or endpoints
  - [ ] Figure out starting position of robot and pen
  - [ ] Change end effector position to end of pen


# TASK LIST Wed 16/03/2022

# TASK LIST Thu 17/03/2022
- Internal **Soft** Deadline to implement Task 2.1, 2.2.

# TASK LIST Fri 18/03/2022
- Internal **HARD** Deadline to **film** Task 2.1, 2.2.
- Internal Deadline to implement Task 3.

# TASK LIST Sat 19/03/2022
- Internal **HARD** Deadline to **film** Task 3.
### By now, we **should** have finished filming Tasks 2.1, 2.2 and 3.
- Start thinking about implementing self-proposed task.
- Start writing report



NOTES:
- For 
cubePickPlace([225, 0, 50, -pi/2], [225, 0, 30, -pi/2], [225, 0, 50, -pi/2], true, port_num)
cubePickPlace([225, 0, 50, 0], [225, 0, 20, 0], [225, 0, 50, 0], true, port_num)

Things to try:
- Change the grid resolution in A*
- Tune the PID parameters back to default
  - Decrease P gains to stop sticking to positions?
- Tune parameters for new gripper (first check it can get to 2nd placeholder)
- Interpolation for theta G in A*
- Destination cube height solution


startIdx =

     3    18    13    10


goalIdx =

     5     5    18     6