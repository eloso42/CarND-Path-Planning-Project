Model Documentation
===================

How path's are generated
------------------------

### Behavior Planning

On the top level, we have a state machine consisting of the two states

- KEEP_LANE and
- SWITCH_LANE

together with a destination lane (i.e. the lane we want to keep or switch to).

In KEEP_LANE, we evaluate the cars in front of us. For each lane we search
for the slowest car within a 90 meters range to determine the "lane velocity".
If there is a lane with a velocity higher than 0.5 m/s above the current lane
velocity, the nearest car is less than 60m away, and it is safe to switch lanes,
we set the new lane as our destination lane change the state to SWITCH_LANE.

In SWITCH_LANE, we just wait until we reached the destination lane and then
change back to KEEP_LANE.

### Velocity Controller

On the second level we determine the destination velocity (i.e. how fast
we want to drive). If there is no car directly in front of us, we use a
little bit less than the maximum allowed speed of 50mph. If there is a
car less then 30 meters in front of us, we set our velocity to the same of
the car in front of us. If it is only 25 meters away, we set it to 2 m/s
below that car.

### Trajectory Generation

This is more or less copied from the Q&A video. So I use a spline that
is evaluated in car coordinates to approximate the right speed by the
distance of evaluation points. This distance is calculated using the
current velocity, the destination velocity and the maximum acceleration
(see function nextVel() in the code).
