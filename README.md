# Boundary-Marker-Planning-Simulator
Finds a driveable path on a track defined by boundary markers, for Brandeis Autonomous Robotics Lab.

* Uses delawney triangulation for midpoint detection and hierarchical clustering to find a regression line to follow.
* track generator class generates only circular tracks currently.
* Adds artificial noise to boundary markers to more closely simulate real world scenarios.
* Track generator has parameters to tune number of boundary markers, distance between markers, and radius of track.
* Planning code is in planner.py
* Simulation code is in sim.py
* Track generator code is in trackgen.py

Run with "python3 main.py"

# sim.py
1. Get track from trackgen.py
2. Add noise to boundary marker points
3. Set robot default pose
4. For N steps run the planner with the same boundary points
5. At each step recalculate the robot pose based on the previous transform + rotation

# psuedocode for simulator.simulate()
```
current_pose = starting_pose
boundary_points = getPointsFromTrackGen()
While i < N:
  current_pose = getPoseFromPlanner(current_pose, boundary_points)
  i++
  ```
```getPoseFromPlanner()``` accepts a current pose and series of boundary points as inputs and calculates the path to follow along with displaying the map and getting the new pose of the robot.  
```getPointsFromTrackGen()``` represents the points from trackgen with added noise for more realism. 

# trackgen.py
1. set robot default pose
2. create circular track defined by (x, y) points
3. store track for access in sim.py

# planner.py
1. Accept as input a series of points representing the boundary of a track and a pose of a robot
2. Filter out points based on a mask that looks ahead of the robot
3. Order points based on distance to robot 
4. Select which points 
5. Calculate mid points for the series of points 
6. Cluster points based on average distance to each other 
7. Take largest cluster as the set of points to draw a path through
8. Use quadratic regression on set of points to find a spline that fits the mid points well
9. Add boundary points, path, and robot to map and display the map as an image

# linefit.py
1. Use numpy.polyfit to get quadratic coefficents from a set of (x, y) points

# intersection.py
1. Checks if lines (x1, y1) and (x2, y2) intersect

# main.py
1. Get robot pose
2. Create simulator object and add list of boundary points to simulator
3. Add robot pose to simulator
4. Run simulation

