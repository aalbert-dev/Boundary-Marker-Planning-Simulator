# Boundary-Marker-Planning-Simulator
Finds a driveable path on a track defined by boundary markers, for Brandeis Autonomous Robotics Lab.

* Uses delawney triangulation for midpoint detection and hierarchical clustering to find a regression line to follow.
* track generator class generates only circular tracks currently.
* Adds artificial noise to boundary markers to more closely simulate real world scenarios.
* Track generator has parameters to tune number of boundary markers, distance between markers, and radius of track.
