from sim import simulator
from trackgen import trackgen

# track generator
t = trackgen()

# get points to use as boundary markers
points = t.gen() 

# get exact robot pose
robot_pose = t.robot_pose

# create simulator 
s = simulator()

# add boundary points
[s.addMarker(x, y) for x, y in points]

# add robot
s.addRobot(robot_pose)

# calculate plan 
s.simulate()