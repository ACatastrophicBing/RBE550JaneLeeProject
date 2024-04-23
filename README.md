# RobotSim373

A simple robot simulator based on Box2D and visualization with matplotlib.

## Install

from Anaconda prompt do:

```
pip install Box2d

pip install "git+git://github.com/bblais/RobotSim373" --upgrade
```
TODO :
Creation of a couple of actual maps (Tim)
    self.obstacles.append(Box(self.env, x=center_of_obj, y=center_of_obj,
                    width=width_of_box, angle=0, color='purple', density=1000)) # Change density to something else if you want object to move
Trajectory Controller (Can)

path_plan function needs to be called in act to tell the robot the next waypoint to drive to
path_plan function in Map needs to return a path for Trajectory Controller

LIDAR needs to be optimized (Chandler) using ktree and the vertices?
add linear damping to the humans

Measurement code for ranking algorithms
