# ExploBench

This repository contains code of different algorithms for autonomous exploration.

## For Developers

* `/visible_map`: the current mapping result.
* `/ground_truth`: the actual underlying world without sensor noise or occlusion.
* `/av1/odom`: the odometry.
* `/av1/cmd_vel`: the twist of robot.

It is suggested to put all robot-related topic into `av1` namespace.

## Algorithms

Currently, two algorithms are developed.

* `find_nearest`: greedy algorithm, which assigns the nearest frontier to robot.
* `nbvplanner`: sampling-based algorithm, maintaining a RRT and selecting one frontier each time

## Dependencies

Please run the command below.

```bash
sudo apt install ros-humble-octomap-ros ros-humble-octomap-msgs 
```