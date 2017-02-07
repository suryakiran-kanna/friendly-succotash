# friendly-succotash
EE5900- Intro to Robotics - Project 4

Please refer to Repository [Wiki Page](https://github.com/suryakiran-kanna/friendly-succotash/wiki) to project description.

This project has Patrol mode and Replay mode.
### Patrol
Patrol spawns a Jackal to a randomly pick a spawn point, plan a patrol path based on closest known waypoint and bag the readings from /scan and /tf topics while patrolling.
### Replay
Replay script to play back the bag file and save a map.
## Instructions to run
### Patrol
* Clone repository `git clone https://github.com/suryakiran-kanna/friendly-succotash.git`
* `cd friendly-succotash/lab_four_ws/`
* `catkin_make`
* `source devel/setup.bash`
* `roslaunch patrolling lab_four_patrol.launch`    

### Replay
* `cd friendly-succotash/lab_four_ws/`
* `source devel/setup.bash`
* `roslaunch patrolling lab_four_replay.launch`
