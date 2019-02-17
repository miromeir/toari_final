# Robotics final project
## Abstract
This is an implementation of maze navigation for Turtlebot 3. The robot should
* Map the maze
* Navigate to maze exit until locating a blue box
* Move the box out of the way
* Continue the navigation to maze exit
This uses a finite state machine and utilizes the ros navigation package.

## Running the code
* Before running the code, a mapping of the maze needs to be run, we used the gmapping to do so. Use the command
  ```bash
  roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
  ```
  And save the map via
  ```bash
  rosrun map_server map_saver -f map
  ```
* Make sure the navigation node is running
  ```bash
  roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
  ```
