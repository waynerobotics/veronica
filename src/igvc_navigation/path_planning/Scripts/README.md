
## Scripts and their usage/explanations.

### [Basic Follower](basic_follower.py)

This subscribes to the /basic_follower topic and reads a geometry_msgs/Twist message).When the script starts, the x, y, theta is initialized to 0.0. When a  

Basic Usage: `rostopic pub -r 10 /basic_waypoint geometry_msgs/Twist  "{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.0}}"`




