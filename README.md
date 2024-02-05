# PID distance controller

## Start Simulation

    cd ~/ros2_ws && source install/setup.bash

Empty simulation

    ros2 launch rosbot_xl_gazebo empty_simulation.launch.py

Maze simulation

    ros2 launch rosbot_xl_gazebo simulation.launch.py

## Control test
After compile:

### Test in simulation

    ros2 run pid_maze_solver pid_maze_solver
### Test in real robot

    ros2 run pid_maze_solver pid_maze_solver 2
    
## Others
Capture points

    ros2 topic echo /rosbot_xl_base_controller/odom --field pose.pose.position

Manual control

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Visualization