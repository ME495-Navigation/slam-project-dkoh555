# NUSLAM
Author: Damien Koh

## Package Description
This package uses `nusim` alongside other packages to simulate a turtlebot in ROS utilizing SLAM for accurate positioning, which is visualized in rviz.

## Quickstart
1. Use the following command to run the SLAM control nodes along with the simulator launch file:
    ```
    ros2 launch nuslam slam.launch.xml
    ``` 
    The robot can be navigated using keyboard teleop (default), or automatically in a circular path.

    1. To run automatically in a circle path, run the following:
        ```
        ros2 launch nuslam slam.launch.xml cmd_src:=circle
        ```
        1. To prompt the robot to begin moving use:
            ```
            ros2 service call /control nuturtle_control/srv/Control '{velocity: 0.25, radius: 0.5}'
            ```
            Feel free to adjust values.
        2. To stop the robot from moving use:
            ```
            ros2 service call /stop std_srvs/srv/Empty '{}'
            ```
    2. To control the robot with keyboard teleop, run:
        ```
        ros2 launch nuslam slam.launch.xml cmd_src:=teleop
        ```

2. Additional parameters for the simulation can be found and adjusted in NUSIM and its correlating configuration files.