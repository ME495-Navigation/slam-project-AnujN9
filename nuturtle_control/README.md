# Nuturtle  Control

Main package that contains all the nodes that control the turtlebot

* `ros2 launch nuturtle_control start_robot.launch.xml` to Launch the robot and the ndes to control it. Has 3 arguments.

    * `cmd_src` has 3 options ['circle', 'teleop', 'none'] which select the control source of the robot, i.e. circle node, teleop keys and none. 

    * `robot` has 3 options ['nusim', 'localhost', 'none'] which select the which robot runs, i.e. on simulation, the real robot and none. 

    * `use_rviz_3` has 2 options ['true', 'false'], which select whether or not rviz is being used.

Below is a video of the robot running counter-clockwise for 1.5 rotations, then clockwise for 1.5 rotations. After the 2 sets of circles the odom error amounted to 0.01401724m or 14mm.

https://github.com/ME495-Navigation/slam-project-AnujN9/assets/144274873/f154b650-bd6f-4e5c-a822-ff0550fac8c4