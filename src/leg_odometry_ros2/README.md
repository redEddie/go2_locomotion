# leg_odometry_ros2

ROS2 Foxy package for Unitree Go2 leg odometry using SDK joint states, IMU, and foot contact forces.

Quick start:
- `colcon build --base-paths /home/user/unitree_mujoco/example/go2_locomotion/src`
- `source install/setup.bash`
- `ros2 run leg_odometry_ros2 leg_odometry_node --ros-args --params-file /home/user/unitree_mujoco/example/go2_locomotion/leg_odometry_ros2/config/leg_odometry.yaml`
