cmake_minimum_required(VERSION 3.8)
project(wheelloong_m2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes mujoco rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/view_m2_mujoco_arm_driver_state.py
    scripts/view_mujoco_joint_slider.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
