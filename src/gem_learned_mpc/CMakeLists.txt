cmake_minimum_required(VERSION 3.0.2)
project(gem_learned_mpc)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf
  gazebo_msgs
  ackermann_msgs
  message_generation
)

# IMPORTANT:
# catkin_python_setup() must be called before add_message_files() and
# generate_messages(). This order avoids the Catkin configuration error:
# "generate_messages() must be called after catkin_python_setup()".
catkin_python_setup()

add_message_files(
  FILES
    VehicleState.msg
    ControlCommand.msg
    MPCStatus.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    tf
    gazebo_msgs
    ackermann_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
    scripts/state_adapter.py
    scripts/excitation_driver.py
    scripts/command_adapter.py
    scripts/waypoint_server.py
    scripts/learned_mpc_node.py
    scripts/cte_monitor.py
    scripts/acceptance_logger.py
    scripts/generate_acceptance_report.py
    scripts/rviz_validation_markers.py
    scripts/set_scenario_initial_pose.py
    scripts/scenario_speed_override.py
    scripts/fault_injector.py
    scripts/run_scenario.py
    scripts/summarize_scenarios.py
    scripts/bag_to_npz.py
    scripts/prepare_dataset.py
    scripts/train_dynamics.py
    scripts/validate_one_step.py
    scripts/validate_rollout.py
    scripts/export_model_json.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
    waypoints
    rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
