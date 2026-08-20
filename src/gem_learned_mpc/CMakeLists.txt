cmake_minimum_required(VERSION 3.0.2)
project(gem_learned_mpc)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs nav_msgs sensor_msgs gazebo_msgs ackermann_msgs tf message_generation)
add_message_files(FILES VehicleState.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs gazebo_msgs ackermann_msgs tf message_runtime)
include_directories(
${catkin_INCLUDE_DIRS}
)
catkin_install_python(
	PROGRAMS 
		scripts/state_adapter.py 
		scripts/excitation_node.py 
		scripts/bag_to_dataset.py 
		scripts/preprocess_dataset.py 
		scripts/train_dynamics.py 
		scripts/validate_dynamics.py 
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
