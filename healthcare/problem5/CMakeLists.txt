cmake_minimum_required(VERSION 3.5)
project(problem5)

find_package(ament_cmake REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(deliver_item_action_node src/deliver_item_action_node.cpp)
ament_target_dependencies(deliver_item_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
)

add_executable(deploy_box_action_node src/deploy_box_action_node.cpp)
ament_target_dependencies(deploy_box_action_node
  plansys2_executor
  rclcpp
)

add_executable(deploy_container_action_node src/deploy_container_action_node.cpp)
ament_target_dependencies(deploy_container_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
)

add_executable(drop_off_patient_action_node src/drop_off_patient_action_node.cpp)
ament_target_dependencies(drop_off_patient_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
)

add_executable(get_patient_action_node src/get_patient_action_node.cpp)
ament_target_dependencies(get_patient_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
)

add_executable(insert_box_action_node src/insert_box_action_node.cpp)
ament_target_dependencies(insert_box_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
)

add_executable(insert_item_action_node src/insert_item_action_node.cpp)
ament_target_dependencies(insert_item_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
)

add_executable(move_busy_carrier_action_node src/move_busy_carrier_action_node.cpp)
ament_target_dependencies(move_busy_carrier_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
)

add_executable(move_escorting_accompanier_action_node src/move_escorting_accompanier_action_node.cpp)
ament_target_dependencies(move_escorting_accompanier_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
)

add_executable(move_free_accompanier_action_node src/move_free_accompanier_action_node.cpp)
ament_target_dependencies(move_free_accompanier_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
)

add_executable(pickup_container_action_node src/pickup_container_action_node.cpp)
ament_target_dependencies(pickup_container_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
)

install(DIRECTORY launch pddl
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  deliver_item_action_node
  deploy_box_action_node
  deploy_container_action_node
  drop_off_patient_action_node
  get_patient_action_node
  insert_box_action_node
  insert_item_action_node
  move_busy_carrier_action_node
  move_escorting_accompanier_action_node
  move_free_accompanier_action_node
  pickup_container_action_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(
  plansys2_executor
  rclcpp
  rclcpp_action
)

ament_package()