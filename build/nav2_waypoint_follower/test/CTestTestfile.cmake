# CMake generated Testfile for 
# Source directory: /home/user/ros2_ws/src/navigation2/nav2_waypoint_follower/test
# Build directory: /home/user/ros2_ws/build/nav2_waypoint_follower/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_task_executors "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/user/ros2_ws/build/nav2_waypoint_follower/test_results/nav2_waypoint_follower/test_task_executors.gtest.xml" "--package-name" "nav2_waypoint_follower" "--output-file" "/home/user/ros2_ws/build/nav2_waypoint_follower/ament_cmake_gtest/test_task_executors.txt" "--command" "/home/user/ros2_ws/build/nav2_waypoint_follower/test/test_task_executors" "--gtest_output=xml:/home/user/ros2_ws/build/nav2_waypoint_follower/test_results/nav2_waypoint_follower/test_task_executors.gtest.xml")
set_tests_properties(test_task_executors PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/user/ros2_ws/build/nav2_waypoint_follower/test/test_task_executors" TIMEOUT "60" WORKING_DIRECTORY "/home/user/ros2_ws/build/nav2_waypoint_follower/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/user/ros2_ws/src/navigation2/nav2_waypoint_follower/test/CMakeLists.txt;2;ament_add_gtest;/home/user/ros2_ws/src/navigation2/nav2_waypoint_follower/test/CMakeLists.txt;0;")
add_test(test_dynamic_parameters "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/user/ros2_ws/build/nav2_waypoint_follower/test_results/nav2_waypoint_follower/test_dynamic_parameters.gtest.xml" "--package-name" "nav2_waypoint_follower" "--output-file" "/home/user/ros2_ws/build/nav2_waypoint_follower/ament_cmake_gtest/test_dynamic_parameters.txt" "--command" "/home/user/ros2_ws/build/nav2_waypoint_follower/test/test_dynamic_parameters" "--gtest_output=xml:/home/user/ros2_ws/build/nav2_waypoint_follower/test_results/nav2_waypoint_follower/test_dynamic_parameters.gtest.xml")
set_tests_properties(test_dynamic_parameters PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/user/ros2_ws/build/nav2_waypoint_follower/test/test_dynamic_parameters" TIMEOUT "60" WORKING_DIRECTORY "/home/user/ros2_ws/build/nav2_waypoint_follower/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/user/ros2_ws/src/navigation2/nav2_waypoint_follower/test/CMakeLists.txt;13;ament_add_gtest;/home/user/ros2_ws/src/navigation2/nav2_waypoint_follower/test/CMakeLists.txt;0;")
subdirs("../gtest")
