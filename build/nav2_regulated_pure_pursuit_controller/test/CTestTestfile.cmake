# CMake generated Testfile for 
# Source directory: /home/user/ros2_ws/src/navigation2/nav2_regulated_pure_pursuit_controller/test
# Build directory: /home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_regulated_pp "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/test_results/nav2_regulated_pure_pursuit_controller/test_regulated_pp.gtest.xml" "--package-name" "nav2_regulated_pure_pursuit_controller" "--output-file" "/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/ament_cmake_gtest/test_regulated_pp.txt" "--command" "/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/test/test_regulated_pp" "--gtest_output=xml:/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/test_results/nav2_regulated_pure_pursuit_controller/test_regulated_pp.gtest.xml")
set_tests_properties(test_regulated_pp PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/test/test_regulated_pp" TIMEOUT "60" WORKING_DIRECTORY "/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/user/ros2_ws/src/navigation2/nav2_regulated_pure_pursuit_controller/test/CMakeLists.txt;2;ament_add_gtest;/home/user/ros2_ws/src/navigation2/nav2_regulated_pure_pursuit_controller/test/CMakeLists.txt;0;")
add_test(test_path_utils "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/test_results/nav2_regulated_pure_pursuit_controller/test_path_utils.gtest.xml" "--package-name" "nav2_regulated_pure_pursuit_controller" "--output-file" "/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/ament_cmake_gtest/test_path_utils.txt" "--command" "/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/test/test_path_utils" "--gtest_output=xml:/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/test_results/nav2_regulated_pure_pursuit_controller/test_path_utils.gtest.xml")
set_tests_properties(test_path_utils PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/test/test_path_utils" TIMEOUT "60" WORKING_DIRECTORY "/home/user/ros2_ws/build/nav2_regulated_pure_pursuit_controller/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/user/ros2_ws/src/navigation2/nav2_regulated_pure_pursuit_controller/test/CMakeLists.txt;14;ament_add_gtest;/home/user/ros2_ws/src/navigation2/nav2_regulated_pure_pursuit_controller/test/CMakeLists.txt;0;")
subdirs("../gtest")
