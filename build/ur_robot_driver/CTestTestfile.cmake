# CMake generated Testfile for 
# Source directory: /home/jcab/ros_ws/src/universal_robots_ros_driver/ur_robot_driver
# Build directory: /home/jcab/ros_ws/build/ur_robot_driver
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_ur_robot_driver_rostest_test_driver.test "/home/jcab/ros_ws/build/ur_robot_driver/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/jcab/ros_ws/build/ur_robot_driver/test_results/ur_robot_driver/rostest-test_driver.xml" "--return-code" "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/jcab/ros_ws/src/universal_robots_ros_driver/ur_robot_driver --package=ur_robot_driver --results-filename test_driver.xml --results-base-dir \"/home/jcab/ros_ws/build/ur_robot_driver/test_results\" /home/jcab/ros_ws/src/universal_robots_ros_driver/ur_robot_driver/test/driver.test ")
subdirs("gtest")
