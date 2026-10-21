# CMake generated Testfile for 
# Source directory: /home/kendas/桌面/zimiao/sp_vision_25/tests/tuning
# Build directory: /home/kendas/桌面/zimiao/sp_vision_25/build-tuning
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(planner_rollout "/home/kendas/桌面/zimiao/sp_vision_25/build-tuning/planner_rollout_test" "/home/kendas/桌面/zimiao/sp_vision_ros/artifacts/planner-rollout.json")
set_tests_properties(planner_rollout PROPERTIES  _BACKTRACE_TRIPLES "/home/kendas/桌面/zimiao/sp_vision_25/tests/tuning/CMakeLists.txt;27;add_test;/home/kendas/桌面/zimiao/sp_vision_25/tests/tuning/CMakeLists.txt;0;")
add_test(tracker_intervals "/home/kendas/桌面/zimiao/sp_vision_25/build-tuning/tracker_interval_test" "/home/kendas/桌面/zimiao/sp_vision_ros/config/simulator.yaml")
set_tests_properties(tracker_intervals PROPERTIES  _BACKTRACE_TRIPLES "/home/kendas/桌面/zimiao/sp_vision_25/tests/tuning/CMakeLists.txt;28;add_test;/home/kendas/桌面/zimiao/sp_vision_25/tests/tuning/CMakeLists.txt;0;")
add_test(fire_window "/home/kendas/桌面/zimiao/sp_vision_25/build-tuning/fire_window_test" "/home/kendas/桌面/zimiao/sp_vision_ros/config/simulator.yaml")
set_tests_properties(fire_window PROPERTIES  _BACKTRACE_TRIPLES "/home/kendas/桌面/zimiao/sp_vision_25/tests/tuning/CMakeLists.txt;33;add_test;/home/kendas/桌面/zimiao/sp_vision_25/tests/tuning/CMakeLists.txt;0;")
add_test(planner_clock "/home/kendas/桌面/zimiao/sp_vision_25/build-tuning/planner_clock_test" "/home/kendas/桌面/zimiao/sp_vision_ros/configs/standard3.yaml")
set_tests_properties(planner_clock PROPERTIES  _BACKTRACE_TRIPLES "/home/kendas/桌面/zimiao/sp_vision_25/tests/tuning/CMakeLists.txt;44;add_test;/home/kendas/桌面/zimiao/sp_vision_25/tests/tuning/CMakeLists.txt;0;")
