cmake_minimum_required(VERSION 3.5)
project(rtabmap_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
   scripts/icp_odom_cov_scale.py
   scripts/map_tf_stabilizer.py
   scripts/map_topic_stabilizer.py
   DESTINATION lib/${PROJECT_NAME}
)

ament_package()
