# Copyright (c) 2019 Pilz GmbH & Co. KG

cmake_minimum_required(VERSION 2.8.3)
project(psen_scan_tutorials)

find_package(catkin REQUIRED COMPONENTS
  psen_scan
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
