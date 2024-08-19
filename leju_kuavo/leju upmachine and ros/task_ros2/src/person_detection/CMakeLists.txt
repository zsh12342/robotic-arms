cmake_minimum_required(VERSION 3.0.2)
project(person_detection)

## ÕÒµ½ catkin ºÍÆäËûÒÀÀµµÄ°ü
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp
)

## catkin_package ºêÉú³É cmake ÅäÖÃÎÄ¼þ
## ÎªÒÀÀµÏîÄ¿´«µÝµÄÄÚÈÝ
catkin_package(
  CATKIN_DEPENDS rospy std_msgs roscpp
)

## °üº¬Ä¿Â¼
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## °²×° Python ½Å±¾
catkin_install_python(PROGRAMS 
scripts/person_detection.py
scripts/bottle_catch.py
scripts/T1.py
scripts/T5.py
scripts/T6.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
