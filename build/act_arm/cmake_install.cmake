# Install script for directory: /home/easy/yin/robot_arm/opensource_prj/act_plus_ros/src/act_arm

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/act_arm.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/act_arm/cmake" TYPE FILE FILES
    "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/act_armConfig.cmake"
    "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/act_armConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/act_arm" TYPE FILE FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/src/act_arm/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/record_episodes.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/constants.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/dynamixel_client.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/example_waypoint_pid.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/get_episode_len.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/one_side_teleop.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/real_env.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/realsense_test.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/record_episodes.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/replay_and_record_episodes.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/replay_episodes.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/robot_utils.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/sleep.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/speed_test.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/visualize_episodes.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/act_arm" TYPE PROGRAM FILES "/home/easy/yin/robot_arm/opensource_prj/act_plus_ros/build/act_arm/catkin_generated/installspace/waypoint_control.py")
endif()

