# Install script for directory: /home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/samana_msgs/msg" TYPE FILE FILES
    "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg"
    "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Sonar.msg"
    "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg"
    "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg"
    "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg"
    "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/samana_msgs/cmake" TYPE FILE FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/build/samana_msgs/catkin_generated/installspace/samana_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/include/samana_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/share/roseus/ros/samana_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/share/common-lisp/ros/samana_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/share/gennodejs/ros/samana_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/lib/python2.7/dist-packages/samana_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/lib/python2.7/dist-packages/samana_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/build/samana_msgs/catkin_generated/installspace/samana_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/samana_msgs/cmake" TYPE FILE FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/build/samana_msgs/catkin_generated/installspace/samana_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/samana_msgs/cmake" TYPE FILE FILES
    "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/build/samana_msgs/catkin_generated/installspace/samana_msgsConfig.cmake"
    "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/build/samana_msgs/catkin_generated/installspace/samana_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/samana_msgs" TYPE FILE FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/package.xml")
endif()

