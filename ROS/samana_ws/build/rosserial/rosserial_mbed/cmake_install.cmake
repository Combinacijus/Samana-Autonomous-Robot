# Install script for directory: /home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/rosserial/rosserial_mbed

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_mbed/msg" TYPE FILE FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/rosserial/rosserial_mbed/msg/Adc.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_mbed/srv" TYPE FILE FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/rosserial/rosserial_mbed/srv/Test.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_mbed/cmake" TYPE FILE FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/build/rosserial/rosserial_mbed/catkin_generated/installspace/rosserial_mbed-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/include/rosserial_mbed")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/share/roseus/ros/rosserial_mbed")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/share/common-lisp/ros/rosserial_mbed")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/share/gennodejs/ros/rosserial_mbed")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/lib/python2.7/dist-packages/rosserial_mbed")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/devel/lib/python2.7/dist-packages/rosserial_mbed")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/build/rosserial/rosserial_mbed/catkin_generated/installspace/rosserial_mbed.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_mbed/cmake" TYPE FILE FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/build/rosserial/rosserial_mbed/catkin_generated/installspace/rosserial_mbed-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_mbed/cmake" TYPE FILE FILES
    "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/build/rosserial/rosserial_mbed/catkin_generated/installspace/rosserial_mbedConfig.cmake"
    "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/build/rosserial/rosserial_mbed/catkin_generated/installspace/rosserial_mbedConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_mbed" TYPE FILE FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/rosserial/rosserial_mbed/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_mbed/src" TYPE DIRECTORY FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/rosserial/rosserial_mbed/src/ros_lib")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rosserial_mbed" TYPE PROGRAM FILES "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/build/rosserial/rosserial_mbed/catkin_generated/installspace/make_libraries.py")
endif()

