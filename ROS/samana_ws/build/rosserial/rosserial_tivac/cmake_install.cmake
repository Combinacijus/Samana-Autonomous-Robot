# Install script for directory: /home/combinacijus/Documents/ROS/samana_ws/src/rosserial/rosserial_tivac

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/combinacijus/Documents/ROS/samana_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/combinacijus/Documents/ROS/samana_ws/build/rosserial/rosserial_tivac/catkin_generated/installspace/rosserial_tivac.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac/cmake" TYPE FILE FILES "/home/combinacijus/Documents/ROS/samana_ws/build/rosserial/rosserial_tivac/catkin_generated/installspace/rosserial_tivac-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac/cmake" TYPE FILE FILES
    "/home/combinacijus/Documents/ROS/samana_ws/build/rosserial/rosserial_tivac/catkin_generated/installspace/rosserial_tivacConfig.cmake"
    "/home/combinacijus/Documents/ROS/samana_ws/build/rosserial/rosserial_tivac/catkin_generated/installspace/rosserial_tivacConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac" TYPE FILE FILES "/home/combinacijus/Documents/ROS/samana_ws/src/rosserial/rosserial_tivac/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac/src" TYPE DIRECTORY FILES
    "/home/combinacijus/Documents/ROS/samana_ws/src/rosserial/rosserial_tivac/src/ros_lib"
    "/home/combinacijus/Documents/ROS/samana_ws/src/rosserial/rosserial_tivac/src/ros_lib_energia"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac" TYPE DIRECTORY FILES "/home/combinacijus/Documents/ROS/samana_ws/src/rosserial/rosserial_tivac/tivac-cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosserial_tivac" TYPE PROGRAM FILES
    "/home/combinacijus/Documents/ROS/samana_ws/src/rosserial/rosserial_tivac/src/rosserial_tivac/make_libraries_energia"
    "/home/combinacijus/Documents/ROS/samana_ws/src/rosserial/rosserial_tivac/src/rosserial_tivac/make_libraries_tiva"
    )
endif()

