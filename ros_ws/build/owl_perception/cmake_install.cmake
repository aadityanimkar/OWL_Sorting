<<<<<<< HEAD:ros_ws/build/owl_perception/cmake_install.cmake
# Install script for directory: /home/vardan20/OWL_Sorting/ros_ws/src/owl_perception
=======
# Install script for directory: /home/aaditya20/OWL_Sorting/ros_ws/src/moveit_pkgs/owl_moveit_bringup
>>>>>>> origin/aaditya:ros_ws/build/moveit_pkgs/owl_moveit_bringup/cmake_install.cmake

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/aaditya20/OWL_Sorting/ros_ws/install")
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
<<<<<<< HEAD:ros_ws/build/owl_perception/cmake_install.cmake
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/vardan20/OWL_Sorting/ros_ws/build/owl_perception/catkin_generated/installspace/owl_perception.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/owl_perception/cmake" TYPE FILE FILES
    "/home/vardan20/OWL_Sorting/ros_ws/build/owl_perception/catkin_generated/installspace/owl_perceptionConfig.cmake"
    "/home/vardan20/OWL_Sorting/ros_ws/build/owl_perception/catkin_generated/installspace/owl_perceptionConfig-version.cmake"
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/aaditya20/OWL_Sorting/ros_ws/build/moveit_pkgs/owl_moveit_bringup/catkin_generated/installspace/owl_moveit_bringup.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/owl_moveit_bringup/cmake" TYPE FILE FILES
    "/home/aaditya20/OWL_Sorting/ros_ws/build/moveit_pkgs/owl_moveit_bringup/catkin_generated/installspace/owl_moveit_bringupConfig.cmake"
    "/home/aaditya20/OWL_Sorting/ros_ws/build/moveit_pkgs/owl_moveit_bringup/catkin_generated/installspace/owl_moveit_bringupConfig-version.cmake"
>>>>>>> origin/aaditya:ros_ws/build/moveit_pkgs/owl_moveit_bringup/cmake_install.cmake
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD:ros_ws/build/owl_perception/cmake_install.cmake
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/owl_perception" TYPE FILE FILES "/home/vardan20/OWL_Sorting/ros_ws/src/owl_perception/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/owl_moveit_bringup" TYPE FILE FILES "/home/aaditya20/OWL_Sorting/ros_ws/src/moveit_pkgs/owl_moveit_bringup/package.xml")
>>>>>>> origin/aaditya:ros_ws/build/moveit_pkgs/owl_moveit_bringup/cmake_install.cmake
endif()

