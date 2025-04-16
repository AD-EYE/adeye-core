# Install script for directory: /home/adeye/adeye-core/AD-EYE/ROS_Packages/src/AD-EYE_utils

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/adeye/adeye-core/install/adeye_utils")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/adeye/adeye-core/build/adeye_utils/catkin_generated/installspace/adeye_utils.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/adeye_utils/cmake" TYPE FILE FILES
    "/home/adeye/adeye-core/build/adeye_utils/catkin_generated/installspace/adeye_utilsConfig.cmake"
    "/home/adeye/adeye-core/build/adeye_utils/catkin_generated/installspace/adeye_utilsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/adeye_utils" TYPE FILE FILES "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/AD-EYE_utils/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/trajectoryVisualizer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/trajectoryVisualizer")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/trajectoryVisualizer"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/adeye_utils" TYPE EXECUTABLE FILES "/home/adeye/adeye-core/build/adeye_utils/devel/lib/adeye_utils/trajectoryVisualizer")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/trajectoryVisualizer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/trajectoryVisualizer")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/trajectoryVisualizer"
         OLD_RPATH "/home/adeye/AD-EYE_Core/autoware.ai/install/op_ros_helpers/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_simu/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_planner/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_utility/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/vector_map/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/waypoint_follower/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/amathutils_lib/lib:/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/autoware_health_checker/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/trajectoryVisualizer")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudFrameChanger" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudFrameChanger")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudFrameChanger"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/adeye_utils" TYPE EXECUTABLE FILES "/home/adeye/adeye-core/build/adeye_utils/devel/lib/adeye_utils/pointCloudFrameChanger")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudFrameChanger" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudFrameChanger")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudFrameChanger"
         OLD_RPATH "/home/adeye/AD-EYE_Core/autoware.ai/install/op_ros_helpers/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_simu/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_planner/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_utility/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/vector_map/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/waypoint_follower/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/amathutils_lib/lib:/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/autoware_health_checker/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudFrameChanger")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudToOccupancyGrid" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudToOccupancyGrid")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudToOccupancyGrid"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/adeye_utils" TYPE EXECUTABLE FILES "/home/adeye/adeye-core/build/adeye_utils/devel/lib/adeye_utils/pointCloudToOccupancyGrid")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudToOccupancyGrid" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudToOccupancyGrid")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudToOccupancyGrid"
         OLD_RPATH "/home/adeye/AD-EYE_Core/autoware.ai/install/op_ros_helpers/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_simu/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_planner/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_utility/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/vector_map/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/waypoint_follower/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/amathutils_lib/lib:/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/autoware_health_checker/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/adeye_utils/pointCloudToOccupancyGrid")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libadeye_utils.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libadeye_utils.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libadeye_utils.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/adeye/adeye-core/build/adeye_utils/devel/lib/libadeye_utils.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libadeye_utils.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libadeye_utils.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libadeye_utils.so"
         OLD_RPATH "/home/adeye/AD-EYE_Core/autoware.ai/install/op_ros_helpers/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_simu/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_planner/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/op_utility/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/vector_map/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/waypoint_follower/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/amathutils_lib/lib:/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/home/adeye/AD-EYE_Core/autoware.ai/install/autoware_health_checker/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libadeye_utils.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/adeye_utils" TYPE DIRECTORY FILES "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/AD-EYE_utils/include/adeye_utils/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/adeye/adeye-core/build/adeye_utils/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/adeye/adeye-core/build/adeye_utils/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
