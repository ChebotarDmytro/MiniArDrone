# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dima/ROS_Projects/miniardrone_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dima/ROS_Projects/miniardrone_ws/build

# Utility rule file for roslint_miniardrone_pkg.

# Include the progress variables for this target.
include miniardrone_pkg/CMakeFiles/roslint_miniardrone_pkg.dir/progress.make

roslint_miniardrone_pkg: miniardrone_pkg/CMakeFiles/roslint_miniardrone_pkg.dir/build.make
	cd /home/dima/ROS_Projects/miniardrone_ws/src/miniardrone_pkg && /opt/ros/kinetic/share/roslint/cmake/../../../lib/roslint/cpplint --filter=-build/include src/main.cpp src/ardrone_driver.cpp include/miniardrone_pkg/ardrone_driver.h
	cd /home/dima/ROS_Projects/miniardrone_ws/src/miniardrone_pkg && /opt/ros/kinetic/share/roslint/cmake/../../../lib/roslint/cpplint --filter=-runtime/references-runtime/threadsafe_fn include/miniardrone_pkg/ardrone_driver.h src/ardrone_driver.cpp src/main.cpp
.PHONY : roslint_miniardrone_pkg

# Rule to build all files generated by this target.
miniardrone_pkg/CMakeFiles/roslint_miniardrone_pkg.dir/build: roslint_miniardrone_pkg

.PHONY : miniardrone_pkg/CMakeFiles/roslint_miniardrone_pkg.dir/build

miniardrone_pkg/CMakeFiles/roslint_miniardrone_pkg.dir/clean:
	cd /home/dima/ROS_Projects/miniardrone_ws/build/miniardrone_pkg && $(CMAKE_COMMAND) -P CMakeFiles/roslint_miniardrone_pkg.dir/cmake_clean.cmake
.PHONY : miniardrone_pkg/CMakeFiles/roslint_miniardrone_pkg.dir/clean

miniardrone_pkg/CMakeFiles/roslint_miniardrone_pkg.dir/depend:
	cd /home/dima/ROS_Projects/miniardrone_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dima/ROS_Projects/miniardrone_ws/src /home/dima/ROS_Projects/miniardrone_ws/src/miniardrone_pkg /home/dima/ROS_Projects/miniardrone_ws/build /home/dima/ROS_Projects/miniardrone_ws/build/miniardrone_pkg /home/dima/ROS_Projects/miniardrone_ws/build/miniardrone_pkg/CMakeFiles/roslint_miniardrone_pkg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : miniardrone_pkg/CMakeFiles/roslint_miniardrone_pkg.dir/depend

