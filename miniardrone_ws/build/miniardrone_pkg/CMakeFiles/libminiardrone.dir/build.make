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

# Include any dependencies generated for this target.
include miniardrone_pkg/CMakeFiles/libminiardrone.dir/depend.make

# Include the progress variables for this target.
include miniardrone_pkg/CMakeFiles/libminiardrone.dir/progress.make

# Include the compile flags for this target's objects.
include miniardrone_pkg/CMakeFiles/libminiardrone.dir/flags.make

miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o: miniardrone_pkg/CMakeFiles/libminiardrone.dir/flags.make
miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o: /home/dima/ROS_Projects/miniardrone_ws/src/miniardrone_pkg/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dima/ROS_Projects/miniardrone_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o"
	cd /home/dima/ROS_Projects/miniardrone_ws/build/miniardrone_pkg && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libminiardrone.dir/src/main.cpp.o -c /home/dima/ROS_Projects/miniardrone_ws/src/miniardrone_pkg/src/main.cpp

miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libminiardrone.dir/src/main.cpp.i"
	cd /home/dima/ROS_Projects/miniardrone_ws/build/miniardrone_pkg && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dima/ROS_Projects/miniardrone_ws/src/miniardrone_pkg/src/main.cpp > CMakeFiles/libminiardrone.dir/src/main.cpp.i

miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libminiardrone.dir/src/main.cpp.s"
	cd /home/dima/ROS_Projects/miniardrone_ws/build/miniardrone_pkg && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dima/ROS_Projects/miniardrone_ws/src/miniardrone_pkg/src/main.cpp -o CMakeFiles/libminiardrone.dir/src/main.cpp.s

miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o.requires:

.PHONY : miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o.requires

miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o.provides: miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o.requires
	$(MAKE) -f miniardrone_pkg/CMakeFiles/libminiardrone.dir/build.make miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o.provides.build
.PHONY : miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o.provides

miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o.provides.build: miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o


# Object files for target libminiardrone
libminiardrone_OBJECTS = \
"CMakeFiles/libminiardrone.dir/src/main.cpp.o"

# External object files for target libminiardrone
libminiardrone_EXTERNAL_OBJECTS =

/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: miniardrone_pkg/CMakeFiles/libminiardrone.dir/build.make
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /opt/ros/kinetic/lib/libroscpp.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /opt/ros/kinetic/lib/librosconsole.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /opt/ros/kinetic/lib/librostime.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /opt/ros/kinetic/lib/libcpp_common.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so: miniardrone_pkg/CMakeFiles/libminiardrone.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dima/ROS_Projects/miniardrone_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so"
	cd /home/dima/ROS_Projects/miniardrone_ws/build/miniardrone_pkg && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/libminiardrone.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
miniardrone_pkg/CMakeFiles/libminiardrone.dir/build: /home/dima/ROS_Projects/miniardrone_ws/devel/lib/liblibminiardrone.so

.PHONY : miniardrone_pkg/CMakeFiles/libminiardrone.dir/build

miniardrone_pkg/CMakeFiles/libminiardrone.dir/requires: miniardrone_pkg/CMakeFiles/libminiardrone.dir/src/main.cpp.o.requires

.PHONY : miniardrone_pkg/CMakeFiles/libminiardrone.dir/requires

miniardrone_pkg/CMakeFiles/libminiardrone.dir/clean:
	cd /home/dima/ROS_Projects/miniardrone_ws/build/miniardrone_pkg && $(CMAKE_COMMAND) -P CMakeFiles/libminiardrone.dir/cmake_clean.cmake
.PHONY : miniardrone_pkg/CMakeFiles/libminiardrone.dir/clean

miniardrone_pkg/CMakeFiles/libminiardrone.dir/depend:
	cd /home/dima/ROS_Projects/miniardrone_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dima/ROS_Projects/miniardrone_ws/src /home/dima/ROS_Projects/miniardrone_ws/src/miniardrone_pkg /home/dima/ROS_Projects/miniardrone_ws/build /home/dima/ROS_Projects/miniardrone_ws/build/miniardrone_pkg /home/dima/ROS_Projects/miniardrone_ws/build/miniardrone_pkg/CMakeFiles/libminiardrone.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : miniardrone_pkg/CMakeFiles/libminiardrone.dir/depend

