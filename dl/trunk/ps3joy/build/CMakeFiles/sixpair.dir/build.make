# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy/build

# Include any dependencies generated for this target.
include CMakeFiles/sixpair.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sixpair.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sixpair.dir/flags.make

CMakeFiles/sixpair.dir/sixpair.o: CMakeFiles/sixpair.dir/flags.make
CMakeFiles/sixpair.dir/sixpair.o: ../sixpair.c
CMakeFiles/sixpair.dir/sixpair.o: ../manifest.xml
CMakeFiles/sixpair.dir/sixpair.o: /opt/ros/fuerte/share/diagnostic_msgs/manifest.xml
CMakeFiles/sixpair.dir/sixpair.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/sixpair.dir/sixpair.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/sixpair.dir/sixpair.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/sixpair.dir/sixpair.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/sixpair.dir/sixpair.o: /opt/ros/fuerte/share/rosgraph/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/sixpair.dir/sixpair.o"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/sixpair.dir/sixpair.o   -c /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy/sixpair.c

CMakeFiles/sixpair.dir/sixpair.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sixpair.dir/sixpair.i"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy/sixpair.c > CMakeFiles/sixpair.dir/sixpair.i

CMakeFiles/sixpair.dir/sixpair.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sixpair.dir/sixpair.s"
	/usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy/sixpair.c -o CMakeFiles/sixpair.dir/sixpair.s

CMakeFiles/sixpair.dir/sixpair.o.requires:
.PHONY : CMakeFiles/sixpair.dir/sixpair.o.requires

CMakeFiles/sixpair.dir/sixpair.o.provides: CMakeFiles/sixpair.dir/sixpair.o.requires
	$(MAKE) -f CMakeFiles/sixpair.dir/build.make CMakeFiles/sixpair.dir/sixpair.o.provides.build
.PHONY : CMakeFiles/sixpair.dir/sixpair.o.provides

CMakeFiles/sixpair.dir/sixpair.o.provides.build: CMakeFiles/sixpair.dir/sixpair.o

# Object files for target sixpair
sixpair_OBJECTS = \
"CMakeFiles/sixpair.dir/sixpair.o"

# External object files for target sixpair
sixpair_EXTERNAL_OBJECTS =

../bin/sixpair: CMakeFiles/sixpair.dir/sixpair.o
../bin/sixpair: CMakeFiles/sixpair.dir/build.make
../bin/sixpair: CMakeFiles/sixpair.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C executable ../bin/sixpair"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sixpair.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sixpair.dir/build: ../bin/sixpair
.PHONY : CMakeFiles/sixpair.dir/build

CMakeFiles/sixpair.dir/requires: CMakeFiles/sixpair.dir/sixpair.o.requires
.PHONY : CMakeFiles/sixpair.dir/requires

CMakeFiles/sixpair.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sixpair.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sixpair.dir/clean

CMakeFiles/sixpair.dir/depend:
	cd /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy/build /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy/build /home/bren/ros_workspace/mobile_base/dl/trunk/ps3joy/build/CMakeFiles/sixpair.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sixpair.dir/depend

