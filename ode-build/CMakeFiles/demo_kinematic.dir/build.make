# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/dieu_detruit/3A/simulation/ode/ode-0.16

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build

# Include any dependencies generated for this target.
include CMakeFiles/demo_kinematic.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/demo_kinematic.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/demo_kinematic.dir/flags.make

CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o: CMakeFiles/demo_kinematic.dir/flags.make
CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o: ../ode/demo/demo_kinematic.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o -c /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode/demo/demo_kinematic.cpp

CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode/demo/demo_kinematic.cpp > CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.i

CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode/demo/demo_kinematic.cpp -o CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.s

CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o.requires:

.PHONY : CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o.requires

CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o.provides: CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o.requires
	$(MAKE) -f CMakeFiles/demo_kinematic.dir/build.make CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o.provides.build
.PHONY : CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o.provides

CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o.provides.build: CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o


# Object files for target demo_kinematic
demo_kinematic_OBJECTS = \
"CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o"

# External object files for target demo_kinematic
demo_kinematic_EXTERNAL_OBJECTS =

demo_kinematic: CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o
demo_kinematic: CMakeFiles/demo_kinematic.dir/build.make
demo_kinematic: libdrawstuff.so
demo_kinematic: libode.so.0.16.0
demo_kinematic: /usr/lib/x86_64-linux-gnu/libGL.so
demo_kinematic: /usr/lib/x86_64-linux-gnu/libGLU.so
demo_kinematic: /usr/lib/x86_64-linux-gnu/libSM.so
demo_kinematic: /usr/lib/x86_64-linux-gnu/libICE.so
demo_kinematic: /usr/lib/x86_64-linux-gnu/libX11.so
demo_kinematic: /usr/lib/x86_64-linux-gnu/libXext.so
demo_kinematic: CMakeFiles/demo_kinematic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable demo_kinematic"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo_kinematic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/demo_kinematic.dir/build: demo_kinematic

.PHONY : CMakeFiles/demo_kinematic.dir/build

CMakeFiles/demo_kinematic.dir/requires: CMakeFiles/demo_kinematic.dir/ode/demo/demo_kinematic.cpp.o.requires

.PHONY : CMakeFiles/demo_kinematic.dir/requires

CMakeFiles/demo_kinematic.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/demo_kinematic.dir/cmake_clean.cmake
.PHONY : CMakeFiles/demo_kinematic.dir/clean

CMakeFiles/demo_kinematic.dir/depend:
	cd /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dieu_detruit/3A/simulation/ode/ode-0.16 /home/dieu_detruit/3A/simulation/ode/ode-0.16 /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles/demo_kinematic.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/demo_kinematic.dir/depend

