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
include CMakeFiles/demo_space.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/demo_space.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/demo_space.dir/flags.make

CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o: CMakeFiles/demo_space.dir/flags.make
CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o: ../ode/demo/demo_space.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o -c /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode/demo/demo_space.cpp

CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode/demo/demo_space.cpp > CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.i

CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode/demo/demo_space.cpp -o CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.s

CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o.requires:

.PHONY : CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o.requires

CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o.provides: CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o.requires
	$(MAKE) -f CMakeFiles/demo_space.dir/build.make CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o.provides.build
.PHONY : CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o.provides

CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o.provides.build: CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o


# Object files for target demo_space
demo_space_OBJECTS = \
"CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o"

# External object files for target demo_space
demo_space_EXTERNAL_OBJECTS =

demo_space: CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o
demo_space: CMakeFiles/demo_space.dir/build.make
demo_space: libdrawstuff.so
demo_space: libode.so.0.16.0
demo_space: /usr/lib/x86_64-linux-gnu/libGL.so
demo_space: /usr/lib/x86_64-linux-gnu/libGLU.so
demo_space: /usr/lib/x86_64-linux-gnu/libSM.so
demo_space: /usr/lib/x86_64-linux-gnu/libICE.so
demo_space: /usr/lib/x86_64-linux-gnu/libX11.so
demo_space: /usr/lib/x86_64-linux-gnu/libXext.so
demo_space: CMakeFiles/demo_space.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable demo_space"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo_space.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/demo_space.dir/build: demo_space

.PHONY : CMakeFiles/demo_space.dir/build

CMakeFiles/demo_space.dir/requires: CMakeFiles/demo_space.dir/ode/demo/demo_space.cpp.o.requires

.PHONY : CMakeFiles/demo_space.dir/requires

CMakeFiles/demo_space.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/demo_space.dir/cmake_clean.cmake
.PHONY : CMakeFiles/demo_space.dir/clean

CMakeFiles/demo_space.dir/depend:
	cd /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dieu_detruit/3A/simulation/ode/ode-0.16 /home/dieu_detruit/3A/simulation/ode/ode-0.16 /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles/demo_space.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/demo_space.dir/depend
