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
include CMakeFiles/lesson2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lesson2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lesson2.dir/flags.make

CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o: CMakeFiles/lesson2.dir/flags.make
CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o: ../ode/lesson/lesson2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o -c /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode/lesson/lesson2.cpp

CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode/lesson/lesson2.cpp > CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.i

CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode/lesson/lesson2.cpp -o CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.s

CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o.requires:

.PHONY : CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o.requires

CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o.provides: CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o.requires
	$(MAKE) -f CMakeFiles/lesson2.dir/build.make CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o.provides.build
.PHONY : CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o.provides

CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o.provides.build: CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o


# Object files for target lesson2
lesson2_OBJECTS = \
"CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o"

# External object files for target lesson2
lesson2_EXTERNAL_OBJECTS =

lesson2: CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o
lesson2: CMakeFiles/lesson2.dir/build.make
lesson2: libdrawstuff.so
lesson2: libode.so.0.16.0
lesson2: /usr/lib/x86_64-linux-gnu/libGL.so
lesson2: /usr/lib/x86_64-linux-gnu/libGLU.so
lesson2: /usr/lib/x86_64-linux-gnu/libSM.so
lesson2: /usr/lib/x86_64-linux-gnu/libICE.so
lesson2: /usr/lib/x86_64-linux-gnu/libX11.so
lesson2: /usr/lib/x86_64-linux-gnu/libXext.so
lesson2: CMakeFiles/lesson2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable lesson2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lesson2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lesson2.dir/build: lesson2

.PHONY : CMakeFiles/lesson2.dir/build

CMakeFiles/lesson2.dir/requires: CMakeFiles/lesson2.dir/ode/lesson/lesson2.cpp.o.requires

.PHONY : CMakeFiles/lesson2.dir/requires

CMakeFiles/lesson2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lesson2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lesson2.dir/clean

CMakeFiles/lesson2.dir/depend:
	cd /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dieu_detruit/3A/simulation/ode/ode-0.16 /home/dieu_detruit/3A/simulation/ode/ode-0.16 /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build /home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles/lesson2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lesson2.dir/depend

