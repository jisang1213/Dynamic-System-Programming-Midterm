# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jisang/midterm/midterm_2024_lib

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jisang/midterm/midterm_2024_lib/build

# Include any dependencies generated for this target.
include CMakeFiles/myQuaternion_20190837.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/myQuaternion_20190837.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/myQuaternion_20190837.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/myQuaternion_20190837.dir/flags.make

CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.o: CMakeFiles/myQuaternion_20190837.dir/flags.make
CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.o: ../src/myQuaternion_20190837.cpp
CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.o: CMakeFiles/myQuaternion_20190837.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jisang/midterm/midterm_2024_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.o -MF CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.o.d -o CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.o -c /home/jisang/midterm/midterm_2024_lib/src/myQuaternion_20190837.cpp

CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jisang/midterm/midterm_2024_lib/src/myQuaternion_20190837.cpp > CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.i

CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jisang/midterm/midterm_2024_lib/src/myQuaternion_20190837.cpp -o CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.s

# Object files for target myQuaternion_20190837
myQuaternion_20190837_OBJECTS = \
"CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.o"

# External object files for target myQuaternion_20190837
myQuaternion_20190837_EXTERNAL_OBJECTS =

libmyQuaternion_20190837.so: CMakeFiles/myQuaternion_20190837.dir/src/myQuaternion_20190837.cpp.o
libmyQuaternion_20190837.so: CMakeFiles/myQuaternion_20190837.dir/build.make
libmyQuaternion_20190837.so: CMakeFiles/myQuaternion_20190837.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jisang/midterm/midterm_2024_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libmyQuaternion_20190837.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myQuaternion_20190837.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/myQuaternion_20190837.dir/build: libmyQuaternion_20190837.so
.PHONY : CMakeFiles/myQuaternion_20190837.dir/build

CMakeFiles/myQuaternion_20190837.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/myQuaternion_20190837.dir/cmake_clean.cmake
.PHONY : CMakeFiles/myQuaternion_20190837.dir/clean

CMakeFiles/myQuaternion_20190837.dir/depend:
	cd /home/jisang/midterm/midterm_2024_lib/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jisang/midterm/midterm_2024_lib /home/jisang/midterm/midterm_2024_lib /home/jisang/midterm/midterm_2024_lib/build /home/jisang/midterm/midterm_2024_lib/build /home/jisang/midterm/midterm_2024_lib/build/CMakeFiles/myQuaternion_20190837.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/myQuaternion_20190837.dir/depend

