# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/wg/Documents/can_message_decode/can_message_decode

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wg/Documents/can_message_decode/build

# Include any dependencies generated for this target.
include CMakeFiles/run_decode_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/run_decode_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/run_decode_test.dir/flags.make

CMakeFiles/run_decode_test.dir/decode_test.cpp.o: CMakeFiles/run_decode_test.dir/flags.make
CMakeFiles/run_decode_test.dir/decode_test.cpp.o: /home/wg/Documents/can_message_decode/can_message_decode/decode_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wg/Documents/can_message_decode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/run_decode_test.dir/decode_test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_decode_test.dir/decode_test.cpp.o -c /home/wg/Documents/can_message_decode/can_message_decode/decode_test.cpp

CMakeFiles/run_decode_test.dir/decode_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_decode_test.dir/decode_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wg/Documents/can_message_decode/can_message_decode/decode_test.cpp > CMakeFiles/run_decode_test.dir/decode_test.cpp.i

CMakeFiles/run_decode_test.dir/decode_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_decode_test.dir/decode_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wg/Documents/can_message_decode/can_message_decode/decode_test.cpp -o CMakeFiles/run_decode_test.dir/decode_test.cpp.s

# Object files for target run_decode_test
run_decode_test_OBJECTS = \
"CMakeFiles/run_decode_test.dir/decode_test.cpp.o"

# External object files for target run_decode_test
run_decode_test_EXTERNAL_OBJECTS =

run_decode_test: CMakeFiles/run_decode_test.dir/decode_test.cpp.o
run_decode_test: CMakeFiles/run_decode_test.dir/build.make
run_decode_test: CMakeFiles/run_decode_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wg/Documents/can_message_decode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable run_decode_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_decode_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/run_decode_test.dir/build: run_decode_test

.PHONY : CMakeFiles/run_decode_test.dir/build

CMakeFiles/run_decode_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_decode_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_decode_test.dir/clean

CMakeFiles/run_decode_test.dir/depend:
	cd /home/wg/Documents/can_message_decode/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wg/Documents/can_message_decode/can_message_decode /home/wg/Documents/can_message_decode/can_message_decode /home/wg/Documents/can_message_decode/build /home/wg/Documents/can_message_decode/build /home/wg/Documents/can_message_decode/build/CMakeFiles/run_decode_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_decode_test.dir/depend

