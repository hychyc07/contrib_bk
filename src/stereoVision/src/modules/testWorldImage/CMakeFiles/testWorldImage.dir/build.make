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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/local/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /usr/local/robot/iCub/contrib/src/stereoVision/src/modules

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /usr/local/robot/iCub/contrib/src/stereoVision/src/modules

# Include any dependencies generated for this target.
include testWorldImage/CMakeFiles/testWorldImage.dir/depend.make

# Include the progress variables for this target.
include testWorldImage/CMakeFiles/testWorldImage.dir/progress.make

# Include the compile flags for this target's objects.
include testWorldImage/CMakeFiles/testWorldImage.dir/flags.make

testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o: testWorldImage/CMakeFiles/testWorldImage.dir/flags.make
testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o: testWorldImage/testWorldImage.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o"
	cd /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/testWorldImage && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o -c /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/testWorldImage/testWorldImage.cpp

testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testWorldImage.dir/testWorldImage.cpp.i"
	cd /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/testWorldImage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/testWorldImage/testWorldImage.cpp > CMakeFiles/testWorldImage.dir/testWorldImage.cpp.i

testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testWorldImage.dir/testWorldImage.cpp.s"
	cd /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/testWorldImage && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/testWorldImage/testWorldImage.cpp -o CMakeFiles/testWorldImage.dir/testWorldImage.cpp.s

testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o.requires:
.PHONY : testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o.requires

testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o.provides: testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o.requires
	$(MAKE) -f testWorldImage/CMakeFiles/testWorldImage.dir/build.make testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o.provides.build
.PHONY : testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o.provides

testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o.provides.build: testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o

# Object files for target testWorldImage
testWorldImage_OBJECTS = \
"CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o"

# External object files for target testWorldImage
testWorldImage_EXTERNAL_OBJECTS =

testWorldImage/testWorldImage: testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o
testWorldImage/testWorldImage: testWorldImage/CMakeFiles/testWorldImage.dir/build.make
testWorldImage/testWorldImage: /usr/local/robot/iCub/main/build/lib/libiKin.a
testWorldImage/testWorldImage: /usr/local/robot/iCub/main/build/lib/libicubmod.a
testWorldImage/testWorldImage: /usr/local/lib/libYARP_OS.a
testWorldImage/testWorldImage: /usr/local/lib/libYARP_sig.a
testWorldImage/testWorldImage: /usr/local/lib/libYARP_math.a
testWorldImage/testWorldImage: /usr/local/lib/libYARP_dev.a
testWorldImage/testWorldImage: /usr/local/lib/libYARP_name.a
testWorldImage/testWorldImage: /usr/local/lib/libYARP_init.a
testWorldImage/testWorldImage: /usr/local/robot/iCub/main/build/lib/libcartesiancontrollerserver.a
testWorldImage/testWorldImage: /usr/local/robot/iCub/main/build/lib/libcartesiancontrollerclient.a
testWorldImage/testWorldImage: /usr/local/robot/iCub/main/build/lib/libiKin.a
testWorldImage/testWorldImage: /usr/local/robot/iCub/main/build/lib/libctrlLib.a
testWorldImage/testWorldImage: /usr/local/robot/ipopt/lib/coin/libipopt.so
testWorldImage/testWorldImage: /usr/local/robot/ipopt/lib/coin/ThirdParty/libcoinhsl.so
testWorldImage/testWorldImage: /usr/local/robot/iCub/main/build/lib/libgazecontrollerclient.a
testWorldImage/testWorldImage: /usr/local/robot/iCub/main/build/lib/libTalkingHeadcalibrator.a
testWorldImage/testWorldImage: /usr/local/robot/iCub/main/build/lib/libdebugInterfaceClient.a
testWorldImage/testWorldImage: /usr/local/robot/iCub/main/build/lib/libcontrolboardwrapper2.a
testWorldImage/testWorldImage: /usr/local/lib/libYARP_init.a
testWorldImage/testWorldImage: /usr/local/lib/libYARP_math.a
testWorldImage/testWorldImage: /usr/lib/libgsl.so
testWorldImage/testWorldImage: /usr/lib/libgslcblas.so
testWorldImage/testWorldImage: /usr/local/lib/libYARP_dev.a
testWorldImage/testWorldImage: /usr/local/lib/libYARP_sig.a
testWorldImage/testWorldImage: /usr/local/lib/libYARP_name.a
testWorldImage/testWorldImage: /usr/local/lib/libYARP_OS.a
testWorldImage/testWorldImage: /usr/lib/libACE.so
testWorldImage/testWorldImage: testWorldImage/CMakeFiles/testWorldImage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable testWorldImage"
	cd /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/testWorldImage && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testWorldImage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
testWorldImage/CMakeFiles/testWorldImage.dir/build: testWorldImage/testWorldImage
.PHONY : testWorldImage/CMakeFiles/testWorldImage.dir/build

testWorldImage/CMakeFiles/testWorldImage.dir/requires: testWorldImage/CMakeFiles/testWorldImage.dir/testWorldImage.cpp.o.requires
.PHONY : testWorldImage/CMakeFiles/testWorldImage.dir/requires

testWorldImage/CMakeFiles/testWorldImage.dir/clean:
	cd /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/testWorldImage && $(CMAKE_COMMAND) -P CMakeFiles/testWorldImage.dir/cmake_clean.cmake
.PHONY : testWorldImage/CMakeFiles/testWorldImage.dir/clean

testWorldImage/CMakeFiles/testWorldImage.dir/depend:
	cd /usr/local/robot/iCub/contrib/src/stereoVision/src/modules && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/local/robot/iCub/contrib/src/stereoVision/src/modules /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/testWorldImage /usr/local/robot/iCub/contrib/src/stereoVision/src/modules /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/testWorldImage /usr/local/robot/iCub/contrib/src/stereoVision/src/modules/testWorldImage/CMakeFiles/testWorldImage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : testWorldImage/CMakeFiles/testWorldImage.dir/depend

