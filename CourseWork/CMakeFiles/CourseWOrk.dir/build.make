# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /usr/bin/cmake3

# The command to remove a file.
RM = /usr/bin/cmake3 -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork

# Include any dependencies generated for this target.
include CMakeFiles/CourseWOrk.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/CourseWOrk.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/CourseWOrk.dir/flags.make

CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o: CMakeFiles/CourseWOrk.dir/flags.make
CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o: CourseWork.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o -c /home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork/CourseWork.cpp

CMakeFiles/CourseWOrk.dir/CourseWork.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CourseWOrk.dir/CourseWork.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork/CourseWork.cpp > CMakeFiles/CourseWOrk.dir/CourseWork.cpp.i

CMakeFiles/CourseWOrk.dir/CourseWork.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CourseWOrk.dir/CourseWork.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork/CourseWork.cpp -o CMakeFiles/CourseWOrk.dir/CourseWork.cpp.s

CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o.requires:

.PHONY : CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o.requires

CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o.provides: CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o.requires
	$(MAKE) -f CMakeFiles/CourseWOrk.dir/build.make CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o.provides.build
.PHONY : CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o.provides

CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o.provides.build: CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o


# Object files for target CourseWOrk
CourseWOrk_OBJECTS = \
"CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o"

# External object files for target CourseWOrk
CourseWOrk_EXTERNAL_OBJECTS =

CourseWOrk: CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o
CourseWOrk: CMakeFiles/CourseWOrk.dir/build.make
CourseWOrk: /opt/icub/lib64/libYARP_dev.so.2.3.70
CourseWOrk: /opt/icub/lib64/libYARP_name.so.2.3.70
CourseWOrk: /opt/icub/lib64/libYARP_init.so.2.3.70
CourseWOrk: /usr/lib64/libopencv_calib3d.so
CourseWOrk: /usr/lib64/libopencv_contrib.so
CourseWOrk: /usr/lib64/libopencv_core.so
CourseWOrk: /usr/lib64/libopencv_features2d.so
CourseWOrk: /usr/lib64/libopencv_flann.so
CourseWOrk: /usr/lib64/libopencv_highgui.so
CourseWOrk: /usr/lib64/libopencv_imgproc.so
CourseWOrk: /usr/lib64/libopencv_legacy.so
CourseWOrk: /usr/lib64/libopencv_ml.so
CourseWOrk: /usr/lib64/libopencv_objdetect.so
CourseWOrk: /usr/lib64/libopencv_photo.so
CourseWOrk: /usr/lib64/libopencv_stitching.so
CourseWOrk: /usr/lib64/libopencv_superres.so
CourseWOrk: /usr/lib64/libopencv_ts.so
CourseWOrk: /usr/lib64/libopencv_video.so
CourseWOrk: /usr/lib64/libopencv_videostab.so
CourseWOrk: /opt/icub/lib64/libYARP_math.so.2.3.70
CourseWOrk: /opt/icub/lib64/libYARP_sig.so.2.3.70
CourseWOrk: /opt/icub/lib64/libYARP_OS.so.2.3.70
CourseWOrk: CMakeFiles/CourseWOrk.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable CourseWOrk"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CourseWOrk.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/CourseWOrk.dir/build: CourseWOrk

.PHONY : CMakeFiles/CourseWOrk.dir/build

CMakeFiles/CourseWOrk.dir/requires: CMakeFiles/CourseWOrk.dir/CourseWork.cpp.o.requires

.PHONY : CMakeFiles/CourseWOrk.dir/requires

CMakeFiles/CourseWOrk.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CourseWOrk.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CourseWOrk.dir/clean

CMakeFiles/CourseWOrk.dir/depend:
	cd /home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork /home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork /home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork /home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork /home/msc/mmm18/private/rOBOTIC/Orginal/CourseWork/CMakeFiles/CourseWOrk.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/CourseWOrk.dir/depend

