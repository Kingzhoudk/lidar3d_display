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
CMAKE_COMMAND = /home/uisee/Downloads/clion-2020.1.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/uisee/Downloads/clion-2020.1.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/uisee/Desktop/lidar3d_display

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/uisee/Desktop/lidar3d_display/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/lidar3d.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lidar3d.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lidar3d.dir/flags.make

CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.o: CMakeFiles/lidar3d.dir/flags.make
CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.o: ../src/lidar3d_data/lidar3d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/uisee/Desktop/lidar3d_display/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.o -c /home/uisee/Desktop/lidar3d_display/src/lidar3d_data/lidar3d.cpp

CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/uisee/Desktop/lidar3d_display/src/lidar3d_data/lidar3d.cpp > CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.i

CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/uisee/Desktop/lidar3d_display/src/lidar3d_data/lidar3d.cpp -o CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.s

CMakeFiles/lidar3d.dir/main.cpp.o: CMakeFiles/lidar3d.dir/flags.make
CMakeFiles/lidar3d.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/uisee/Desktop/lidar3d_display/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/lidar3d.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lidar3d.dir/main.cpp.o -c /home/uisee/Desktop/lidar3d_display/main.cpp

CMakeFiles/lidar3d.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lidar3d.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/uisee/Desktop/lidar3d_display/main.cpp > CMakeFiles/lidar3d.dir/main.cpp.i

CMakeFiles/lidar3d.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lidar3d.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/uisee/Desktop/lidar3d_display/main.cpp -o CMakeFiles/lidar3d.dir/main.cpp.s

CMakeFiles/lidar3d.dir/src/system/display.cpp.o: CMakeFiles/lidar3d.dir/flags.make
CMakeFiles/lidar3d.dir/src/system/display.cpp.o: ../src/system/display.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/uisee/Desktop/lidar3d_display/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/lidar3d.dir/src/system/display.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lidar3d.dir/src/system/display.cpp.o -c /home/uisee/Desktop/lidar3d_display/src/system/display.cpp

CMakeFiles/lidar3d.dir/src/system/display.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lidar3d.dir/src/system/display.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/uisee/Desktop/lidar3d_display/src/system/display.cpp > CMakeFiles/lidar3d.dir/src/system/display.cpp.i

CMakeFiles/lidar3d.dir/src/system/display.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lidar3d.dir/src/system/display.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/uisee/Desktop/lidar3d_display/src/system/display.cpp -o CMakeFiles/lidar3d.dir/src/system/display.cpp.s

# Object files for target lidar3d
lidar3d_OBJECTS = \
"CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.o" \
"CMakeFiles/lidar3d.dir/main.cpp.o" \
"CMakeFiles/lidar3d.dir/src/system/display.cpp.o"

# External object files for target lidar3d
lidar3d_EXTERNAL_OBJECTS =

lidar3d: CMakeFiles/lidar3d.dir/src/lidar3d_data/lidar3d.cpp.o
lidar3d: CMakeFiles/lidar3d.dir/main.cpp.o
lidar3d: CMakeFiles/lidar3d.dir/src/system/display.cpp.o
lidar3d: CMakeFiles/lidar3d.dir/build.make
lidar3d: /usr/local/lib/libopencv_shape.so.3.2.0
lidar3d: /usr/local/lib/libopencv_stitching.so.3.2.0
lidar3d: /usr/local/lib/libopencv_superres.so.3.2.0
lidar3d: /usr/local/lib/libopencv_videostab.so.3.2.0
lidar3d: /usr/lib/x86_64-linux-gnu/libprotobuf.so
lidar3d: /usr/local/lib/libpangolin.so
lidar3d: /usr/lib/libpcl_segmentation.so
lidar3d: /usr/lib/libpcl_surface.so
lidar3d: /usr/lib/libpcl_recognition.so
lidar3d: /usr/lib/libpcl_keypoints.so
lidar3d: /usr/lib/libpcl_tracking.so
lidar3d: /usr/lib/libpcl_stereo.so
lidar3d: /usr/lib/x86_64-linux-gnu/libboost_system.so
lidar3d: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
lidar3d: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
lidar3d: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
lidar3d: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
lidar3d: /usr/lib/x86_64-linux-gnu/libboost_regex.so
lidar3d: /usr/lib/x86_64-linux-gnu/libqhull.so
lidar3d: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
lidar3d: /usr/local/lib/libopencv_objdetect.so.3.2.0
lidar3d: /usr/local/lib/libopencv_calib3d.so.3.2.0
lidar3d: /usr/local/lib/libopencv_features2d.so.3.2.0
lidar3d: /usr/local/lib/libopencv_flann.so.3.2.0
lidar3d: /usr/local/lib/libopencv_highgui.so.3.2.0
lidar3d: /usr/local/lib/libopencv_ml.so.3.2.0
lidar3d: /usr/local/lib/libopencv_photo.so.3.2.0
lidar3d: /usr/local/lib/libopencv_video.so.3.2.0
lidar3d: /usr/local/lib/libopencv_videoio.so.3.2.0
lidar3d: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
lidar3d: /usr/local/lib/libopencv_imgproc.so.3.2.0
lidar3d: /usr/local/lib/libopencv_core.so.3.2.0
lidar3d: /usr/lib/x86_64-linux-gnu/libGLU.so
lidar3d: /usr/lib/x86_64-linux-gnu/libGL.so
lidar3d: /usr/lib/x86_64-linux-gnu/libGLEW.so
lidar3d: /usr/lib/x86_64-linux-gnu/libX11.so
lidar3d: /usr/lib/x86_64-linux-gnu/libXext.so
lidar3d: /usr/lib/x86_64-linux-gnu/libdc1394.so
lidar3d: /usr/lib/x86_64-linux-gnu/libavcodec.so
lidar3d: /usr/lib/x86_64-linux-gnu/libavformat.so
lidar3d: /usr/lib/x86_64-linux-gnu/libavutil.so
lidar3d: /usr/lib/x86_64-linux-gnu/libswscale.so
lidar3d: /usr/lib/x86_64-linux-gnu/libavdevice.so
lidar3d: /usr/lib/x86_64-linux-gnu/libpng.so
lidar3d: /usr/lib/x86_64-linux-gnu/libz.so
lidar3d: /usr/lib/x86_64-linux-gnu/libjpeg.so
lidar3d: /usr/lib/x86_64-linux-gnu/libtiff.so
lidar3d: /usr/lib/x86_64-linux-gnu/libIlmImf.so
lidar3d: /usr/lib/libpcl_ml.so
lidar3d: /usr/lib/libpcl_registration.so
lidar3d: /usr/lib/libpcl_features.so
lidar3d: /usr/lib/libpcl_filters.so
lidar3d: /usr/lib/libpcl_sample_consensus.so
lidar3d: /usr/lib/libpcl_search.so
lidar3d: /usr/lib/libpcl_kdtree.so
lidar3d: /usr/lib/libpcl_io.so
lidar3d: /usr/lib/libpcl_octree.so
lidar3d: /usr/lib/libpcl_common.so
lidar3d: CMakeFiles/lidar3d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/uisee/Desktop/lidar3d_display/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable lidar3d"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lidar3d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lidar3d.dir/build: lidar3d

.PHONY : CMakeFiles/lidar3d.dir/build

CMakeFiles/lidar3d.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lidar3d.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lidar3d.dir/clean

CMakeFiles/lidar3d.dir/depend:
	cd /home/uisee/Desktop/lidar3d_display/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/uisee/Desktop/lidar3d_display /home/uisee/Desktop/lidar3d_display /home/uisee/Desktop/lidar3d_display/cmake-build-debug /home/uisee/Desktop/lidar3d_display/cmake-build-debug /home/uisee/Desktop/lidar3d_display/cmake-build-debug/CMakeFiles/lidar3d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lidar3d.dir/depend

