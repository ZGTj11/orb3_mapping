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
CMAKE_SOURCE_DIR = /home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3/build

# Include any dependencies generated for this target.
include CMakeFiles/stereo_tum_vi.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_tum_vi.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_tum_vi.dir/flags.make

CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.o: CMakeFiles/stereo_tum_vi.dir/flags.make
CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.o: ../Examples/Stereo/stereo_tum_vi.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.o -c /home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3/Examples/Stereo/stereo_tum_vi.cc

CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3/Examples/Stereo/stereo_tum_vi.cc > CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.i

CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3/Examples/Stereo/stereo_tum_vi.cc -o CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.s

# Object files for target stereo_tum_vi
stereo_tum_vi_OBJECTS = \
"CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.o"

# External object files for target stereo_tum_vi
stereo_tum_vi_EXTERNAL_OBJECTS =

../Examples/Stereo/stereo_tum_vi: CMakeFiles/stereo_tum_vi.dir/Examples/Stereo/stereo_tum_vi.cc.o
../Examples/Stereo/stereo_tum_vi: CMakeFiles/stereo_tum_vi.dir/build.make
../Examples/Stereo/stereo_tum_vi: ../lib/libORB_SLAM3.so
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_gapi.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_highgui.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_ml.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_objdetect.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_photo.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_stitching.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_video.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_calib3d.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_dnn.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_features2d.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_flann.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_videoio.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_imgcodecs.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_imgproc.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /home/robot/mylib/opencv455/lib/libopencv_core.so.4.5.5
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_glgeometry.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_geometry.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_plot.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_python.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_scene.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_tools.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_display.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_vars.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_video.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_packetstream.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_windowing.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_opengl.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_image.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libpango_core.so
../Examples/Stereo/stereo_tum_vi: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/Stereo/stereo_tum_vi: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../Examples/Stereo/stereo_tum_vi: /usr/lib/x86_64-linux-gnu/libGLX.so
../Examples/Stereo/stereo_tum_vi: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/Stereo/stereo_tum_vi: /usr/local/lib/libtinyobj.so
../Examples/Stereo/stereo_tum_vi: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/Stereo/stereo_tum_vi: ../Thirdparty/g2o/lib/libg2o.so
../Examples/Stereo/stereo_tum_vi: /opt/ros/noetic/lib/x86_64-linux-gnu/librealsense2.so.2.50.0
../Examples/Stereo/stereo_tum_vi: CMakeFiles/stereo_tum_vi.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/Stereo/stereo_tum_vi"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_tum_vi.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_tum_vi.dir/build: ../Examples/Stereo/stereo_tum_vi

.PHONY : CMakeFiles/stereo_tum_vi.dir/build

CMakeFiles/stereo_tum_vi.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_tum_vi.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_tum_vi.dir/clean

CMakeFiles/stereo_tum_vi.dir/depend:
	cd /home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3 /home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3 /home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3/build /home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3/build /home/robot/SLAM/orb_slam3_pcl_mapping/ORB_SLAM3/build/CMakeFiles/stereo_tum_vi.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo_tum_vi.dir/depend

