# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.8

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = D:\cmake\bin\cmake.exe

# The command to remove a file.
RM = D:\cmake\bin\cmake.exe -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\build

# Include any dependencies generated for this target.
include CMakeFiles/icar_pic.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/icar_pic.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/icar_pic.dir/flags.make

CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj: CMakeFiles/icar_pic.dir/flags.make
CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj: CMakeFiles/icar_pic.dir/includes_CXX.rsp
CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj: C:/Users/nanwe/Desktop/nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm/src/src/icar_pic.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj"
	cd /d C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\build && C:\msys64\mingw64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\icar_pic.dir\src\icar_pic.cpp.obj -c C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\src\src\icar_pic.cpp

CMakeFiles/icar_pic.dir/src/icar_pic.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icar_pic.dir/src/icar_pic.cpp.i"
	cd /d C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\build && C:\msys64\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\src\src\icar_pic.cpp > CMakeFiles\icar_pic.dir\src\icar_pic.cpp.i

CMakeFiles/icar_pic.dir/src/icar_pic.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icar_pic.dir/src/icar_pic.cpp.s"
	cd /d C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\build && C:\msys64\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\src\src\icar_pic.cpp -o CMakeFiles\icar_pic.dir\src\icar_pic.cpp.s

CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj.requires:

.PHONY : CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj.requires

CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj.provides: CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj.requires
	$(MAKE) -f CMakeFiles\icar_pic.dir\build.make CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj.provides.build
.PHONY : CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj.provides

CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj.provides.build: CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj


# Object files for target icar_pic
icar_pic_OBJECTS = \
"CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj"

# External object files for target icar_pic
icar_pic_EXTERNAL_OBJECTS =

icar_pic.exe: CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj
icar_pic.exe: CMakeFiles/icar_pic.dir/build.make
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_dnn348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_highgui348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_ml348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_objdetect348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_shape348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_stitching348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_superres348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_videostab348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_calib3d348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_features2d348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_flann348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_photo348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_video348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_videoio348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_imgcodecs348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_imgproc348.dll.a
icar_pic.exe: D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64/x64/mingw/lib/libopencv_core348.dll.a
icar_pic.exe: CMakeFiles/icar_pic.dir/linklibs.rsp
icar_pic.exe: CMakeFiles/icar_pic.dir/objects1.rsp
icar_pic.exe: CMakeFiles/icar_pic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable icar_pic.exe"
	cd /d C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\build && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\icar_pic.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/icar_pic.dir/build: icar_pic.exe

.PHONY : CMakeFiles/icar_pic.dir/build

CMakeFiles/icar_pic.dir/requires: CMakeFiles/icar_pic.dir/src/icar_pic.cpp.obj.requires

.PHONY : CMakeFiles/icar_pic.dir/requires

CMakeFiles/icar_pic.dir/clean:
	cd /d C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\build && $(CMAKE_COMMAND) -P CMakeFiles\icar_pic.dir\cmake_clean.cmake
.PHONY : CMakeFiles/icar_pic.dir/clean

CMakeFiles/icar_pic.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\src C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\src C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\build C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\build C:\Users\nanwe\Desktop\nan_baidu_maincontrol+_240604_zhaijiao_camera_ipm\build\CMakeFiles\icar_pic.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/icar_pic.dir/depend
