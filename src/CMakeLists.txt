PROJECT(the_first_book)
cmake_minimum_required(VERSION 3.27)

#set(OpenCV_DIR D:/INSTALLED_SOFTWARE/OpenCV-MinGW-Build-OpenCV-3.4.8-x64)
set(OpenCV_DIR D:/OpenCV-MinGW-Build-OpenCV-3.4.8-x64)
find_package(OpenCV REQUIRED)


# 相机标定
set(CAL_PROJECT_NAME calibrate_camera)
set(CAL_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/calibration.cpp)
add_executable(${CAL_PROJECT_NAME} ${CAL_PROJECT_SOURCES})
target_link_libraries(${CAL_PROJECT_NAME} pthread )
target_link_libraries(${CAL_PROJECT_NAME} ${OpenCV_LIBS})


add_executable(icar_pic ./src/icar_pic.cpp)
target_link_libraries(icar_pic ${OpenCV_LIBS}) 