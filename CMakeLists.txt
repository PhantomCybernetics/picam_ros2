cmake_minimum_required(VERSION 3.8)
project(picam_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(cv_bridge REQUIRED)

find_library(AVCODEC_LIBRARY avcodec)
find_library(AVUTIL_LIBRARY avutil)
find_library(FMT_LIBRARY fmt)
find_library(JSONCPP_LIBRARY jsoncpp)

pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(/usr/local/include/libcamera/)
include_directories(/usr/include/jsoncpp/)
include_directories(./include/)
include_directories(/usr/include)

add_executable(picam
              src/picam_ros2.cpp
              src/camera_interface.cpp
              src/encoder_base.cpp
              src/encoder_libav.cpp
              src/encoder_hw.cpp
              src/calibration.cpp
              src/dma_heaps.cpp
              )
ament_target_dependencies(picam
                          rclcpp
                          std_msgs
                          sensor_msgs
                          std_srvs
                          ffmpeg_image_transport_msgs
                          OpenCV
                          )

target_link_libraries(picam
  ${LIBCAMERA_LIBRARIES}
  ${AVCODEC_LIBRARY}
  ${AVUTIL_LIBRARY}
  ${FMT_LIBRARY}
  ${JSONCPP_LIBRARY}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS
  picam
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
