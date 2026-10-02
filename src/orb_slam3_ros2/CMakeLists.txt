cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2)

# ── ORB-SLAM3 위치 ───────────────────────────────────────────────
# ORB-SLAM3 와 Pangolin 은 colcon 트리 밖에서 빌드한다 (PC 전용).
# 다른 위치에 빌드했다면 환경변수로 덮어쓴다:
#   ORB_SLAM3_ROOT=/path/to/ORB_SLAM3  PANGOLIN_PREFIX=/path/to/install  colcon build
if(DEFINED ENV{ORB_SLAM3_ROOT})
  set(ORB_SLAM3_ROOT "$ENV{ORB_SLAM3_ROOT}")
else()
  set(ORB_SLAM3_ROOT "$ENV{HOME}/workspace/third_party/ORB_SLAM3")
endif()
if(DEFINED ENV{PANGOLIN_PREFIX})
  set(PANGOLIN_PREFIX "$ENV{PANGOLIN_PREFIX}")
else()
  set(PANGOLIN_PREFIX "$ENV{HOME}/workspace/third_party/install")
endif()

# Pi 에는 ORB-SLAM3 가 없다. 같은 저장소를 Pi 에서도 colcon build 하므로, 못 찾으면
# 빌드를 깨지 말고 이 패키지만 건너뛴다. ament_package() 를 부르지 않으면 패키지가
# ament 인덱스에 등록되지 않아, slam_orbslam3.launch.py 가 "패키지 없음" 으로
# 명확히 멈춘다.
set(ORB_SLAM3_LIB "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so")
if(NOT EXISTS "${ORB_SLAM3_LIB}")
  message(WARNING
    "orb_slam3_ros2: ${ORB_SLAM3_LIB} 이 없어 이 패키지를 건너뛴다 (Pi 라면 정상). "
    "PC 라면 ORB-SLAM3 를 빌드하거나 ORB_SLAM3_ROOT 를 지정할 것.")
  return()
endif()

# -march=native 는 쓰지 않는다. ORB-SLAM3/g2o/DBoW2 를 -march=native 로 빌드하면
# Ubuntu 22.04 에서 첫 맵 초기화 직후 "double free or corruption (out)" 으로 죽는다
# (Eigen 정렬이 32 바이트로 올라가 C++11 라이브러리와 C++17 래퍼 사이 new/delete 가 어긋남).
# 그래서 ORB_SLAM3, Thirdparty/g2o, Thirdparty/DBoW2 의 CMakeLists 에서도 플래그를 지우고
# 빌드해야 하며, 래퍼도 같은 기본 정렬(16 바이트)로 맞춘다.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3)

# PC 셸에서는 conda 환경(miniforge3/envs/rc_car)이 켜져 있어 python3 가 그쪽을 가리킨다.
# 그 환경엔 catkin_pkg 가 없어 ament_package() 가 package.xml 을 못 읽고 실패한다.
# ROS Humble 은 시스템 파이썬 기준이므로 명시적으로 지정하지 않았으면 그걸 쓴다.
if(NOT DEFINED PYTHON_EXECUTABLE AND EXISTS /usr/bin/python3)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  set(Python3_EXECUTABLE /usr/bin/python3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED PATHS "${PANGOLIN_PREFIX}/lib/cmake/Pangolin" NO_DEFAULT_PATH)

add_executable(mono_node src/mono_node.cpp)
target_include_directories(mono_node PRIVATE
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
  ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(mono_node
  "${ORB_SLAM3_LIB}"
  "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so"
  "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so"
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)
ament_target_dependencies(mono_node rclcpp sensor_msgs geometry_msgs std_msgs std_srvs cv_bridge)

# 위 .so 들은 표준 라이브러리 경로 밖에 있으므로 RPATH 로 찾게 한다
# (LD_LIBRARY_PATH 를 셸마다 맞출 필요가 없도록).
set_target_properties(mono_node PROPERTIES
  INSTALL_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib;${PANGOLIN_PREFIX}/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS mono_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/scale_aligner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scale_aligner)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
