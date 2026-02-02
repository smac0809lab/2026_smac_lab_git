cmake_minimum_required(VERSION 3.8)
project(ekf_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지들
find_package(ament_cmake REQUIRED)
find_package(ublox_gps REQUIRED)
find_package(robot_localization REQUIRED)

# [핵심] 런치 파일과 설정 파일을 설치 폴더로 복사하는 코드
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
