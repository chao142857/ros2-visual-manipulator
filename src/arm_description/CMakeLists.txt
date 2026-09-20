cmake_minimum_required(VERSION 3.8)
project(arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  # Explicit inputs keep macOS AppleDouble metadata out of source linters.
  set(arm_description_python_lint_inputs
    launch/robot_state.launch.py
    launch/view_arm.launch.py
  )
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_dummy_v2_meshes.py")
    list(APPEND arm_description_python_lint_inputs tools/generate_dummy_v2_meshes.py)
  endif()
  ament_flake8(${arm_description_python_lint_inputs})
  ament_lint_cmake(CMakeLists.txt)
  ament_pep257(${arm_description_python_lint_inputs})
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf/dummy-v2.urdf")
    ament_xmllint(urdf/dummy-v2.urdf)
  else()
    message(STATUS "Model XML lint skipped: no URDF in this software-only package")
  endif()
endif()

install(
  DIRECTORY
  urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "._*" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
