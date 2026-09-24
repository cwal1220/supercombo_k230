# 호스트 단위 테스트. host_tests가 전부를 빌드하고 ctest가 모은다(scripts/run_host_tests.sh).
include(FetchContent)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.18.0.tar.gz
  URL_HASH SHA256=6e3191c1455468b3fc35a417fb565c1c5071aee1b7e7f85e30cf48a98d37d8b5)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

# add_host_test(<이름> <라이브러리...>): <이름>.cc 하나로 된 테스트. 저장소 루트에서 돈다
# (params/ 같은 상대 경로를 읽는 테스트가 있다).
add_custom_target(host_tests)
function(add_host_test target)
  add_host_tool(${target} ${target}.cc)
  target_link_libraries(${target} PRIVATE ${ARGN} GTest::gtest_main)
  gtest_discover_tests(${target} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  add_dependencies(host_tests ${target})
endfunction()

add_host_test(gtest_adaptive_cruise control_core)
add_host_test(gtest_calibration_equivalence perception)
add_host_test(gtest_can_queue common)
add_host_test(gtest_control_replay control_core)
add_host_test(gtest_departure_alert control_core)
add_host_test(gtest_lateral_learners control_core)
add_host_test(gtest_lateral_mpc planning)
add_host_test(gtest_model_output_parser common)
add_host_test(gtest_overlay_state overlay_state)
add_host_test(gtest_panda_can_codec panda_codec)
add_host_test(gtest_recording_writer recording_writer)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME check_param_server.py
         COMMAND ${Python3_EXECUTABLE} diagnostics/check_param_server.py
         WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
