cmake_minimum_required(VERSION 3.16)

project(supercombo_k230 LANGUAGES C CXX)

option(SUPERCOMBO_BUILD_DIAGNOSTICS "Build standalone benchmark/diagnostic utilities" OFF)
option(SUPERCOMBO_BUILD_PANDA "Build optional panda USB/CAN bridge" ON)
option(SUPERCOMBO_BUILD_RUNTIME "Build K230 runtime binaries" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
foreach(config_type DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config_type} "${CMAKE_BINARY_DIR}/bin")
endforeach()

set(K230_SYSROOT "" CACHE PATH "K230 target sysroot")
if(K230_SYSROOT)
  set(_K230_DEFAULT_LIB_DIR "${K230_SYSROOT}/usr/lib")
  set(_K230_DEFAULT_BASE_LIB_DIR "${K230_SYSROOT}/lib")
  set(_K230_DEFAULT_INCLUDE_DIR "${K230_SYSROOT}/usr/include")
else()
  set(_K230_DEFAULT_LIB_DIR "/usr/lib/riscv64-linux-gnu")
  set(_K230_DEFAULT_BASE_LIB_DIR "/lib/riscv64-linux-gnu")
  set(_K230_DEFAULT_INCLUDE_DIR "/usr/include")
endif()
set(K230_SYSROOT_LIB_DIR "${_K230_DEFAULT_LIB_DIR}" CACHE PATH
    "K230 system library directory")
set(K230_SYSROOT_BASE_LIB_DIR "${_K230_DEFAULT_BASE_LIB_DIR}" CACHE PATH
    "K230 base system library directory")
set(K230_SYSROOT_INCLUDE_DIR "${_K230_DEFAULT_INCLUDE_DIR}" CACHE PATH
    "K230 system include directory")
set(NNCASE_DEPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps" CACHE PATH
    "nncase K230 runtime dependency directory")

set(PROJECT_WARNINGS -Wall -Wextra)
set(PROJECT_LOCAL_INCLUDES
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
set(PROJECT_SYSTEM_INCLUDES)

# 런타임(overlayd)에는 필수, 호스트 검사에서는 hud_snapshot만 쓴다.
if(SUPERCOMBO_BUILD_RUNTIME)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
else()
  find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs)
endif()

# 프로세스 공용 코드. 호스트 검사도 쓴다. glibc는 shm_open이 librt에 있고 macOS는 libc에 있다.
add_library(k230_common STATIC
  src/app_config.cc
  src/k230_ipc.cc
  src/model_output.cc
  src/projection.cc
)
target_include_directories(k230_common PUBLIC ${PROJECT_LOCAL_INCLUDES})
target_compile_options(k230_common PRIVATE ${PROJECT_WARNINGS})
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(k230_common PUBLIC rt pthread)
endif()

add_library(control_core STATIC
  src/control_params.cc
  src/departure_alert.cc
  src/hyundai_can.cc
  src/json_utils.cc
  src/adaptive_cruise.cc
  src/lateral_controller.cc
  src/lateral_path.cc
  src/torque_controller.cc
  src/vehicle_can.cc
)
target_include_directories(control_core PUBLIC ${PROJECT_LOCAL_INCLUDES})
target_compile_options(control_core PRIVATE ${PROJECT_WARNINGS})

# 호스트 검사·도구: 로컬 헤더와 경고만 붙인 일반 실행 파일.
function(add_host_tool target)
  add_executable(${target} ${ARGN})
  target_include_directories(${target} PRIVATE ${PROJECT_LOCAL_INCLUDES})
  target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
endfunction()

if(SUPERCOMBO_BUILD_RUNTIME)
  find_package(PkgConfig QUIET)
  if(SUPERCOMBO_BUILD_PANDA)
    if(PkgConfig_FOUND)
      pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    else()
      message(FATAL_ERROR "PkgConfig is required when SUPERCOMBO_BUILD_PANDA=ON")
    endif()
  endif()

  list(APPEND PROJECT_SYSTEM_INCLUDES
    "${NNCASE_DEPS_DIR}/include"
    "${K230_SYSROOT_INCLUDE_DIR}/libdrm"
    ${OpenCV_INCLUDE_DIRS}
  )
  if(SUPERCOMBO_BUILD_PANDA)
    list(APPEND PROJECT_SYSTEM_INCLUDES ${LIBUSB_INCLUDE_DIRS})
  endif()

  set(NNCASE_STATIC_LIBS
    "${NNCASE_DEPS_DIR}/lib/libNncase.Runtime.Native.a"
    "${NNCASE_DEPS_DIR}/lib/libnncase.rt_modules.k230.a"
    "${NNCASE_DEPS_DIR}/lib/libfunctional_k230.a"
  )

  set(K230_SHARED_LIBS
    "${K230_SYSROOT_LIB_DIR}/libv4l2-drm.so"
    "${K230_SYSROOT_LIB_DIR}/libdisplay.so"
    "${K230_SYSROOT_LIB_DIR}/libdrm.so.2"
  )

  foreach(required_lib IN LISTS NNCASE_STATIC_LIBS K230_SHARED_LIBS)
    if(NOT EXISTS "${required_lib}")
      message(FATAL_ERROR
        "Missing required library: ${required_lib}\n"
        "Run scripts/fetch_nncase_runtime.sh for deps/, and install/restore K230 display libraries.")
    endif()
  endforeach()

  add_library(mmz STATIC src/mmz.c)
  target_include_directories(mmz PUBLIC ${PROJECT_LOCAL_INCLUDES})
  target_include_directories(mmz SYSTEM PUBLIC ${PROJECT_SYSTEM_INCLUDES})
  target_compile_options(mmz PRIVATE ${PROJECT_WARNINGS})

  # 보드 실행 파일: 시스템 헤더, 링크 경로, 공용 시스템 라이브러리.
  function(configure_k230_target target)
    target_include_directories(${target} PRIVATE ${PROJECT_LOCAL_INCLUDES})
    target_include_directories(${target} SYSTEM PRIVATE ${PROJECT_SYSTEM_INCLUDES})
    target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
    target_link_directories(${target} PRIVATE ${K230_SYSROOT_LIB_DIR} ${K230_SYSROOT_BASE_LIB_DIR})
    target_link_options(${target} PRIVATE
      "-Wl,-rpath-link,${K230_SYSROOT_LIB_DIR}"
      "-Wl,-rpath-link,${K230_SYSROOT_BASE_LIB_DIR}"
    )
    target_link_libraries(${target} PRIVATE
      -Wl,--start-group
      ${ARGN}
      -Wl,--end-group
      pthread
      atomic
      dl
      rt
    )
  endfunction()

  add_executable(k230_camerad
    src/k230_camerad.cc
    src/input_source.cc
  )
  configure_k230_target(k230_camerad k230_common mmz ${K230_SHARED_LIBS})

  add_executable(k230_modeld
    src/k230_modeld.cc
    src/calibration_service.cc
    src/input_source.cc
    src/json_utils.cc
    src/model_input_transform.cc
    src/online_calibrator.cc
    src/supercombo_model.cc
  )
  configure_k230_target(k230_modeld k230_common ${NNCASE_STATIC_LIBS} mmz ${K230_SHARED_LIBS})

  add_executable(k230_overlayd
    src/k230_overlayd.cc
    src/piezo_buzzer.c
    src/overlay_renderer.cc
  )
  configure_k230_target(k230_overlayd k230_common mmz ${K230_SHARED_LIBS} ${OpenCV_LIBS})

  add_executable(k230_recordd
    src/k230_recordd.cc
    src/json_utils.cc
    src/mvx_v4l2_encoder.cc
    src/recording_writer.cc
  )
  configure_k230_target(k230_recordd k230_common)

  if(SUPERCOMBO_BUILD_PANDA)
    add_executable(k230_pandad
      src/k230_pandad.cc
      src/panda_client.cc
      src/panda_can_codec.cc
    )
    configure_k230_target(k230_pandad k230_common ${LIBUSB_LIBRARIES})

    add_executable(k230_controlsd
      src/k230_controlsd.cc
      src/lateral_mpc.cc
      src/lateral_planner.cc
    )
    configure_k230_target(k230_controlsd k230_common control_core)
  endif()
endif()

if(SUPERCOMBO_BUILD_DIAGNOSTICS)
  add_host_tool(check_model_output_parser diagnostics/check_model_output_parser.cc)
  target_link_libraries(check_model_output_parser PRIVATE k230_common)

  add_host_tool(bench_nv12_to_yuv6 diagnostics/bench_nv12_to_yuv6.cc)

  add_host_tool(check_panda_can_codec
    diagnostics/check_panda_can_codec.cc
    src/panda_can_codec.cc
  )

  add_host_tool(check_k230_can_queue diagnostics/check_k230_can_queue.cc)
  target_link_libraries(check_k230_can_queue PRIVATE k230_common)

  add_host_tool(check_control_replay
    diagnostics/check_control_replay.cc
    diagnostics/can_replay.cc
  )
  target_link_libraries(check_control_replay PRIVATE control_core)

  add_host_tool(extract_lateral_dataset diagnostics/extract_lateral_dataset.cc)
  target_link_libraries(extract_lateral_dataset PRIVATE control_core)

  add_host_tool(check_departure_alert diagnostics/check_departure_alert.cc)
  target_link_libraries(check_departure_alert PRIVATE control_core)

  add_host_tool(check_adaptive_cruise diagnostics/check_adaptive_cruise.cc)
  target_link_libraries(check_adaptive_cruise PRIVATE control_core)

  add_host_tool(check_lateral_mpc
    diagnostics/check_lateral_mpc.cc
    src/lateral_mpc.cc
  )

  add_host_tool(planner_replay
    diagnostics/planner_replay.cc
    src/lateral_mpc.cc
    src/lateral_planner.cc
  )
  target_link_libraries(planner_replay PRIVATE k230_common control_core)

  add_host_tool(bench_input_warp_overhead
    diagnostics/bench_input_warp_overhead.cc
    src/model_input_transform.cc
  )
  target_link_libraries(bench_input_warp_overhead PRIVATE k230_common)

  add_host_tool(verify_calibration_equivalence
    diagnostics/verify_calibration_equivalence.cc
    src/calibration_service.cc
    src/json_utils.cc
    src/model_input_transform.cc
    src/online_calibrator.cc
  )
  target_link_libraries(verify_calibration_equivalence PRIVATE k230_common)

  # HUD 스냅샷·타이밍 도구. 렌더러가 OpenCV를 쓰므로 OpenCV가 있는 곳에서만 만든다.
  if(OpenCV_FOUND)
    add_host_tool(hud_snapshot
      diagnostics/hud_snapshot.cc
      src/overlay_renderer.cc
    )
    target_include_directories(hud_snapshot SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(hud_snapshot PRIVATE k230_common ${OpenCV_LIBS})
  endif()

  if(SUPERCOMBO_BUILD_RUNTIME)
    add_executable(bench_capture_nv12 diagnostics/bench_capture_nv12.cc)
    configure_k230_target(bench_capture_nv12 mmz ${K230_SHARED_LIBS})

    add_executable(bench_ai2d_resize diagnostics/bench_ai2d_resize.cc)
    configure_k230_target(bench_ai2d_resize ${NNCASE_STATIC_LIBS} mmz ${K230_SHARED_LIBS})

    add_executable(bench_kmodel diagnostics/bench_kmodel.cc)
    configure_k230_target(bench_kmodel ${NNCASE_STATIC_LIBS} mmz ${K230_SHARED_LIBS})

    add_executable(run_kmodel_sequence diagnostics/run_kmodel_sequence.cc)
    configure_k230_target(run_kmodel_sequence ${NNCASE_STATIC_LIBS} mmz ${K230_SHARED_LIBS})

    add_executable(probe_drm_planes diagnostics/probe_drm_planes.cc)
    configure_k230_target(probe_drm_planes mmz ${K230_SHARED_LIBS})
  endif()
endif()

if(SUPERCOMBO_BUILD_RUNTIME)
  install(TARGETS k230_camerad k230_modeld k230_overlayd k230_recordd
    RUNTIME DESTINATION .
  )
  install(FILES
    assets/ui/traffic_wait_red_retro-270x155-v3.png
    assets/ui/traffic_go_green_retro-270x155-v3.png
    DESTINATION assets/ui
  )
  if(SUPERCOMBO_BUILD_PANDA)
    install(TARGETS k230_pandad k230_controlsd RUNTIME DESTINATION .)
  endif()
  install(PROGRAMS scripts/k230_manager.py scripts/param_server.py
    scripts/k230_display_control.py DESTINATION .)
  install(FILES scripts/requirements-param-server.txt DESTINATION .)
  install(DIRECTORY models DESTINATION . OPTIONAL FILES_MATCHING PATTERN "*.kmodel")
  install(DIRECTORY params DESTINATION . FILES_MATCHING PATTERN "*.json")
endif()
