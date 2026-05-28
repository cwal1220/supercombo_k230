cmake_minimum_required(VERSION 3.16)

project(supercombo_k230 LANGUAGES C CXX)

option(SUPERCOMBO_BUILD_BENCHMARKS "Build standalone benchmark/diagnostic utilities" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
foreach(config_type DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config_type} "${CMAKE_CURRENT_SOURCE_DIR}")
endforeach()

set(K230_SYSROOT_LIB_DIR "/usr/lib/riscv64-linux-gnu" CACHE PATH
    "K230 system library directory")
set(NNCASE_DEPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps" CACHE PATH
    "nncase K230 runtime dependency directory")

set(PROJECT_WARNINGS -Wall -Wextra)
set(PROJECT_LOCAL_INCLUDES
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
set(PROJECT_SYSTEM_INCLUDES
  "${NNCASE_DEPS_DIR}/include"
  "/usr/include/libdrm"
)

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
      "Run ./fetch_nncase_runtime.sh for deps/, and install/restore K230 display libraries.")
  endif()
endforeach()

add_library(mmz STATIC src/mmz.c)
target_include_directories(mmz PUBLIC ${PROJECT_LOCAL_INCLUDES})
target_include_directories(mmz SYSTEM PUBLIC ${PROJECT_SYSTEM_INCLUDES})
target_compile_options(mmz PRIVATE ${PROJECT_WARNINGS})

function(configure_k230_target target)
  target_include_directories(${target} PRIVATE ${PROJECT_LOCAL_INCLUDES})
  target_include_directories(${target} SYSTEM PRIVATE ${PROJECT_SYSTEM_INCLUDES})
  target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
  target_link_libraries(${target} PRIVATE
    -Wl,--start-group
    ${ARGN}
    -Wl,--end-group
    pthread
    atomic
    dl
  )
endfunction()

set(COMMON_SOURCES
  src/app_config.cc
  src/model_output.cc
  src/projection.cc
)

set(MODEL_SOURCES
  src/ai_base.cc
  src/calibration_service.cc
  src/input_source.cc
  src/k230_ipc.cc
  src/lateral_control.cc
  src/online_calibrator.cc
  src/supercombo_model.cc
)

add_executable(supercombo_elf
  src/main.cc
  src/overlay_renderer.cc
  src/supercombo_runtime.cc
  ${COMMON_SOURCES}
  ${MODEL_SOURCES}
)
set_target_properties(supercombo_elf PROPERTIES OUTPUT_NAME "supercombo.elf")
configure_k230_target(supercombo_elf
  ${NNCASE_STATIC_LIBS}
  mmz
  ${K230_SHARED_LIBS}
)

add_executable(k230_camerad
  src/k230_camerad.cc
  src/input_source.cc
  src/k230_ipc.cc
  ${COMMON_SOURCES}
)
configure_k230_target(k230_camerad
  mmz
  ${K230_SHARED_LIBS}
)

add_executable(k230_modeld
  src/k230_modeld.cc
  ${COMMON_SOURCES}
  ${MODEL_SOURCES}
)
configure_k230_target(k230_modeld
  ${NNCASE_STATIC_LIBS}
  mmz
  ${K230_SHARED_LIBS}
)

add_executable(k230_overlay
  src/k230_overlay.cc
  src/k230_ipc.cc
  src/overlay_renderer.cc
  ${COMMON_SOURCES}
)
configure_k230_target(k230_overlay
  mmz
  ${K230_SHARED_LIBS}
)

if(SUPERCOMBO_BUILD_BENCHMARKS)
  add_executable(check_model_output_parser
    benchmarks/check_model_output_parser.cc
    src/model_output.cc
  )
  target_include_directories(check_model_output_parser PRIVATE ${PROJECT_LOCAL_INCLUDES})
  target_include_directories(check_model_output_parser SYSTEM PRIVATE ${PROJECT_SYSTEM_INCLUDES})
  target_compile_options(check_model_output_parser PRIVATE ${PROJECT_WARNINGS})

  add_executable(bench_nv12_to_yuv6 benchmarks/bench_nv12_to_yuv6.cc)
  target_compile_options(bench_nv12_to_yuv6 PRIVATE ${PROJECT_WARNINGS})

  add_executable(bench_capture_nv12 benchmarks/bench_capture_nv12.cc)
  configure_k230_target(bench_capture_nv12 mmz ${K230_SHARED_LIBS})

  add_executable(bench_ai2d_resize benchmarks/bench_ai2d_resize.cc)
  configure_k230_target(bench_ai2d_resize ${NNCASE_STATIC_LIBS} mmz ${K230_SHARED_LIBS})

  add_executable(sequence_runner benchmarks/sequence_runner.cc)
  configure_k230_target(sequence_runner ${NNCASE_STATIC_LIBS} mmz ${K230_SHARED_LIBS})

  add_executable(probe_drm_planes benchmarks/probe_drm_planes.cc)
  configure_k230_target(probe_drm_planes mmz ${K230_SHARED_LIBS})
endif()

install(TARGETS supercombo_elf k230_camerad k230_modeld k230_overlay
  RUNTIME DESTINATION .
)
install(PROGRAMS k230_manager.py DESTINATION .)
install(DIRECTORY models DESTINATION . OPTIONAL FILES_MATCHING PATTERN "*.kmodel")
