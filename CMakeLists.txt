cmake_minimum_required(VERSION 3.16)

project(supercombo_k230 LANGUAGES C CXX)

option(SUPERCOMBO_BUILD_BENCHMARKS "Build standalone benchmark/diagnostic utilities" OFF)
option(SUPERCOMBO_BUILD_PANDA "Build optional panda USB/CAN bridge" OFF)
option(SUPERCOMBO_BUILD_RUNTIME "Build K230 runtime binaries" ON)

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

if(SUPERCOMBO_BUILD_RUNTIME)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc)
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
    src/model_input_transform.cc
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
    ${OpenCV_LIBS}
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
    ${OpenCV_LIBS}
  )

  if(SUPERCOMBO_BUILD_PANDA)
    add_executable(k230_pandad
      src/k230_pandad.cc
      src/panda_client.cc
      src/panda_can_codec.cc
      src/k230_ipc.cc
      src/model_output.cc
      src/projection.cc
      src/lateral_control.cc
    )
    configure_k230_target(k230_pandad ${LIBUSB_LIBRARIES})
  endif()
endif()

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

  add_executable(check_panda_can_codec
    benchmarks/check_panda_can_codec.cc
    src/panda_can_codec.cc
  )
  target_include_directories(check_panda_can_codec PRIVATE ${PROJECT_LOCAL_INCLUDES})
  target_compile_options(check_panda_can_codec PRIVATE ${PROJECT_WARNINGS})

  add_executable(bench_input_warp_overhead
    benchmarks/bench_input_warp_overhead.cc
    src/app_config.cc
    src/model_input_transform.cc
  )
  target_include_directories(bench_input_warp_overhead PRIVATE ${PROJECT_LOCAL_INCLUDES})
  target_compile_options(bench_input_warp_overhead PRIVATE ${PROJECT_WARNINGS})

  add_executable(verify_calibration_equivalence
    benchmarks/verify_calibration_equivalence.cc
    src/app_config.cc
    src/calibration_service.cc
    src/model_input_transform.cc
    src/online_calibrator.cc
    src/projection.cc
  )
  target_include_directories(verify_calibration_equivalence PRIVATE ${PROJECT_LOCAL_INCLUDES})
  target_compile_options(verify_calibration_equivalence PRIVATE ${PROJECT_WARNINGS})

  if(SUPERCOMBO_BUILD_RUNTIME)
    add_executable(bench_capture_nv12 benchmarks/bench_capture_nv12.cc)
    configure_k230_target(bench_capture_nv12 mmz ${K230_SHARED_LIBS})

    add_executable(bench_ai2d_resize benchmarks/bench_ai2d_resize.cc)
    configure_k230_target(bench_ai2d_resize ${NNCASE_STATIC_LIBS} mmz ${K230_SHARED_LIBS})

    add_executable(sequence_runner benchmarks/sequence_runner.cc)
    configure_k230_target(sequence_runner ${NNCASE_STATIC_LIBS} mmz ${K230_SHARED_LIBS})

    add_executable(probe_drm_planes benchmarks/probe_drm_planes.cc)
    configure_k230_target(probe_drm_planes mmz ${K230_SHARED_LIBS})
  endif()
endif()

if(SUPERCOMBO_BUILD_RUNTIME)
  install(TARGETS supercombo_elf k230_camerad k230_modeld k230_overlay
    RUNTIME DESTINATION .
  )
  if(SUPERCOMBO_BUILD_PANDA)
    install(TARGETS k230_pandad RUNTIME DESTINATION .)
  endif()
  install(PROGRAMS k230_manager.py k230_controlsd.py DESTINATION .)
  install(DIRECTORY models DESTINATION . OPTIONAL FILES_MATCHING PATTERN "*.kmodel")
endif()
