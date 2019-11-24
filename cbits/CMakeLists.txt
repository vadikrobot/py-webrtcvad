cmake_minimum_required(VERSION 3.1)
project(webrtc)

# Enable C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")
endif()


find_package(Threads REQUIRED)

##################################

SET(signal_processing_SOURCE
    webrtc/common_audio/signal_processing/complex_bit_reverse.c
    webrtc/common_audio/signal_processing/complex_fft.c
    webrtc/common_audio/signal_processing/cross_correlation.c
    webrtc/common_audio/signal_processing/division_operations.c
    webrtc/common_audio/signal_processing/downsample_fast.c
    webrtc/common_audio/signal_processing/energy.c
    webrtc/common_audio/signal_processing/get_scaling_square.c
    webrtc/common_audio/signal_processing/min_max_operations.c
    webrtc/common_audio/signal_processing/resample_48khz.c
    webrtc/common_audio/signal_processing/resample_by_2_internal.c
    webrtc/common_audio/signal_processing/resample_fractional.c
    webrtc/common_audio/signal_processing/spl_init.c
    webrtc/common_audio/signal_processing/spl_inl.c
    webrtc/common_audio/signal_processing/spl_sqrt.c
    webrtc/common_audio/signal_processing/vector_scaling_operations.c
    webrtc/common_audio/third_party/spl_sqrt_floor/spl_sqrt_floor.c
    webrtc/common_audio/vad/vad_core.c
    webrtc/common_audio/vad/vad_filterbank.c
    webrtc/common_audio/vad/vad_gmm.c
    webrtc/common_audio/vad/vad_sp.c
    webrtc/common_audio/vad/webrtc_vad.c
    webrtc/common_audio/audio_util.cc
    webrtc/common_audio/vad/vad.cc
    webrtc/common_audio/wav_file.cc
    webrtc/common_audio/wav_header.cc
    webrtc/rtc_base/checks.cc
    webrtc/rtc_base/system/file_wrapper.cc
    webrtc/rtc_base/critical_section.cc
    webrtc/rtc_base/platform_thread.cc
    webrtc/rtc_base/platform_thread_types.cc
)

add_library(${PROJECT_NAME} ${signal_processing_SOURCE})
target_link_libraries(${PROJECT_NAME} Threads::Threads)

target_compile_definitions(${PROJECT_NAME} PUBLIC WEBRTC_POSIX)
target_compile_definitions(${PROJECT_NAME} PUBLIC WEBRTC_LINUX)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<INSTALL_INTERFACE:include>    
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/webrtc/common_audio/vad/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/webrtc/common_audio/signal_processing/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/webrtc/common_audio>
        ${CMAKE_CURRENT_SOURCE_DIR}
)
