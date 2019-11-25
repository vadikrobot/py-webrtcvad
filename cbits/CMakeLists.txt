cmake_minimum_required(VERSION 3.1)
project(webrtc)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-stack-protector")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-stack-protector")

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -msse -msse2 -msse3")
set(CMAKE_C_FLAGS_RELEASE "-O3 -msse -msse2 -msse3")

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

set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

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

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "webrtc/common_audio/vad/include/webrtc_vad.h;webrtc/common_audio/vad/include/vad.h;webrtc/common_audio/wav_file.h")

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Config
        LIBRARY DESTINATION lib ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION lib ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION include/webrtc
        )

# install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include)
