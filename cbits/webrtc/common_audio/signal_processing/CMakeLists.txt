cmake_minimum_required(VERSION 3.1)
project(asio-calc)

set(asio-calc-build_tests OFF CACHE BOOL "Whether to build tests")

# Enable C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")
endif()

##################################



SET(signal_processing_SOURCE
    include/real_fft.h
    include/signal_processing_library.h
    include/spl_inl.h

    complex_fft_tables.h
    dot_product_with_scale.h
    resample_by_2_internal.h


    complex_bit_reverse.c
    complex_fft.c
    complex_fft_tables.h
    cross_correlation.c
    division_operations.c
    dot_product_with_scale.cc
    dot_product_with_scale.h
    downsample_fast.c
    energy.c
    get_scaling_square.c
    include
    min_max_operations.c
    resample_48khz.c
    resample_by_2_internal.c
    resample_by_2_internal.h
    resample_fractional.c
    spl_init.c
    spl_inl.c
    spl_sqrt.c
    vector_scaling_operations.c
)

add_library(webrtc STATIC ${signal_processing_SOURCE})
target_include_directories(webrtc PUBLIC include/)