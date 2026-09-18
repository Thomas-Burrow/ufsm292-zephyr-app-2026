# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

# Unit test: built for the `unit_testing` pseudo-board, i.e. a plain host
# executable. No Zephyr kernel, no devicetree, no drivers are linked in.
cmake_minimum_required(VERSION 3.28.0)

# Must be set before find_package(): the unittest component creates the
# `testbinary` target from this list.
set(SOURCES
  src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/radio/sensor_frame.c
)

find_package(Zephyr COMPONENTS unittest REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app_lib_radio_test)

# The codec is pure C. Compile the real translation unit straight into the
# test binary so the test exercises exactly the production code, no copies.
target_include_directories(testbinary PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
)
