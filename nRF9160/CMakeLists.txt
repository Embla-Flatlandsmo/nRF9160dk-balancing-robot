# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)
set(BOARD nrf9160dk_nrf9160_ns)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project("Balancing Robot")

# Application sources
# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)
# NORDIC SDK APP END

# Include application events and configuration headers
zephyr_library_include_directories(
  src/events
  src/util
  )

# Include source directory
target_include_directories(app PRIVATE src)

# Application directories
add_subdirectory(src/modules)
add_subdirectory(src/events)
add_subdirectory(src/util)
add_subdirectory(drivers)