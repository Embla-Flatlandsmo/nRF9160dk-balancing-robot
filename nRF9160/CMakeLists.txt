# #
# # Copyright (c) 2019 Nordic Semiconductor ASA
# #
# # SPDX-License-Identifier: Apache-2.0
# #

# cmake_minimum_required(VERSION 3.20.0)


# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(balancing-robot)

# FILE(GLOB app_sources src/*.c)
# target_sources(app PRIVATE ${app_sources})


# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

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