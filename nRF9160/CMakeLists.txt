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

# Include application event headers
zephyr_library_include_directories(src/events)

# Application sources
# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)

target_sources(app PRIVATE
		src/events/ack_event.c
		src/events/config_event.c
		src/events/control_event.c
        src/events/imu_module_event.c
		src/events/measurement_event.c
)

target_sources(app PRIVATE
		src/modules/controller.c
		# src/modules/sensor_simulated.c
        src/modules/imu_module.c
		src/modules/stats.c
)
# NORDIC SDK APP END
