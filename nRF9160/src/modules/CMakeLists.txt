#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_include_directories(app PRIVATE .)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/modules_common.c)
target_sources_ifdef(CONFIG_IMU_MODULE app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu_module.c)
target_sources_ifdef(CONFIG_CONTROLLER_MODULE app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/controller_module.c)
target_sources_ifdef(CONFIG_QDEC_MODULE app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/qdec_module.c)