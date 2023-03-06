# Copyright (c) 2023 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources(app PRIVATE
  app_mpu.c
  eulerconversion.c
  inv_mpu_dmp.c
  inv_mpu.c
  twi.c)

target_include_directories(app PRIVATE include)
