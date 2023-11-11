# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

set(CONFIG_MOTORCTRL 1)
set(CONFIG_HPM_ADC 1)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(hpm_foc_test)

file(GLOB SRC_FILES src/*.c)

sdk_app_src(${SRC_FILES})
generate_ses_project()
