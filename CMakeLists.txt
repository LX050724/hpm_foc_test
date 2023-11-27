# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)
set(SES_TOOLCHAIN_VARIANT "Andes")

set(CONFIG_HPM_MATH 1)
set(CONFIG_HPM_MATH_DSP 1)
set(HPM_MATH_DSP_SES_LIB "libdspf")
set(CONFIG_MOTORCTRL 1)
set(CONFIG_HPM_ADC 1)
set(CONFIG_CHERRYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_USB_DEVICE_CDC 1)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})
project(hpm_foc_test)
sdk_ses_compile_options(-mabi=ilp32f)
sdk_ses_compile_options(-march=rv32imafc)
sdk_ses_compile_options(-O3)

sdk_inc(src/usb)

file(GLOB_RECURSE SRC_FILES src/*.c)

sdk_app_src(${SRC_FILES})
generate_ses_project()
