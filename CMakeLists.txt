# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adc_respiratory)

target_sources(app PRIVATE src/main.c src/ble_send.c src/adc_control.c src/ppi_config.c src/storage_nvs.c src/battery.c)
