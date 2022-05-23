# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/joachimls/esp/esp-idf/components/bootloader/subproject"
  "/home/joachimls/esp/Instability-ESP32/build/bootloader"
  "/home/joachimls/esp/Instability-ESP32/build/bootloader-prefix"
  "/home/joachimls/esp/Instability-ESP32/build/bootloader-prefix/tmp"
  "/home/joachimls/esp/Instability-ESP32/build/bootloader-prefix/src/bootloader-stamp"
  "/home/joachimls/esp/Instability-ESP32/build/bootloader-prefix/src"
  "/home/joachimls/esp/Instability-ESP32/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/joachimls/esp/Instability-ESP32/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
