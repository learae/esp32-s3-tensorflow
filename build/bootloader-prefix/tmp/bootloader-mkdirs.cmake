# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/vl/esp32/esp-idf/components/bootloader/subproject"
  "/home/vl/esp32/project_my/esp32s3_lvgl/build/bootloader"
  "/home/vl/esp32/project_my/esp32s3_lvgl/build/bootloader-prefix"
  "/home/vl/esp32/project_my/esp32s3_lvgl/build/bootloader-prefix/tmp"
  "/home/vl/esp32/project_my/esp32s3_lvgl/build/bootloader-prefix/src/bootloader-stamp"
  "/home/vl/esp32/project_my/esp32s3_lvgl/build/bootloader-prefix/src"
  "/home/vl/esp32/project_my/esp32s3_lvgl/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/vl/esp32/project_my/esp32s3_lvgl/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/vl/esp32/project_my/esp32s3_lvgl/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
