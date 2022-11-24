# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/softwares/ESPIDF/components/bootloader/subproject"
  "E:/EMI_Fusion/ESP-IDF/Attendance_system_DB/build/bootloader"
  "E:/EMI_Fusion/ESP-IDF/Attendance_system_DB/build/bootloader-prefix"
  "E:/EMI_Fusion/ESP-IDF/Attendance_system_DB/build/bootloader-prefix/tmp"
  "E:/EMI_Fusion/ESP-IDF/Attendance_system_DB/build/bootloader-prefix/src/bootloader-stamp"
  "E:/EMI_Fusion/ESP-IDF/Attendance_system_DB/build/bootloader-prefix/src"
  "E:/EMI_Fusion/ESP-IDF/Attendance_system_DB/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/EMI_Fusion/ESP-IDF/Attendance_system_DB/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
