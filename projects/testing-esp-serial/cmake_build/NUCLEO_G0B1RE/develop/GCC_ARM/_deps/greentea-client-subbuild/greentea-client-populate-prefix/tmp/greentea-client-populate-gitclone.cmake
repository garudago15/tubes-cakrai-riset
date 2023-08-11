# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if(EXISTS "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitclone-lastrun.txt" AND EXISTS "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitinfo.txt" AND
  "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitclone-lastrun.txt" IS_NEWER_THAN "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitinfo.txt")
  message(STATUS
    "Avoiding repeated git clone, stamp file is up to date: "
    "'D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitclone-lastrun.txt'"
  )
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-src"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe" 
            clone --no-checkout --config "advice.detachedHead=false" "https://github.com/ARMmbed/greentea-client.git" "greentea-client-src"
    WORKING_DIRECTORY "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps"
    RESULT_VARIABLE error_code
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once: ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/ARMmbed/greentea-client.git'")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe" 
          checkout "472aad2327fbfde827852fc44775904706847a3a" --
  WORKING_DIRECTORY "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-src"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: '472aad2327fbfde827852fc44775904706847a3a'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe" 
            submodule update --recursive --init 
    WORKING_DIRECTORY "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-src"
    RESULT_VARIABLE error_code
  )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitinfo.txt" "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitclone-lastrun.txt'")
endif()
