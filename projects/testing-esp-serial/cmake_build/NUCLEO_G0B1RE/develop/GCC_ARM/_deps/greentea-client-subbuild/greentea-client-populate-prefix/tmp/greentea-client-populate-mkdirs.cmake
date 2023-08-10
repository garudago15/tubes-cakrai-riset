# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-src"
  "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-build"
  "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix"
  "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/tmp"
  "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp"
  "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src"
  "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/2. Programming and Project/URO-mbed-os/tubes-cakrai-riset/projects/testing-esp-serial/cmake_build/NUCLEO_G0B1RE/develop/GCC_ARM/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
