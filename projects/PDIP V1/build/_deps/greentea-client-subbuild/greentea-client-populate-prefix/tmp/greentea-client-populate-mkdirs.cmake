# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/URO-mbed-os/tubes-cakrai-riset/projects/PDIP V1/build/_deps/greentea-client-src"
  "D:/URO-mbed-os/tubes-cakrai-riset/projects/PDIP V1/build/_deps/greentea-client-build"
  "D:/URO-mbed-os/tubes-cakrai-riset/projects/PDIP V1/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix"
  "D:/URO-mbed-os/tubes-cakrai-riset/projects/PDIP V1/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/tmp"
  "D:/URO-mbed-os/tubes-cakrai-riset/projects/PDIP V1/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp"
  "D:/URO-mbed-os/tubes-cakrai-riset/projects/PDIP V1/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src"
  "D:/URO-mbed-os/tubes-cakrai-riset/projects/PDIP V1/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp"
)

set(configSubDirs Debug)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/URO-mbed-os/tubes-cakrai-riset/projects/PDIP V1/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/URO-mbed-os/tubes-cakrai-riset/projects/PDIP V1/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
