# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(__PROJECT_GROUP_LINK_COMPONENTS ON)

set(PLANE_COMPONENT_DIRS "./components/core"
                        "./components/drivers"
                        "./components/drivers/general"
                        "./components/drivers/i2c_devices"
                        "./components/drivers/spi_devices"
                         "./components/lib" )

set(EXTRA_COMPONENT_DIRS "${EXTRA_COMPONENT_DIRS} ${PLANE_COMPONENT_DIRS}")

# The app version (USB 'status' fw=, the image's app description). IDF takes PROJECT_VER, else a version.txt here,
# else 'git describe' of this directory, else "1" (02cbbe0 flew as fw=1: its build ran from a copy of the tree).
# A copy (no .git) gets a version.txt written into it before the build:
#   git -C <tree> describe --always --dirty > <copy>/version.txt
# IDF re-runs the configure when version.txt changes, so a reused build directory follows it. A
# -D PROJECT_VER=<sha> is used for its own configure only (taken out of the cache here): a later configure without
# it falls back to version.txt / git and never keeps an old commit's sha. With none of the three the configure stops.
if(DEFINED CACHE{PROJECT_VER})
  set(espdrone_ver "${PROJECT_VER}")
  unset(PROJECT_VER CACHE)
  set(PROJECT_VER "${espdrone_ver}")
elseif(NOT DEFINED PROJECT_VER)
  execute_process(COMMAND git describe --always --dirty
                  WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
                  OUTPUT_VARIABLE espdrone_git_ver OUTPUT_STRIP_TRAILING_WHITESPACE
                  RESULT_VARIABLE espdrone_git_rc ERROR_QUIET)
  if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/version.txt")
    file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/version.txt" espdrone_txt_ver)
    if(espdrone_git_rc EQUAL 0 AND NOT espdrone_git_ver STREQUAL "" AND NOT espdrone_txt_ver STREQUAL espdrone_git_ver)
      message(FATAL_ERROR "version.txt (${espdrone_txt_ver}) in a git tree at ${espdrone_git_ver}: IDF would take the "
                          "file. Delete it (it is for copies of the tree only).")
    endif()
  elseif(NOT espdrone_git_rc EQUAL 0 OR espdrone_git_ver STREQUAL "")
    message(FATAL_ERROR "No git here, no version.txt and no PROJECT_VER: the app version would read \"1\" "
                        "(status fw=1). Write one into the copy: git -C <tree> describe --always --dirty > "
                        "${CMAKE_CURRENT_LIST_DIR}/version.txt")
  endif()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESPDrone)
