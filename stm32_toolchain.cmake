# This is main toolchain file, from here we set all anothers
# First of all global sets 

message(STATUS "Configuring your embedded toolchain")
if(STM_DEVICE STREQUAL "")
  message(FATAL_ERROR "STM_DEVICE is not set")
endif()

if(TRIPLET_PREFIX STREQUAL "")
  message(FATAL_ERROR "TRIPLET_PREFIX is not set")
endif()

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER ${TRIPLET_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TRIPLET_PREFIX}-g++)
set(CMAKE_ASM_COMPILER ${TRIPLET_PREFIX}-as)
set(CMAKE_OBJDUMP ${TRIPLET_PREFIX}-objdump)
set(CMAKE_READELF ${TRIPLET_PREFIX}-readelf)
set(CMAKE_OBJCOPY ${TRIPLET_PREFIX}-objcopy)
set(CMAKE_SIZEUTIL ${TRIPLET_PREFIX}-size)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

include(${CMAKE_CURRENT_LIST_DIR}/user_path.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/selector.cmake)

add_compile_options(
  -mcpu=cortex-m4
  -mthumb
  -mfloat-abi=hard
  -mfpu=fpv4-sp-d16
  -ffunction-sections
  -O0
  -Wall
  -g
  -pedantic
  )

add_link_options(
  -T ${DEVICE_FILES}/linker.ld
  -mthumb
  -mcpu=cortex-m4
  -mfloat-abi=hard
  -mfpu=fpv4-sp-d16
  -nostartfiles
  --specs=nano.specs
  --specs=nosys.specs
  -Wl,--gc-sections,--print-memory-usage
  )

include_directories(
  ${ARM_COMPILER_DIR}/
  ${CMSIS_DEVICE_DIR}/
  ${CMSIS_CORE_DIR}/
  )


set(STARTUP_FILE ${DEVICE_FILES}/startup.c)

set(CMAKE_PROJECT_${CMAKE_PROJECT_NAME}_INCLUDE ${CMAKE_CURRENT_LIST_DIR}/post_utils.cmake)



