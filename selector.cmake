if("${STM_DEVICE}" STREQUAL "")
  message(STATUS "Lol empty")
else()
  string(SUBSTRING "${STM_DEVICE}" 5 4 STM_FAMILY)
endif()
message(STATUS "Selected device: ${STM_DEVICE}")
message(STATUS "Device family: ${STM_FAMILY}")

set(DEVICE_FILES ${CMAKE_CURRENT_LIST_DIR}/${STM_FAMILY})
message(STATUS "Device files directory: ${DEVICE_FILES}")

if("${STM_FAMILY}" STREQUAL "F411")
  set(CMSIS_DEVICE_DIR ${CMSIS_DIR}/cmsis_device_f4/Include)
  message(STATUS "Device includes: ${CMSIS_DEVICE_DIR}")
elseif("${STM_FAMILY}" STREQUAL "F103")
  set(CMSIS_DEVICE_DIR ${CMSIS_DIR}/cmsis_device_f1/Include)
else()
  message(STATUS "Not yet implemented")
endif()



set(CMSIS_CORE_DIR ${CMSIS_DIR}/cmsis_core/Core/Include/)
