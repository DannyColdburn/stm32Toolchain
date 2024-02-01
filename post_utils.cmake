message(STATUS "${CMAKE_PROJECT_NAME} Heheheeh:")

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

macro(generate_binary_files)
  add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME} POST_BUILD 
    COMMAND ${CMAKE_OBJCOPY} ARGS -O binary ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_PROJECT_NAME}.bin
    )


endmacro()

