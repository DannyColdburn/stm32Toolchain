#This is an example of CMakeLists.txt 
cmake_minimum_required(VERSION 3.20)
#Before adding toolchain file we should set a few parameters
#Set your compiler triplet
set(TRIPLET_PREFIX "arm-none-eabi")
#Set stm device you working on
set(STM_DEVICE "STM32F411CEU6")

#Then, set toolchain file
set(CMAKE_TOOLCHAIN_FILE "path to toolchain.cmake")

#Adding project, after project name you set languages that will be used
project(projectName C CXX ASM)

#Sometimes libraries requires some #define to change it behaviour
add_compile_definitions(DEBUG ANOTHERDEFINE ...)

#to add libraries you can do this. LIBS_DIR specified in user_path.cmake
add_subdirectory(${LIBS_DIR}/nameOfLib ${CMAKE_CURRENT_BINARY_DIR}/nameOfLib)

#then we add executable to our target to compile 
#STARTUP_FILE is .c file that gets selected by toolchain
add_executable(${PROJECT_NAME} main.c ${STARTUP_FILE} somefile.c somefile2.c ...)

#When working with header files you should set directories where compiler will seek those files
target_include_directories(
  ${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include #Will seek in project root/include
  )

#Then we link libraries to this projects
target_link_libraries(${PROJECT_NAME} nameOfLib)

#After all we will need to get our .bin 
generate_binary_files()


