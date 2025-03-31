cmake_minimum_required(VERSION 3.10)
project(legged_unitree_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        legged_hw
)
# if (NOT DEFINED ARCH)
#     message(FATAL_ERROR "ARCH is not defined! Set it before linking the library.")
# endif()
# message(STATUS "Using ARCH=${ARCH}")

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(lib)
link_directories(${CMAKE_SOURCE_DIR}/utils/faceLightSDK_Nano/lib)
include_directories(include 
                   ${CMAKE_SOURCE_DIR}/utils/faceLightSDK_Nano/include)

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  SET(ARCH amd64)
else()
  SET(ARCH arm64)
endif()
SET(EXTRA_LIBS 
    -pthread 
    libfaceLight_SDK_${ARCH}.so
)

set(SOURCE src/legged_unitree_hw.cpp src/UnitreeHW.cpp)

add_executable(${PROJECT_NAME}_3_8_0
        ${SOURCE}
)

target_compile_definitions(${PROJECT_NAME}_3_8_0 PRIVATE UNITREE_SDK_3_8_0)

target_link_libraries(${PROJECT_NAME}_3_8_0
        ${catkin_LIBRARIES}
        libunitree_legged_sdk_amd64_3_8_0.a
        ${CMAKE_SOURCE_DIR}/utils/faceLightSDK_Nano/lib/libfaceLight_SDK_${ARCH}.so
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}_3_8_0
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CF_WERROR
    )

endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(
  DIRECTORY include ${CMAKE_SOURCE_DIR}/utils/unitree_legged_sdk/include ${CMAKE_SOURCE_DIR}/utils/faceLightSDK_Nano/include
  DESTINATION include
)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/faceLightSDK_Nano/lib/libfaceLight_SDK_${ARCH}.so
        DESTINATION lib)
