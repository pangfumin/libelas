# project
cmake_minimum_required (VERSION 2.6)
project (libelas)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -msse3")
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)



find_package(PCL REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package (Glog)


# directories
set (LIBELAS_SRC_DIR src)

# include directory
include_directories("${LIBELAS_SRC_DIR}"
        utility
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        )



# sources
FILE(GLOB LIBELAS_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

add_library(elas
        ${LIBELAS_SRC_FILES})

FILE(GLOB UTILITY_SRC_FILES "utility/*.cpp")

add_library(utility
        ${UTILITY_SRC_FILES})


# build demo program
add_executable(main app/main.cpp)
target_link_libraries(main elas)

add_executable(kitti app/kitti.cpp)
target_link_libraries(kitti
        elas
        utility
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})



