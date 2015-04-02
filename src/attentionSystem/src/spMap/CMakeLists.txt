PROJECT(spMap)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

FILE(GLOB folder_header include/iCub/*.h)
FILE(GLOB folder_source src/*.cpp)

ADD_LIBRARY(spMap ${folder_source} ${folder_header})

INSTALL_TARGETS(/lib spMap)
