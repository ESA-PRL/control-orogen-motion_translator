PROJECT(motion_translator)
cmake_minimum_required(VERSION 2.6)

add_definitions(-Wall)
add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME})
add_definitions(-DBASE_LOG_DEBUG)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(motion_translatorBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)
