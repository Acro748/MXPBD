cmake_minimum_required(VERSION 3.21)
message("Using toolchain file ${CMAKE_TOOLCHAIN_FILE}.")

########################################################################################################################
## Define project
########################################################################################################################
project(
        MXPBD
        VERSION 0.0.2
        DESCRIPTION "Mu Extended Position Based Dynamics by Acro"
        LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)

include(GNUInstallDirs)

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/version.rc
        @ONLY
)

set(headers
        include/MXPBD/ConvexHull.h
        include/MXPBD/DynamicAABBTree.h
        include/MXPBD/PhysicsConfig.h
        include/MXPBD/PhysicsWorld.h
        include/MXPBD/PhysicsWorldCommon.h
        include/MXPBD/PhysicsWorldSystem.h
        include/MXPBD/PhysicsWorldUpdater.h
        include/Command.h
        include/Condition.h
        include/Config.h
        include/Hook.h
        include/lString.h
        include/nifUtil.h
        include/Store.h
        include/Utility.hpp
)

set(sources
        src/MXPBD/ConvexHull.cpp
        src/MXPBD/DynamicAABBTree.cpp
        src/MXPBD/PhysicsConfig.cpp
        src/MXPBD/PhysicsWorld.cpp
        src/MXPBD/PhysicsWorldSystem.cpp
        src/MXPBD/PhysicsWorldUpdater.cpp
        src/Command.cpp
        src/Condition.cpp
        src/Config.cpp
        src/Hook.cpp
        src/Main.cpp
        src/nifUtil.cpp
        src/Store.cpp

        ${CMAKE_CURRENT_BINARY_DIR}/version.rc
)

source_group(
        TREE ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
        ${headers}
        ${sources})

########################################################################################################################
## Configure target DLL
########################################################################################################################
find_package(CommonLibSSE REQUIRED)
find_package(Directxtk CONFIG REQUIRED)
find_package(detours)
find_package(tinyxml2 REQUIRED)
find_package(magic_enum CONFIG REQUIRED)
find_package(xxhash CONFIG REQUIRED)
find_package(meshoptimizer CONFIG REQUIRED)
find_package(unordered_dense CONFIG REQUIRED)
find_package(tbb CONFIG REQUIRED)

add_commonlibsse_plugin(${PROJECT_NAME} SOURCES ${headers} ${sources})
add_library("${PROJECT_NAME}::${PROJECT_NAME}" ALIAS "${PROJECT_NAME}")

target_compile_features(
    ${PROJECT_NAME}
    PUBLIC
        cxx_std_23
)

target_compile_definitions(
    ${PROJECT_NAME}
    PUBLIC
        SKSE_SUPPORT_XBYAK
        _UNICODE
        UNICODE
)

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/src>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extern/quickhull>
        ${DETOURS_INCLUDE_DIRS}
)

file(GLOB QUICKHULL_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/quickhull/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/quickhull/*.hpp
)
add_library(quickhull STATIC ${QUICKHULL_SRC})
target_include_directories(quickhull PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/quickhull
)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
        CommonLibSSE::CommonLibSSE
    PRIVATE
        tinyxml2::tinyxml2
        magic_enum::magic_enum
        xxHash::xxhash
        quickhull
        meshoptimizer::meshoptimizer
        unordered_dense::unordered_dense
        TBB::tbb
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/vcpkg_installed/x64-windows-static-md/lib/detours.lib>
)

target_precompile_headers(
    ${PROJECT_NAME}
    PRIVATE
        include/PCH.h)
        
target_compile_options(${PROJECT_NAME} PRIVATE "$<$<CONFIG:Release>:/sdl;/utf-8;/Zi;/permissive-;/Zc:preprocessor;/Zc:inline;/JMC-;/O2;/Ob3;/arch:AVX>")
target_link_options(${PROJECT_NAME} PRIVATE "$<$<CONFIG:Release>:/INCREMENTAL:NO;/OPT:REF;/OPT:ICF;/DEBUG:FULL>")

install(TARGETS ${PROJECT_NAME}
        DESTINATION "${CMAKE_INSTALL_LIBDIR}")


########################################################################################################################
## Automatic plugin deployment
########################################################################################################################
# Automatic deployment to FOMOD directory.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(BUILD_NAME "Debug")
else()
    set(BUILD_NAME "Release")
endif()
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_SOURCE_DIR}/contrib/Distribution/Plugin${BUILD_NAME}/")
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> "${CMAKE_CURRENT_SOURCE_DIR}/contrib/Distribution/Plugin${BUILD_NAME}/")
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_PDB_FILE:${PROJECT_NAME}> "${CMAKE_CURRENT_SOURCE_DIR}/contrib/Distribution/Plugin${BUILD_NAME}/")

file(GLOB_RECURSE OUTPUT_DLLS "${CMAKE_CURRENT_SOURCE_DIR}/contrib/Distribution/**/*.dll")
file(GLOB_RECURSE OUTPUT_PDBS "${CMAKE_CURRENT_SOURCE_DIR}/contrib/Distribution/**/*.pdb")
set_property(TARGET ${PROJECT_NAME}
        APPEND PROPERTY ADDITIONAL_CLEAN_FILES "${OUTPUT_DLLS}")
set_property(TARGET ${PROJECT_NAME}
        APPEND PROPERTY ADDITIONAL_CLEAN_FILES "${OUTPUT_PDBS}")
