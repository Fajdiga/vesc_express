# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "6.0") 

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Derive project name from HW_HEADER if set, otherwise use default hardware name
if(DEFINED ENV{HW_HEADER})
    # Extract hardware name from path (e.g., "hw_jfbms_slave.h" -> "jfbms_slave")
    get_filename_component(HW_FILENAME "$ENV{HW_HEADER}" NAME_WE)
    string(REGEX REPLACE "^hw_" "" ProjectId ${HW_FILENAME})
else()
    # Default hardware is jfbms_slave (see main/CMakeLists.txt)
    set(ProjectId "jfbms_slave")
endif()
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})

include(cmake/git_rev_parse.cmake)

git_describe(GIT_COMMIT_HASH ".")
# get_git_head_revision(GIT_COMMIT_REF GIT_COMMIT_HASH ".")

if(DEFINED ENV{GIT_COMMIT_HASH})
    set(GIT_COMMIT_HASH $ENV{GIT_COMMIT_HASH})
else()
    git_describe(GIT_COMMIT_HASH ".")
endif()

if(DEFINED ENV{GIT_BRANCH_NAME})
    set(GIT_BRANCH_NAME $ENV{GIT_BRANCH_NAME})
else()
    git_rev_parse(GIT_BRANCH_NAME "." "--abbrev-ref" "HEAD")
endif()

idf_build_set_property(COMPILE_DEFINITIONS "GIT_COMMIT_HASH=\"${GIT_COMMIT_HASH}\"" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "GIT_BRANCH_NAME=\"${GIT_BRANCH_NAME}\"" APPEND)

if(DEFINED ENV{USER_GIT_COMMIT_HASH})
    idf_build_set_property(COMPILE_DEFINITIONS "USER_GIT_COMMIT_HASH=\"$ENV{USER_GIT_COMMIT_HASH}\"" APPEND)
endif()
if(DEFINED ENV{USER_GIT_BRANCH_NAME})
    idf_build_set_property(COMPILE_DEFINITIONS "USER_GIT_BRANCH_NAME=\"$ENV{USER_GIT_BRANCH_NAME}\"" APPEND)
endif()

idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=calloc-transposed-args" APPEND)


add_custom_target(rename_bin ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Running post_build script..."
    COMMAND ${CMAKE_COMMAND} -E env python ${CMAKE_SOURCE_DIR}/post_build.py
        ${CMAKE_BINARY_DIR} ${PROJECT_NAME}
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Renaming firmware binary after full build"
    VERBATIM
)
