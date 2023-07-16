cmake_minimum_required(VERSION 3.0)
project(PBF)

option(GLFW_BUILD_DOCS OFF)
option(GLFW_BUILD_EXAMPLES OFF)
option(GLFW_BUILD_TESTS OFF)
add_subdirectory(PBF/Vendor/glfw)

option(ASSIMP_BUILD_ASSIMP_TOOLS OFF)
option(ASSIMP_BUILD_SAMPLES OFF)
option(ASSIMP_BUILD_TESTS OFF)
add_subdirectory(PBF/Vendor/assimp)

option(BUILD_CPU_DEMOS OFF)
option(BUILD_EXTRAS OFF)
option(BUILD_OPENGL3_DEMOS OFF)
option(BUILD_UNIT_TESTS OFF)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -std=c++11")
    if(NOT WIN32)
        set(GLAD_LIBRARIES dl)
    endif()
endif()

include_directories(PBF/Headers/
                    PBF/imgui/
                    PBF/Vendor/assimp/include/
                    
                    PBF/Vendor/glad/include/
                    PBF/Vendor/glfw/include/
                    PBF/Vendor/glm/
                    PBF/Vendor/stb/)

file(GLOB VENDORS_SOURCES PBF/Vendor/glad/src/glad.c)
file(GLOB PROJECT_HEADERS PBF/Headers/*.hpp)
file(GLOB PROJECT_SOURCES PBF/Sources/*.cpp)
file(GLOB PROJECT_SHADERS PBF/Shaders/*.comp
                          PBF/Shaders/*.frag
                          PBF/Shaders/*.geom
                          PBF/Shaders/*.vert)
file(GLOB IMGUI PBF/imgui/*.h
				PBF/imgui/backends/*.h
                PBF/imgui/*.cpp
				PBF/imgui/backends/*.cpp)
file(GLOB PROJECT_ASSETS PBF/Assets/*)
file(GLOB PROJECT_CONFIGS CMakeLists.txt
                          Readme.md
                         .gitattributes
                         .gitignore
                         .gitmodules)

source_group("Headers" FILES ${PROJECT_HEADERS})
source_group("Shaders" FILES ${PROJECT_SHADERS})
source_group("Assets"  FILES ${PROJECT_ASSETS})
source_group("Sources" FILES ${PROJECT_SOURCES})
source_group("Vendors" FILES ${VENDORS_SOURCES})
source_group("Imgui" FILES ${IMGUI})

add_definitions(-DGLFW_INCLUDE_NONE
                -DPROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")
add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${PROJECT_HEADERS}
                               ${PROJECT_SHADERS} ${PROJECT_CONFIGS} ${IMGUI}
                               ${VENDORS_SOURCES})
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/PBF")
target_link_libraries(${PROJECT_NAME} assimp glfw
                      ${GLFW_LIBRARIES} ${GLAD_LIBRARIES}
                      )
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PROJECT_NAME})

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/PBF/Shaders $<TARGET_FILE_DIR:${PROJECT_NAME}>/Shaders
    DEPENDS ${PROJECT_SHADERS})

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/PBF/Assets $<TARGET_FILE_DIR:${PROJECT_NAME}>/Assets
    DEPENDS ${PROJECT_ASSETS})
