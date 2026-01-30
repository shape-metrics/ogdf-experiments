cmake_minimum_required(VERSION 3.16)
project(OGDF_experiments LANGUAGES CXX)

# Do NOT build DOMUS executables
set(DOMUS_BUILD_EXECUTABLES OFF CACHE BOOL "" FORCE)

add_subdirectory(domus)
add_subdirectory(OGDF)

add_executable(ogdf_experiments
    src/ogdf-drawer.cpp
    src/main.cpp
)

target_compile_features(ogdf_experiments PRIVATE cxx_std_23)

target_link_libraries(ogdf_experiments
    PRIVATE DOMUS::core OGDF
)
