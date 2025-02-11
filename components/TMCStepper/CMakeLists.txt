# components/TMCStepper/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Gather all C/C++ files under the src folder (you may also add other extensions if needed)
file(GLOB_RECURSE TMC_SOURCES "src/source/*.cpp" "src/source/*.c")

idf_component_register(
        SRCS ${TMC_SOURCES}
        INCLUDE_DIRS "src" "src/source"
        REQUIRES arduino-esp32
)