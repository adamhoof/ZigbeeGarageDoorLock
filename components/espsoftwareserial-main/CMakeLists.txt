cmake_minimum_required(VERSION 3.16)

idf_component_register(
        SRCS "src/SoftwareSerial.cpp"
        INCLUDE_DIRS "src"
        REQUIRES arduino-esp32
)
