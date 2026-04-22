cmake_minimum_required(VERSION 3.20)
project(SDR_Project VERSION 0.1.0)

string(TIMESTAMP dateBuilt "%Y-%m-%d %H:%M:%S" UTC)
execute_process(COMMAND git rev-parse --short HEAD WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" OUTPUT_VARIABLE GitCI OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin")

# --- SDR ---
add_executable(sdr src/sdr.cpp src/ElasticReceiver.cpp src/ChannelProcessor.cpp src/NCO.cpp)
target_include_directories(sdr PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/inc")
target_link_libraries(sdr PRIVATE ws2_32)

# --- Collector (MODIFIED SECTION) ---
add_executable(collector src/collector.cpp)
target_include_directories(collector PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/inc")
target_link_libraries(collector PRIVATE "${FTDI_LIB}" ws2_32)

# Define the absolute path manually and PRINT IT for verification
set(ABS_FTDI_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/FTD2XX.lib")
message(STATUS "CHECKING LIB PATH: ${ABS_FTDI_PATH}")

# Force the linker to use the absolute path by wrapping it in quotes
target_link_libraries(collector PRIVATE "${ABS_FTDI_PATH}")

# --- Relay Server ---
add_executable(relay_server src/relay_server.cpp)
target_include_directories(relay_server PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/inc")
target_link_libraries(relay_server PRIVATE ws2_32)

function(set_sdr_metadata target_name major minor patch)
    target_compile_definitions(${target_name} PRIVATE 
        MAJOR_VERSION=${major} MINOR_VERSION=${minor} PATCH_VERSION=${patch}
        APP_NAME="${target_name}" CURRENT_HASH="${GitCI}" CURRENT_DATE="${dateBuilt}"
    )
endfunction()

set_sdr_metadata(sdr 0 1 0)
set_sdr_metadata(collector 0 1 0)
set_sdr_metadata(relay_server 0 1 0)