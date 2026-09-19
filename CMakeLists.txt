cmake_minimum_required(VERSION 3.16)
project(sensorhub LANGUAGES C)

# Layout note: sources live under src/sensorhub/ rather than the usual
# include/ + src/ split so that this directory is simultaneously a valid
# Arduino library. arduino-cli puts <library>/src on the include path and
# compiles everything beneath it, which makes <sensorhub/sensorhub.h> resolve
# identically on the Nano and on the Pi. src/sensorhub/serial.c is guarded so
# it compiles away to nothing on AVR.
add_library(sensorhub STATIC
    src/sensorhub/parser.c
)

target_include_directories(sensorhub
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_features(sensorhub PUBLIC c_std_11)

# The POSIX serial transport. Guarded internally too, but there is no reason
# to hand it to a non-Unix build at all.
if(UNIX)
    target_sources(sensorhub PRIVATE src/sensorhub/serial.c)
endif()

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(SH_TOP_LEVEL ON)
else()
    set(SH_TOP_LEVEL OFF)
endif()

option(SH_BUILD_MONITOR "Build the sensorhub-monitor bench tool" ${SH_TOP_LEVEL})
option(SH_BUILD_TESTS   "Build the framing tests"                ${SH_TOP_LEVEL})

if(SH_BUILD_MONITOR AND UNIX)
    add_executable(sensorhub-monitor tools/monitor.c)
    target_link_libraries(sensorhub-monitor PRIVATE sensorhub)
endif()

if(SH_BUILD_TESTS)
    enable_testing()
    add_executable(test_parser test/test_parser.c)
    target_link_libraries(test_parser PRIVATE sensorhub)
    add_test(NAME parser COMMAND test_parser)

    # The counter width is configurable for AVR's sake; build the tests
    # against the narrow setting too so that path cannot rot unnoticed.
    #
    # This compiles parser.c into the test rather than linking libsensorhub,
    # deliberately: SH_COUNTER_BITS changes sizeof(sh_stats_t), so a test
    # defining 16 while the library was built with 64 would disagree about
    # the layout of every sh_parser_t it passed across that boundary.
    add_executable(test_parser_16 test/test_parser.c src/sensorhub/parser.c)
    target_include_directories(test_parser_16 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_compile_definitions(test_parser_16 PRIVATE SH_COUNTER_BITS=16)
    add_test(NAME parser_narrow_counters COMMAND test_parser_16)
endif()
