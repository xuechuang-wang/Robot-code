# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
# add_executable(app_scout_demo scout_demo/scout_demo.cpp)
# target_link_libraries(app_scout_demo wrp_sdk)

add_executable(app_bunker_demo bunker_demo/bunker_demo.cpp)
target_link_libraries(app_bunker_demo wrp_sdk)

add_executable(app_bunker_CSMC bunker_demo/bunker_CSMC_NDOB.cpp)
target_link_libraries(app_bunker_CSMC wrp_sdk)

add_executable(app_bunker_CTSMC bunker_demo/bunker_CTSMC_SMO.cpp)
target_link_libraries(app_bunker_CTSMC wrp_sdk)

# add_executable(app_scout_discharge scout_demo/scout_discharge.cpp)
# target_link_libraries(app_scout_discharge wrp_sdk)

add_executable(app_hunter_demo hunter_demo/hunter_demo.cpp)
target_link_libraries(app_hunter_demo wrp_sdk)

# add_executable(app_tracer_demo tracer_demo/tracer_demo.cpp)
# target_link_libraries(app_tracer_demo wrp_sdk)

if(BUILD_MONITOR)
    add_subdirectory(scout_monitor)
endif()