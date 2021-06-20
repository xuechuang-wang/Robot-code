# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
# add_executable(app_scout_demo scout_demo/scout_demo.cpp)
# target_link_libraries(app_scout_demo wrp_sdk)

# 例程
add_executable(app_bunker_demo bunker_demo/bunker_demo.cpp)
target_link_libraries(app_bunker_demo wrp_sdk)

# CSMC+NDOB
add_executable(app_bunker_CSMC bunker_demo/bunker_CSMC_NDOB.cpp)
target_link_libraries(app_bunker_CSMC wrp_sdk)

# CTSMC+SMO & CTSMC 圆
add_executable(app_bunker_CTSMC bunker_demo/bunker_CTSMC_SMO.cpp)
target_link_libraries(app_bunker_CTSMC wrp_sdk)

# CTSMC+SMC & CTSMC 螺旋
add_executable(app_bunker_CTSMCL bunker_demo/bunker_CTSMO_Luo.cpp)
target_link_libraries(app_bunker_CTSMCL wrp_sdk)

# CSMC 圆
add_executable(app_CSMC_Cir bunker_demo/bunker_CSMC_Circle.cpp)
target_link_libraries(app_CSMC_Cir wrp_sdk)

# CSMC 螺旋
add_executable(app_CSMC_Luo bunker_demo/bunker_CSMC_LuoXuan.cpp)
target_link_libraries(app_CSMC_Luo wrp_sdk)

# add_executable(app_scout_discharge scout_demo/scout_discharge.cpp)
# target_link_libraries(app_scout_discharge wrp_sdk)

# 中文核心
add_executable(app_PI bunker_demo/bunker_PI.cpp)
target_link_libraries(app_PI wrp_sdk)



add_executable(app_hunter_demo hunter_demo/hunter_demo.cpp)
target_link_libraries(app_hunter_demo wrp_sdk)

# add_executable(app_tracer_demo tracer_demo/tracer_demo.cpp)
# target_link_libraries(app_tracer_demo wrp_sdk)

if(BUILD_MONITOR)
    add_subdirectory(scout_monitor)
endif()