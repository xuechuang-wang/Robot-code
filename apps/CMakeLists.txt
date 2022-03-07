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

# CTSMC+SMO & CTSMC 方形
add_executable(app_bunker_F bunker_demo/bunker_CTSMC_F.cpp)
target_link_libraries(app_bunker_F wrp_sdk)

# ITSMC fang
add_executable(app_bunker_FIT bunker_demo/bunker_ITSMC_F.cpp)
target_link_libraries(app_bunker_FIT wrp_sdk)

# CTSMC+SMO & CTSMC 螺旋
add_executable(app_bunker_CTSMCL bunker_demo/bunker_CTSMO_Luo.cpp)
target_link_libraries(app_bunker_CTSMCL wrp_sdk)

# CTSMC+SMO & CTSMC 双扭线
add_executable(app_bunker_CTSMCSN bunker_demo/bunker_CTSMO_SN.cpp)
target_link_libraries(app_bunker_CTSMCSN wrp_sdk)


# CSMC 圆
add_executable(app_CSMC_Cir bunker_demo/bunker_CSMC_Circle.cpp)
target_link_libraries(app_CSMC_Cir wrp_sdk)


# CSMC 方形
add_executable(app_CSMC_F bunker_demo/bunker_CSMC_F.cpp)
target_link_libraries(app_CSMC_F wrp_sdk)

# CSMC 螺旋
add_executable(app_CSMC_Luo bunker_demo/bunker_CSMC_LuoXuan.cpp)
target_link_libraries(app_CSMC_Luo wrp_sdk)

# CSMC 双扭线
add_executable(app_CSMC_SN bunker_demo/bunker_CSMC_SN.cpp)
target_link_libraries(app_CSMC_SN wrp_sdk)




# add_executable(app_scout_discharge scout_demo/scout_discharge.cpp)
# target_link_libraries(app_scout_discharge wrp_sdk)

# 中文核心
add_executable(app_PI bunker_demo/bunker_PI.cpp)
target_link_libraries(app_PI wrp_sdk)

# 中文核心修改版
add_executable(app_PI_R1 bunker_demo/bunker_demo_PI.cpp)
target_link_libraries(app_PI_R1 wrp_sdk)

add_executable(app_LADRC_R1 bunker_demo/bunker_demo_LADRC.cpp)
target_link_libraries(app_LADRC_R1 wrp_sdk)

add_executable(app_hunter_demo_R1 hunter_demo/hunter_demo.cpp)
target_link_libraries(app_hunter_demo_R1 wrp_sdk)

# add_executable(app_tracer_demo tracer_demo/tracer_demo.cpp)
# target_link_libraries(app_tracer_demo wrp_sdk)

if(BUILD_MONITOR)
    add_subdirectory(scout_monitor)
endif()