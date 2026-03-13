function(fgo_add_app target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE fgo_ppp)
endfunction()

fgo_add_app(rinex_reader       rinex_reader/main.cpp)
fgo_add_app(nav_reader         nav_reader/main.cpp)
fgo_add_app(satellite_position satellite_position/main.cpp)

# fgo_add_app(ppp_solver  ppp_solver/main.cpp)
# fgo_add_app(rtk_solver  rtk_solver/main.cpp)
# fgo_add_app(rt_receiver rt_receiver/main.cpp)
