add_library(gnss_input OBJECT)

target_sources(gnss_input PRIVATE
    rinex/RinexObsParser.cpp
    rinex/RinexNavParser.cpp
)

target_include_directories(gnss_input
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gnss_input
    PUBLIC
        Eigen3::Eigen
        spdlog::spdlog
        gnss_time
        fgo_compile_options
)
