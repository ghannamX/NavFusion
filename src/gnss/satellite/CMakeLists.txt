add_library(gnss_satellite OBJECT)

target_sources(gnss_satellite PRIVATE
    BroadcastOrbitComputer.cpp
    SatelliteClockCorrector.cpp
    EphemerisSelector.cpp
    SagnacCorrection.cpp
)

target_include_directories(gnss_satellite
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gnss_satellite
    PUBLIC Eigen3::Eigen gnss_time fgo_compile_options
)
