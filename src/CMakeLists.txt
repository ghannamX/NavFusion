# ── Core library ─────────────────────────────────────────────────────────────
# Modules are added here as they are implemented.
# Phase 1: time utilities + input parsers + satellite orbit/clock

add_library(fgo_ppp STATIC)

add_subdirectory(gnss/time)
add_subdirectory(gnss/input)
add_subdirectory(gnss/satellite)

target_link_libraries(fgo_ppp
    PUBLIC
        fgo_compile_options
        Eigen3::Eigen
        spdlog::spdlog
        fmt::fmt
        gnss_time
        gnss_input
        gnss_satellite
)

target_include_directories(fgo_ppp
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
