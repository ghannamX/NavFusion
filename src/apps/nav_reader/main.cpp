#include "input/rinex/RinexNavParser.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <spdlog/spdlog.h>

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: nav_reader <path-to-rinex3-nav-file>\n";
        return 1;
    }

    const std::filesystem::path navFilePath(argv[1]);

    spdlog::set_level(spdlog::level::info);
    spdlog::info("Reading RINEX 3 navigation file: {}", navFilePath.string());

    gnss::rinex::RinexNavParser parser;
    auto result = parser.parseNavigationFile(navFilePath);

    if (!result.parseSucceeded())
    {
        const auto &error = result.getParseError();
        spdlog::error("Parse failed at line {}: {}", error.lineNumber, error.description);
        return 1;
    }

    const auto &data = result.getResult();
    const auto &hdr  = data.header;

    std::cout << "\n";
    std::cout << "=== RINEX Navigation Header ===\n";
    std::cout << "  Version          : " << hdr.rinexVersion << "\n";
    std::cout << "  File type        : " << hdr.fileType << "\n";
    std::cout << "  Satellite system : " << hdr.satelliteSystem << "\n";
    std::cout << "  Leap seconds     : " << hdr.leapSeconds << "\n";

    if (hdr.klobucharIonosphere.isComplete())
    {
        std::cout << "\n=== Klobuchar Ionosphere Parameters ===\n";
        std::cout << std::scientific << std::setprecision(4);
        std::cout << "  Alpha : ";
        for (int i = 0; i < 4; ++i)
        {
            std::cout << std::setw(13) << hdr.klobucharIonosphere.alpha[i];
        }
        std::cout << "\n";
        std::cout << "  Beta  : ";
        for (int i = 0; i < 4; ++i)
        {
            std::cout << std::setw(13) << hdr.klobucharIonosphere.beta[i];
        }
        std::cout << "\n";
    }
    else
    {
        std::cout << "  Klobuchar iono   : incomplete or absent\n";
    }

    std::cout << "\n=== Ephemeris Summary ===\n";
    std::cout << "  GPS records      : " << data.gpsEphemerides.size() << "\n";

    if (!data.gpsEphemerides.empty())
    {
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "\n=== GPS Ephemeris Records ===\n";
        std::cout << "  SAT   Toc                     "
                  << "af0 [s]          "
                  << "e [-]           "
                  << "sqrt(A) [m^0.5]  "
                  << "IODE  "
                  << "Health\n";
        std::cout << "  "
                  << std::string(100, '-') << "\n";

        for (const auto &eph : data.gpsEphemerides)
        {
            std::cout << "  "
                      << eph.satelliteId.formatSatelliteIdAsRinex3String() << "   "
                      << eph.timeOfClock.year_ << "-"
                      << std::setfill('0') << std::setw(2) << eph.timeOfClock.month_ << "-"
                      << std::setw(2) << eph.timeOfClock.day_ << " "
                      << std::setw(2) << eph.timeOfClock.hour_ << ":"
                      << std::setw(2) << eph.timeOfClock.minute_ << ":"
                      << std::setw(2) << static_cast<int>(eph.timeOfClock.second_)
                      << std::setfill(' ') << "   "
                      << std::scientific << std::setprecision(6)
                      << std::setw(16) << eph.clockBias << " "
                      << std::setw(16) << eph.eccentricity << " "
                      << std::fixed << std::setprecision(4)
                      << std::setw(13) << eph.squareRootOfSemiMajorAxis << "  "
                      << std::setw(4) << static_cast<int>(eph.issueOfDataEphemeris) << "  "
                      << std::setw(4) << static_cast<int>(eph.satelliteHealth)
                      << "\n";
        }
    }

    return 0;
}
