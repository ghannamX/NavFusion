#include "input/rinex/RinexObsParser.hpp"

#include <filesystem>
#include <iostream>
#include <spdlog/spdlog.h>

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: rinex_reader <path-to-rinex3-obs-file>\n";
        return 1;
    }

    const std::filesystem::path obsFilePath(argv[1]);

    spdlog::set_level(spdlog::level::info);
    spdlog::info("Reading RINEX 3 observation file: {}", obsFilePath.string());

    gnss::rinex::RinexObsParser parser;
    auto result = parser.parseObservationFile(obsFilePath);

    if (!result.parseSucceeded())
    {
        const auto &error = result.getParseError();
        spdlog::error("Parse failed at line {}: {}", error.lineNumber, error.description);
        return 1;
    }

    const auto &data = result.getResult();
    const auto &hdr  = data.header;

    std::cout << "\n";
    std::cout << "=== RINEX Header ===\n";
    std::cout << "  Version        : " << hdr.rinexVersion << "\n";
    std::cout << "  Marker         : " << hdr.markerName << "\n";
    std::cout << "  Receiver       : " << hdr.receiverType << "\n";
    std::cout << "  Antenna        : " << hdr.antennaType << "\n";
    std::cout << "  Approx XYZ     : "
              << hdr.approximatePosition.x << "  "
              << hdr.approximatePosition.y << "  "
              << hdr.approximatePosition.z << "\n";
    std::cout << "  Antenna H/E/N  : "
              << hdr.antennaDeltaOffset.height << "  "
              << hdr.antennaDeltaOffset.east   << "  "
              << hdr.antennaDeltaOffset.north  << "\n";
    std::cout << "  Interval       : " << hdr.nominalSamplingInterval << " s\n";
    std::cout << "  Leap seconds   : " << hdr.leapSeconds << "\n";

    std::cout << "\n=== Observation Types ===\n";
    for (const auto &[constellation, codes] : hdr.observationTypes)
    {
        std::cout << "  " << static_cast<char>(constellation)
                  << " (" << codes.size() << " types): ";
        for (const auto &code : codes)
        {
            std::cout << code.formatObsCodeAsRinex3String() << " ";
        }
        std::cout << "\n";
    }

    std::cout << "\n=== Epoch Summary ===\n";
    std::cout << "  Total epochs              : " << data.epochs.size() << "\n";
    std::cout << "  Total sat-epoch obs       : "
              << data.getTotalSatelliteEpochCount() << "\n";

    if (!data.epochs.empty())
    {
        const auto &first = data.epochs.front();
        const auto &last  = data.epochs.back();

        std::cout << "  First epoch               : "
                  << first.time.year << "-"
                  << first.time.month << "-"
                  << first.time.day << " "
                  << first.time.hour << ":"
                  << first.time.minute << ":"
                  << first.time.second << "\n";

        std::cout << "  Last epoch                : "
                  << last.time.year << "-"
                  << last.time.month << "-"
                  << last.time.day << " "
                  << last.time.hour << ":"
                  << last.time.minute << ":"
                  << last.time.second << "\n";

        // Print first epoch satellite details
        std::cout << "\n=== First Epoch Detail ===\n";
        std::cout << "  Satellites in first epoch : " << first.satellites.size() << "\n";

        for (const auto &sat : first.satellites)
        {
            std::cout << "    " << sat.satelliteId.formatSatelliteIdAsRinex3String()
                      << " : ";

            const auto sysIt = hdr.observationTypes.find(sat.satelliteId.system);
            if (sysIt == hdr.observationTypes.end())
            {
                std::cout << "(no obs types)\n";
                continue;
            }

            const auto &codes = sysIt->second;
            for (std::size_t i = 0; i < sat.observations.size() && i < codes.size(); ++i)
            {
                if (sat.observations[i].hasMeasurementValue())
                {
                    std::cout << codes[i].formatObsCodeAsRinex3String()
                              << "=" << sat.observations[i].value << " ";
                }
            }
            std::cout << "\n";
        }
    }

    return 0;
}
