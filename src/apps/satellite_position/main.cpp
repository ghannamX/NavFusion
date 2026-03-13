#include "input/rinex/RinexNavParser.hpp"
#include "input/rinex/RinexObsParser.hpp"
#include "satellite/BroadcastOrbitComputer.hpp"
#include "satellite/SatelliteClockCorrector.hpp"
#include "satellite/EphemerisSelector.hpp"
#include "satellite/SagnacCorrection.hpp"
#include "common/wgs84Constants.hpp"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <spdlog/spdlog.h>

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: satellite_position <obs-file> <nav-file>\n";
        std::cerr << "  Computes satellite ECEF positions for the first epoch.\n";
        return 1;
    }

    const std::filesystem::path obsFilePath(argv[1]);
    const std::filesystem::path navFilePath(argv[2]);

    spdlog::set_level(spdlog::level::info);

    // Parse observation file (GPS only)
    spdlog::info("Parsing observation file: {}", obsFilePath.string());
    gnss::rinex::RinexObsParserOptions obsOptions;
    obsOptions.constellationFilter = {gnss::Constellation::GPS};
    gnss::rinex::RinexObsParser obsParser(obsOptions);
    auto obsResult = obsParser.parseObservationFile(obsFilePath);

    if (!obsResult.parseSucceeded())
    {
        spdlog::error("Obs parse failed: {}", obsResult.getParseError().description);
        return 1;
    }

    const auto &obsData = obsResult.getResult();
    spdlog::info("Parsed {} epochs", obsData.epochs.size());

    // Parse navigation file
    spdlog::info("Parsing navigation file: {}", navFilePath.string());
    gnss::rinex::RinexNavParser navParser;
    auto navResult = navParser.parseNavigationFile(navFilePath);

    if (!navResult.parseSucceeded())
    {
        spdlog::error("Nav parse failed: {}", navResult.getParseError().description);
        return 1;
    }

    const auto &navData = navResult.getResult();
    spdlog::info("Parsed {} GPS ephemeris records", navData.gpsEphemerides.size());

    if (obsData.epochs.empty())
    {
        spdlog::error("No epochs in observation file");
        return 1;
    }

    // Use first epoch
    const auto &firstEpoch = obsData.epochs[0];
    const auto gpsTime =
        firstEpoch.time.convertToGpsTime();

    std::cout << "\n=== First Epoch ===\n";
    std::cout << "  Time: "
              << firstEpoch.time.year_ << "-"
              << std::setfill('0') << std::setw(2) << firstEpoch.time.month_ << "-"
              << std::setw(2) << firstEpoch.time.day_ << " "
              << std::setw(2) << firstEpoch.time.hour_ << ":"
              << std::setw(2) << firstEpoch.time.minute_ << ":"
              << std::setw(5) << std::fixed << std::setprecision(2)
              << firstEpoch.time.second_
              << std::setfill(' ') << "\n";
    std::cout << "  GPS Week: " << gpsTime.weekNumber
              << "  ToW: " << std::fixed << std::setprecision(3)
              << gpsTime.timeOfWeek << " s\n";
    std::cout << "  Satellites: " << firstEpoch.satellites.size() << "\n";

    // Find C1C observation code slot index for pseudorange
    const auto &obsTypes = obsData.header.observationTypes;
    int pseudorangeSlotIndex = -1;

    auto gpsObsIt = obsTypes.find(gnss::Constellation::GPS);
    if (gpsObsIt != obsTypes.end())
    {
        const auto &gpsCodes = gpsObsIt->second;
        for (std::size_t i = 0; i < gpsCodes.size(); ++i)
        {
            if (gpsCodes[i].formatObsCodeAsRinex3String() == "C1C")
            {
                pseudorangeSlotIndex = static_cast<int>(i);
                break;
            }
        }
    }

    if (pseudorangeSlotIndex < 0)
    {
        spdlog::error("C1C pseudorange observation code not found in GPS obs types");
        return 1;
    }

    // Compute satellite positions
    std::cout << "\n=== Satellite Positions (ECEF) ===\n";
    std::cout << std::setw(5) << "SAT"
              << std::setw(16) << "X [m]"
              << std::setw(16) << "Y [m]"
              << std::setw(16) << "Z [m]"
              << std::setw(14) << "Range [km]"
              << std::setw(16) << "Clk Corr [us]"
              << std::setw(14) << "Rel Corr [ns]"
              << "\n";
    std::cout << "  " << std::string(93, '-') << "\n";

    for (const auto &satObs : firstEpoch.satellites)
    {
        if (satObs.satelliteId.system != gnss::Constellation::GPS)
        {
            continue;
        }

        // Get C1C pseudorange
        if (pseudorangeSlotIndex >= static_cast<int>(satObs.observations.size()))
        {
            continue;
        }

        const auto &pseudorangeObs = satObs.observations[pseudorangeSlotIndex];
        if (!pseudorangeObs.hasMeasurementValue())
        {
            continue;
        }

        const double pseudorange = pseudorangeObs.value;

        // Select best ephemeris
        const auto *ephemeris = gnss::EphemerisSelector::selectBestEphemerisForSatellite(
            navData.gpsEphemerides,
            satObs.satelliteId,
            gpsTime.timeOfWeek);

        if (ephemeris == nullptr)
        {
            std::cout << "  " << satObs.satelliteId.formatSatelliteIdAsRinex3String()
                      << "  -- no valid ephemeris --\n";
            continue;
        }

        // Transmission time iteration (3 iterations for convergence)
        double transmissionTimeOfWeek =
            gpsTime.timeOfWeek - pseudorange / gnss::wgs84::SPEED_OF_LIGHT;

        gnss::BroadcastOrbitResult orbitResult;
        gnss::SatelliteClockCorrectionResult clockResult;

        for (int iteration = 0; iteration < 3; ++iteration)
        {
            orbitResult =
                gnss::BroadcastOrbitComputer::computeSatellitePositionFromBroadcastEphemeris(
                    *ephemeris, transmissionTimeOfWeek);

            clockResult =
                gnss::SatelliteClockCorrector::computeSatelliteClockCorrection(
                    *ephemeris, transmissionTimeOfWeek,
                    orbitResult.eccentricAnomaly, true);

            transmissionTimeOfWeek =
                gpsTime.timeOfWeek
              - pseudorange / gnss::wgs84::SPEED_OF_LIGHT
              - clockResult.totalClockCorrection;
        }

        // Apply Sagnac correction (Earth rotation during signal travel)
        const double signalTravelTime =
            pseudorange / gnss::wgs84::SPEED_OF_LIGHT;

        const Eigen::Vector3d correctedPosition =
            gnss::SagnacCorrection::applySagnacCorrectionToSatellitePosition(
                orbitResult.positionEcef, signalTravelTime);

        // Geometric range from Earth centre (sanity check: ~26,500 km)
        const double distanceFromEarthCentre = correctedPosition.norm() / 1000.0;

        std::cout << "  " << satObs.satelliteId.formatSatelliteIdAsRinex3String()
                  << std::fixed << std::setprecision(3)
                  << std::setw(16) << correctedPosition.x()
                  << std::setw(16) << correctedPosition.y()
                  << std::setw(16) << correctedPosition.z()
                  << std::setw(14) << distanceFromEarthCentre
                  << std::setw(16) << std::setprecision(3)
                  << clockResult.totalClockCorrection * 1.0e6
                  << std::setw(14) << std::setprecision(3)
                  << clockResult.relativisticCorrection * 1.0e9
                  << "\n";
    }

    return 0;
}
