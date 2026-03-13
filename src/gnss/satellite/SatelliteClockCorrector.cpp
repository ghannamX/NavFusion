#include "satellite/SatelliteClockCorrector.hpp"
#include "common/wgs84Constants.hpp"

#include <cmath>

namespace gnss
{

//------------------------------------------------------------------------------//
//                   Satellite Clock Correction                                 //
//------------------------------------------------------------------------------//

SatelliteClockCorrectionResult SatelliteClockCorrector::computeSatelliteClockCorrection(
    const rinex::GpsBroadcastEphemeris &ephemeris,
    double timeOfWeekSeconds,
    double eccentricAnomaly,
    bool applySingleFrequencyTgd)
{
    // Time from clock reference epoch (Toc), with GPS week rollover handling.
    double timeSinceClockEpoch =
        timeOfWeekSeconds - ephemeris.timeOfClock.convertToGpsTime().timeOfWeek;

    if (timeSinceClockEpoch > wgs84::HALF_WEEK_SECONDS)
    {
        timeSinceClockEpoch -= wgs84::SECONDS_PER_WEEK;
    }
    else if (timeSinceClockEpoch < -wgs84::HALF_WEEK_SECONDS)
    {
        timeSinceClockEpoch += wgs84::SECONDS_PER_WEEK;
    }

    // Clock polynomial: af0 + af1*(t - Toc) + af2*(t - Toc)^2
    const double polynomialCorrection =
        ephemeris.clockBias
      + ephemeris.clockDrift * timeSinceClockEpoch
      + ephemeris.clockDriftRate * timeSinceClockEpoch * timeSinceClockEpoch;

    // Relativistic correction: F * e * sqrtA * sin(E)
    const double relativisticCorrection = computeRelativisticClockCorrection(
        ephemeris.eccentricity,
        ephemeris.squareRootOfSemiMajorAxis,
        eccentricAnomaly);

    // Total clock correction
    double totalCorrection = polynomialCorrection + relativisticCorrection;

    // Subtract TGD for single-frequency L1 C/A users
    if (applySingleFrequencyTgd)
    {
        totalCorrection -= ephemeris.totalGroupDelay;
    }

    return SatelliteClockCorrectionResult{
        totalCorrection,
        polynomialCorrection,
        relativisticCorrection
    };
}

//------------------------------------------------------------------------------//
//                   Relativistic Clock Correction                              //
//------------------------------------------------------------------------------//

double SatelliteClockCorrector::computeRelativisticClockCorrection(
    double eccentricity,
    double squareRootOfSemiMajorAxis,
    double eccentricAnomaly)
{
    return wgs84::RELATIVISTIC_CLOCK_CORRECTION_CONSTANT
         * eccentricity
         * squareRootOfSemiMajorAxis
         * std::sin(eccentricAnomaly);
}

} // namespace gnss
