#include "satellite/BroadcastOrbitComputer.hpp"
#include "common/wgs84Constants.hpp"

#include <cmath>

namespace gnss
{

//------------------------------------------------------------------------------//
//                   Satellite Position from Broadcast Ephemeris                //
//------------------------------------------------------------------------------//

BroadcastOrbitResult BroadcastOrbitComputer::computeSatellitePositionFromBroadcastEphemeris(
    const rinex::GpsBroadcastEphemeris &ephemeris,
    double timeOfWeekSeconds)
{
    // Step 1: Time from ephemeris reference epoch
    const double tk = computeTimeFromEphemerisEpoch(
        timeOfWeekSeconds, ephemeris.timeOfEphemeris);

    // Step 2: Semi-major axis and mean motion
    const double semiMajorAxis =
        ephemeris.squareRootOfSemiMajorAxis * ephemeris.squareRootOfSemiMajorAxis;

    const double computedMeanMotion =
        std::sqrt(wgs84::GRAVITATIONAL_PARAMETER / (semiMajorAxis * semiMajorAxis * semiMajorAxis));

    const double correctedMeanMotion =
        computedMeanMotion + ephemeris.meanMotionDifference;

    // Step 3: Mean anomaly at time t
    const double meanAnomaly =
        ephemeris.meanAnomalyAtReference + correctedMeanMotion * tk;

    // Step 4: Solve Kepler's equation for eccentric anomaly
    const double eccentricAnomaly =
        solveKeplerEquationIteratively(meanAnomaly, ephemeris.eccentricity);

    // Step 5: True anomaly
    const double sinEccentricAnomaly = std::sin(eccentricAnomaly);
    const double cosEccentricAnomaly = std::cos(eccentricAnomaly);

    const double oneMinusEcosE = 1.0 - ephemeris.eccentricity * cosEccentricAnomaly;

    const double sinTrueAnomaly =
        (std::sqrt(1.0 - ephemeris.eccentricity * ephemeris.eccentricity) * sinEccentricAnomaly)
        / oneMinusEcosE;

    const double cosTrueAnomaly =
        (cosEccentricAnomaly - ephemeris.eccentricity) / oneMinusEcosE;

    const double trueAnomaly = std::atan2(sinTrueAnomaly, cosTrueAnomaly);

    // Step 6: Argument of latitude (uncorrected)
    const double argumentOfLatitude = trueAnomaly + ephemeris.argumentOfPerigee;

    // Step 7: Second harmonic corrections
    const double sin2phi = std::sin(2.0 * argumentOfLatitude);
    const double cos2phi = std::cos(2.0 * argumentOfLatitude);

    const double correctionToArgumentOfLatitude =
        ephemeris.sinCorrectionToLatitude * sin2phi
      + ephemeris.cosCorrectionToLatitude * cos2phi;

    const double correctionToOrbitRadius =
        ephemeris.sinCorrectionToOrbitRadius * sin2phi
      + ephemeris.cosCorrectionToOrbitRadius * cos2phi;

    const double correctionToInclination =
        ephemeris.sinCorrectionToInclination * sin2phi
      + ephemeris.cosCorrectionToInclination * cos2phi;

    // Step 8: Corrected argument of latitude, radius, and inclination
    const double correctedArgumentOfLatitude =
        argumentOfLatitude + correctionToArgumentOfLatitude;

    const double correctedOrbitRadius =
        semiMajorAxis * oneMinusEcosE + correctionToOrbitRadius;

    const double correctedInclination =
        ephemeris.inclinationAngle
      + correctionToInclination
      + ephemeris.rateOfInclination * tk;

    // Step 9: Position in orbital plane
    const double positionInOrbitalPlaneX =
        correctedOrbitRadius * std::cos(correctedArgumentOfLatitude);

    const double positionInOrbitalPlaneY =
        correctedOrbitRadius * std::sin(correctedArgumentOfLatitude);

    // Step 10: Corrected longitude of ascending node
    const double correctedLongitudeOfAscendingNode =
        ephemeris.longitudeOfAscendingNode
      + (ephemeris.rateOfRightAscension - wgs84::EARTH_ROTATION_RATE) * tk
      - wgs84::EARTH_ROTATION_RATE * ephemeris.timeOfEphemeris;

    const double sinOmega = std::sin(correctedLongitudeOfAscendingNode);
    const double cosOmega = std::cos(correctedLongitudeOfAscendingNode);
    const double sinInclination = std::sin(correctedInclination);
    const double cosInclination = std::cos(correctedInclination);

    // Step 11: Earth-Centered Earth-Fixed (ECEF) coordinates
    const double satelliteX =
        positionInOrbitalPlaneX * cosOmega
      - positionInOrbitalPlaneY * cosInclination * sinOmega;

    const double satelliteY =
        positionInOrbitalPlaneX * sinOmega
      + positionInOrbitalPlaneY * cosInclination * cosOmega;

    const double satelliteZ =
        positionInOrbitalPlaneY * sinInclination;

    return BroadcastOrbitResult{
        Eigen::Vector3d{satelliteX, satelliteY, satelliteZ},
        eccentricAnomaly
    };
}

//------------------------------------------------------------------------------//
//                         Kepler's Equation Solver                             //
//------------------------------------------------------------------------------//

double BroadcastOrbitComputer::solveKeplerEquationIteratively(
    double meanAnomaly,
    double eccentricity)
{
    double eccentricAnomaly = meanAnomaly;

    for (int iteration = 0; iteration < MAX_KEPLER_ITERATIONS; ++iteration)
    {
        const double delta =
            (meanAnomaly - eccentricAnomaly + eccentricity * std::sin(eccentricAnomaly))
            / (1.0 - eccentricity * std::cos(eccentricAnomaly));

        eccentricAnomaly += delta;

        if (std::fabs(delta) < KEPLER_CONVERGENCE_THRESHOLD)
        {
            break;
        }
    }

    return eccentricAnomaly;
}

//------------------------------------------------------------------------------//
//                    Time from Ephemeris Epoch                                  //
//------------------------------------------------------------------------------//

double BroadcastOrbitComputer::computeTimeFromEphemerisEpoch(
    double timeOfWeekSeconds,
    double timeOfEphemeris)
{
    double tk = timeOfWeekSeconds - timeOfEphemeris;

    if (tk > wgs84::HALF_WEEK_SECONDS)
    {
        tk -= wgs84::SECONDS_PER_WEEK;
    }
    else if (tk < -wgs84::HALF_WEEK_SECONDS)
    {
        tk += wgs84::SECONDS_PER_WEEK;
    }

    return tk;
}

} // namespace gnss
