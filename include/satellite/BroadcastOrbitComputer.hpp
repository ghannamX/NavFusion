#pragma once

#include "input/rinex/rinexNavTypes.hpp"
#include "common/wgs84Constants.hpp"
#include <Eigen/Core>

namespace gnss
{

//------------------------------------------------------------------------------//
//                     Broadcast Orbit Computation Result                       //
//------------------------------------------------------------------------------//

/*!
 * \brief Result of computing a satellite position from broadcast ephemeris.
 *
 * Contains the ECEF position vector and the eccentric anomaly E, which is
 * needed by the satellite clock relativistic correction.
 */
struct BroadcastOrbitResult
{
    Eigen::Vector3d positionEcef;  /*!< Satellite ECEF position [m]. */
    double eccentricAnomaly = 0.0; /*!< Eccentric anomaly E [rad] (for clock correction). */
};

//------------------------------------------------------------------------------//
//                    Broadcast Orbit Computer                                  //
//------------------------------------------------------------------------------//

/*!
 * \brief Computes GPS satellite ECEF position from broadcast ephemeris.
 *
 * Implements the IS-GPS-200 algorithm for computing satellite coordinates
 * from the Keplerian orbital elements in the broadcast navigation message.
 *
 * The computation steps are:
 *  1. Compute time from ephemeris reference epoch (tk = t - Toe)
 *  2. Compute mean motion and mean anomaly
 *  3. Solve Kepler's equation iteratively for eccentric anomaly E
 *  4. Compute true anomaly, argument of latitude with harmonic corrections
 *  5. Compute radius and inclination with harmonic corrections
 *  6. Compute position in orbital plane
 *  7. Rotate to ECEF via corrected longitude of ascending node
 *
 * Reference: IS-GPS-200 Section 20.3.3.4.3 (User Algorithm for Ephemeris
 * Determination), Table 20-IV.
 */
class BroadcastOrbitComputer
{
public:
    //--------------------------------------------------------------------------//
    //                         Public Interface                                 //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Compute satellite ECEF position at a given GPS time of week.
     *
     * \param[in] ephemeris       GPS broadcast ephemeris record.
     * \param[in] timeOfWeekSeconds  GPS time of week at which to compute the
     *                            position [seconds]. This should be the signal
     *                            transmission time (corrected for satellite clock).
     *
     * \return Satellite ECEF position [m] and eccentric anomaly [rad].
     */
    static BroadcastOrbitResult computeSatellitePositionFromBroadcastEphemeris(
        const rinex::GpsBroadcastEphemeris &ephemeris,
        double timeOfWeekSeconds);

private:
    //--------------------------------------------------------------------------//
    //                        Internal Helpers                                  //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Solve Kepler's equation M = E - e*sin(E) iteratively.
     *
     * Uses Newton-Raphson iteration until convergence to within 1e-12 radians.
     * Typically converges in 5-10 iterations for GPS eccentricities (e < 0.02).
     *
     * \param[in] meanAnomaly   Mean anomaly M [rad].
     * \param[in] eccentricity  Orbital eccentricity e [dimensionless].
     * \return Eccentric anomaly E [rad].
     */
    static double solveKeplerEquationIteratively(
        double meanAnomaly,
        double eccentricity);

    /*!
     * \brief Compute elapsed time from ephemeris epoch, with GPS week rollover.
     *
     * Adjusts for GPS week boundary crossings:
     *   if tk > 302400, subtract 604800;
     *   if tk < -302400, add 604800.
     *
     * \param[in] timeOfWeekSeconds  Current GPS time of week [s].
     * \param[in] timeOfEphemeris    Toe from the broadcast ephemeris [s].
     * \return Time elapsed since Toe [s], corrected for week rollover.
     */
    static double computeTimeFromEphemerisEpoch(
        double timeOfWeekSeconds,
        double timeOfEphemeris);

    static constexpr int    MAX_KEPLER_ITERATIONS = 30;
    static constexpr double KEPLER_CONVERGENCE_THRESHOLD = 1.0e-12;
};

} // namespace gnss
