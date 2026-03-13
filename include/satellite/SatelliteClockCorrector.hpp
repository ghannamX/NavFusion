#pragma once

#include "input/rinex/rinexNavTypes.hpp"

namespace gnss
{

//------------------------------------------------------------------------------//
//                  Satellite Clock Correction Result                            //
//------------------------------------------------------------------------------//

/*!
 * \brief Result of computing the satellite clock correction.
 */
struct SatelliteClockCorrectionResult
{
    double totalClockCorrection = 0.0;
    /*!< Total satellite clock correction [seconds].
     *   Includes polynomial + relativistic + TGD (for single-frequency).
     *   Apply as: t_corrected = t_sv - totalClockCorrection. */

    double polynomialCorrection = 0.0;
    /*!< Clock polynomial: af0 + af1*(t-Toc) + af2*(t-Toc)^2 [seconds]. */

    double relativisticCorrection = 0.0;
    /*!< Relativistic correction: F * e * sqrtA * sin(E) [seconds]. */
};

//------------------------------------------------------------------------------//
//                    Satellite Clock Corrector                                  //
//------------------------------------------------------------------------------//

/*!
 * \brief Computes GPS satellite clock corrections from broadcast ephemeris.
 *
 * Implements the IS-GPS-200 satellite clock correction model:
 *  1. Second-order polynomial: af0 + af1*(t - Toc) + af2*(t - Toc)^2
 *  2. Relativistic correction: F * e * sqrt(A) * sin(E)
 *  3. Total Group Delay (TGD) subtraction for single-frequency L1 users
 *
 * Reference: IS-GPS-200 Section 20.3.3.3.3.1.
 */
class SatelliteClockCorrector
{
public:
    //--------------------------------------------------------------------------//
    //                         Public Interface                                 //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Compute the satellite clock correction at a given time.
     *
     * \param[in] ephemeris            GPS broadcast ephemeris record.
     * \param[in] timeOfWeekSeconds    GPS time of week [seconds] at which to
     *                                 evaluate the clock correction.
     * \param[in] eccentricAnomaly     Eccentric anomaly E [rad] from the orbit
     *                                 computation (needed for relativistic term).
     * \param[in] applySingleFrequencyTgd  If true, subtract TGD for L1 C/A users.
     *
     * \return Clock correction components and total [seconds].
     */
    static SatelliteClockCorrectionResult computeSatelliteClockCorrection(
        const rinex::GpsBroadcastEphemeris &ephemeris,
        double timeOfWeekSeconds,
        double eccentricAnomaly,
        bool applySingleFrequencyTgd = true);

    /*!
     * \brief Compute the relativistic clock correction.
     *
     * dt_rel = F * e * sqrtA * sin(E)
     *
     * \param[in] eccentricity          Orbital eccentricity e.
     * \param[in] squareRootOfSemiMajorAxis  sqrt(A) [m^0.5].
     * \param[in] eccentricAnomaly      Eccentric anomaly E [rad].
     * \return Relativistic correction [seconds].
     */
    static double computeRelativisticClockCorrection(
        double eccentricity,
        double squareRootOfSemiMajorAxis,
        double eccentricAnomaly);
};

} // namespace gnss
