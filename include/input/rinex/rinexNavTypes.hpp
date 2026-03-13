#pragma once

#include "common/satId.hpp"
#include "time/GnssTime.hpp"
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace gnss::rinex
{

//------------------------------------------------------------------------------//
//                    Klobuchar Ionosphere Parameters                            //
//------------------------------------------------------------------------------//

/*!
 * \brief Klobuchar ionospheric model coefficients from the RINEX 3 nav header.
 *
 * Parsed from the IONOSPHERIC CORR header records labelled GPSA and GPSB.
 * The alpha coefficients define the vertical delay amplitude and the beta
 * coefficients define the period of the cosine model.
 *
 * Reference: IS-GPS-200 Section 20.3.3.5.2.5 (Ionospheric Correction Algorithm).
 */
struct KlobucharIonosphereParameters
{
    //--------------------------------------------------------------------------//
    //                            Member Variables                               //
    //--------------------------------------------------------------------------//

    std::array<double, 4> alpha = {};
    /*!< GPSA coefficients: alpha0 [s], alpha1 [s/semi-circle],
     *   alpha2 [s/semi-circle^2], alpha3 [s/semi-circle^3]. */

    std::array<double, 4> beta = {};
    /*!< GPSB coefficients: beta0 [s], beta1 [s/semi-circle],
     *   beta2 [s/semi-circle^2], beta3 [s/semi-circle^3]. */

    bool hasAlpha = false;
    /*!< true if the GPSA record was found in the header. */

    bool hasBeta = false;
    /*!< true if the GPSB record was found in the header. */

    //--------------------------------------------------------------------------//
    //                          Query Methods                                    //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Returns true if both alpha and beta coefficient sets are available.
     * \return true if the Klobuchar model can be applied.
     */
    bool isComplete() const noexcept
    {
        return hasAlpha && hasBeta;
    }
};

//------------------------------------------------------------------------------//
//                     GPS Broadcast Ephemeris Record                            //
//------------------------------------------------------------------------------//

/*!
 * \brief GPS broadcast ephemeris parameters from a RINEX 3 navigation record.
 *
 * Contains the complete set of Keplerian orbital elements, clock correction
 * polynomial, and health/accuracy metadata for a single GPS satellite at a
 * single reference epoch (Toc/Toe).
 *
 * Field layout follows the RINEX 3.04 specification, Table A5 (GPS Navigation
 * Message File — Record Description). Each group of four parameters corresponds
 * to one line of the 8-line navigation record.
 *
 * Units are SI throughout unless noted. Angular quantities are in radians.
 * All values are read as Fortran D19.12 double-precision fields.
 *
 * Reference: IS-GPS-200 Section 20.3.3 (Navigation Message Content).
 */
struct GpsBroadcastEphemeris
{
    //--------------------------------------------------------------------------//
    //                  Satellite ID and Reference Epoch                         //
    //--------------------------------------------------------------------------//

    SatId    satelliteId;
    /*!< GPS satellite identifier (e.g. G01, G32). */

    DateTime timeOfClock;
    /*!< Toc — reference epoch for the SV clock correction polynomial.
     *   Calendar time in GPS time system. */

    //--------------------------------------------------------------------------//
    //                  SV Clock Correction (Record Line 0)                      //
    //--------------------------------------------------------------------------//

    double clockBias     = 0.0;
    /*!< af0 — satellite clock bias [seconds]. */

    double clockDrift    = 0.0;
    /*!< af1 — satellite clock drift [seconds/second]. */

    double clockDriftRate = 0.0;
    /*!< af2 — satellite clock drift rate [seconds/second^2]. */

    //--------------------------------------------------------------------------//
    //                  Broadcast Orbit — 1 (Record Line 1)                     //
    //--------------------------------------------------------------------------//

    double issueOfDataEphemeris = 0.0;
    /*!< IODE — Issue of Data, Ephemeris. Matches subframe 2/3 IODE. */

    double sinCorrectionToOrbitRadius = 0.0;
    /*!< Crs — amplitude of the sine harmonic correction to the orbit radius [metres]. */

    double meanMotionDifference = 0.0;
    /*!< Delta n — mean motion difference from computed value [rad/s]. */

    double meanAnomalyAtReference = 0.0;
    /*!< M0 — mean anomaly at reference time Toe [radians]. */

    //--------------------------------------------------------------------------//
    //                  Broadcast Orbit — 2 (Record Line 2)                     //
    //--------------------------------------------------------------------------//

    double cosCorrectionToLatitude = 0.0;
    /*!< Cuc — amplitude of the cosine harmonic correction to the argument
     *   of latitude [radians]. */

    double eccentricity = 0.0;
    /*!< e — eccentricity of the satellite orbit [dimensionless]. */

    double sinCorrectionToLatitude = 0.0;
    /*!< Cus — amplitude of the sine harmonic correction to the argument
     *   of latitude [radians]. */

    double squareRootOfSemiMajorAxis = 0.0;
    /*!< sqrt(A) — square root of the semi-major axis [metres^0.5]. */

    //--------------------------------------------------------------------------//
    //                  Broadcast Orbit — 3 (Record Line 3)                     //
    //--------------------------------------------------------------------------//

    double timeOfEphemeris = 0.0;
    /*!< Toe — time of ephemeris, reference time for orbital elements
     *   [seconds of GPS week]. */

    double cosCorrectionToInclination = 0.0;
    /*!< Cic — amplitude of the cosine harmonic correction to the
     *   inclination angle [radians]. */

    double longitudeOfAscendingNode = 0.0;
    /*!< OMEGA0 — longitude of the ascending node at weekly epoch [radians]. */

    double sinCorrectionToInclination = 0.0;
    /*!< Cis — amplitude of the sine harmonic correction to the
     *   inclination angle [radians]. */

    //--------------------------------------------------------------------------//
    //                  Broadcast Orbit — 4 (Record Line 4)                     //
    //--------------------------------------------------------------------------//

    double inclinationAngle = 0.0;
    /*!< i0 — inclination angle at reference time Toe [radians]. */

    double cosCorrectionToOrbitRadius = 0.0;
    /*!< Crc — amplitude of the cosine harmonic correction to the orbit
     *   radius [metres]. */

    double argumentOfPerigee = 0.0;
    /*!< omega — argument of perigee [radians]. */

    double rateOfRightAscension = 0.0;
    /*!< OMEGA_DOT — rate of right ascension [rad/s]. */

    //--------------------------------------------------------------------------//
    //                  Broadcast Orbit — 5 (Record Line 5)                     //
    //--------------------------------------------------------------------------//

    double rateOfInclination = 0.0;
    /*!< IDOT — rate of inclination angle [rad/s]. */

    double codesOnL2Channel = 0.0;
    /*!< Codes on L2 channel (0 = reserved, 1 = P code, 2 = C/A code). */

    double gpsWeekNumber = 0.0;
    /*!< GPS week number (continuous, not modulo 1024). To go with Toe. */

    double l2PDataFlag = 0.0;
    /*!< L2 P data flag (0 = ON, 1 = OFF). */

    //--------------------------------------------------------------------------//
    //                  Broadcast Orbit — 6 (Record Line 6)                     //
    //--------------------------------------------------------------------------//

    double signalAccuracy = 0.0;
    /*!< SV accuracy — User Range Accuracy (URA) index [metres].
     *   Mapped from the 4-bit URA index in the navigation message. */

    double satelliteHealth = 0.0;
    /*!< SV health — satellite health bits (0 = healthy).
     *   Bits 0-4 from subframe 1 word 3. */

    double totalGroupDelay = 0.0;
    /*!< TGD — estimated group delay differential between L1 P(Y) and L1 C/A
     *   [seconds]. Applied to single-frequency (L1) pseudoranges. */

    double issueOfDataClock = 0.0;
    /*!< IODC — Issue of Data, Clock. 10-bit value from subframe 1. */

    //--------------------------------------------------------------------------//
    //                  Broadcast Orbit — 7 (Record Line 7)                     //
    //--------------------------------------------------------------------------//

    double transmissionTime = 0.0;
    /*!< Transmission time of message [seconds of GPS week].
     *   Time of transmission of the first bit of subframe 1. */

    double fitInterval = 0.0;
    /*!< Fit interval [hours]. Duration over which the ephemeris is valid.
     *   0 or blank = 4 hours (standard), > 0 = value in hours. */
};

//------------------------------------------------------------------------------//
//                     RINEX 3 Navigation File Header                           //
//------------------------------------------------------------------------------//

/*!
 * \brief Parsed representation of a RINEX 3 navigation file header.
 *
 * Contains metadata and ionospheric correction model coefficients.
 * Only the records relevant to GPS SPP processing are parsed; other
 * header records (COMMENT, TIME SYSTEM CORR, etc.) are silently skipped.
 */
struct RinexNavHeader
{
    //--------------------------------------------------------------------------//
    //                            Member Variables                               //
    //--------------------------------------------------------------------------//

    double      rinexVersion = 0.0;
    /*!< RINEX version from RINEX VERSION / TYPE record. */

    char        fileType = 'N';
    /*!< File type character ('N' for navigation). */

    char        satelliteSystem = 'M';
    /*!< Satellite system character ('G' = GPS only, 'M' = mixed, etc.). */

    std::string programName;
    /*!< From PGM / RUN BY / DATE. */

    std::string runBy;
    /*!< From PGM / RUN BY / DATE. */

    int         leapSeconds = 0;
    /*!< Leap seconds from the LEAP SECONDS header record. 0 if absent. */

    KlobucharIonosphereParameters klobucharIonosphere;
    /*!< GPS Klobuchar ionospheric model alpha/beta coefficients.
     *   Parsed from IONOSPHERIC CORR GPSA/GPSB header records. */
};

//------------------------------------------------------------------------------//
//                   Complete RINEX 3 Navigation File Data                       //
//------------------------------------------------------------------------------//

/*!
 * \brief Top-level result of parsing a RINEX 3 navigation file.
 *
 * GPS ephemeris records are stored in file order. Multiple records per
 * satellite are common (broadcast every 2 hours, 4-hour validity).
 * Downstream code must select the most appropriate ephemeris for a given
 * epoch based on IODE matching and Toe proximity.
 */
struct RinexNavData
{
    //--------------------------------------------------------------------------//
    //                            Member Variables                               //
    //--------------------------------------------------------------------------//

    RinexNavHeader header;
    /*!< Parsed file header containing metadata and ionospheric coefficients. */

    std::vector<GpsBroadcastEphemeris> gpsEphemerides;
    /*!< All GPS broadcast ephemeris records in file order.
     *   Typically multiple records per satellite per day. */

    //--------------------------------------------------------------------------//
    //                          Statistics Methods                               //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Return the total number of ephemeris records across all constellations.
     * \return Total ephemeris record count.
     */
    std::size_t getTotalEphemerisCount() const noexcept
    {
        return gpsEphemerides.size();
    }
};

} // namespace gnss::rinex
