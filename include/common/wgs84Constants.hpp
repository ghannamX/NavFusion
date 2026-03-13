#pragma once

namespace gnss
{

//------------------------------------------------------------------------------//
//                     WGS-84 & GPS Physical Constants                          //
//------------------------------------------------------------------------------//

/*!
 * \brief WGS-84 and GPS-defined physical constants for orbit and clock computation.
 *
 * Values are taken from IS-GPS-200 (Interface Specification) and the
 * WGS-84 reference system definition. The GPS-defined value of pi is
 * used (not C++ M_PI) to ensure bit-exact reproducibility across
 * implementations as mandated by the ICD.
 */
namespace wgs84
{

/*! \brief GPS-defined value of pi (IS-GPS-200). */
inline constexpr double PI = 3.1415926535898;

/*! \brief Speed of light in vacuum [m/s]. */
inline constexpr double SPEED_OF_LIGHT = 299792458.0;

/*! \brief WGS-84 Earth gravitational parameter (mu) [m^3/s^2]. */
inline constexpr double GRAVITATIONAL_PARAMETER = 3.986005e14;

/*! \brief WGS-84 Earth rotation rate (omega_e) [rad/s]. */
inline constexpr double EARTH_ROTATION_RATE = 7.2921151467e-5;

/*! \brief Relativistic correction constant F = -2*sqrt(mu)/c^2 [s/m^(1/2)].
 *
 * Used in the satellite clock relativistic correction:
 *   dt_rel = F * e * sqrtA * sin(E)
 */
inline constexpr double RELATIVISTIC_CLOCK_CORRECTION_CONSTANT = -4.442807633e-10;

/*! \brief Number of seconds in a half GPS week [s].
 *
 * Used for GPS time rollover detection: if |t - Toe| > HALF_WEEK,
 * add or subtract SECONDS_PER_WEEK.
 */
inline constexpr double HALF_WEEK_SECONDS = 302400.0;

/*! \brief Number of seconds in a full GPS week [s]. */
inline constexpr double SECONDS_PER_WEEK = 604800.0;

/*! \brief WGS-84 semi-major axis of the reference ellipsoid [m]. */
inline constexpr double SEMI_MAJOR_AXIS = 6378137.0;

/*! \brief WGS-84 flattening of the reference ellipsoid [dimensionless]. */
inline constexpr double FLATTENING = 1.0 / 298.257223563;

/*! \brief WGS-84 first eccentricity squared [dimensionless]. */
inline constexpr double ECCENTRICITY_SQUARED = 2.0 * FLATTENING - FLATTENING * FLATTENING;

} // namespace wgs84

} // namespace gnss
