#pragma once

#include "input/rinex/rinexNavTypes.hpp"
#include "common/satId.hpp"
#include <optional>
#include <vector>

namespace gnss
{

//------------------------------------------------------------------------------//
//                        Ephemeris Selector                                     //
//------------------------------------------------------------------------------//

/*!
 * \brief Selects the most appropriate broadcast ephemeris for a given satellite
 *        and observation epoch.
 *
 * Selection criteria (in order of priority):
 *  1. Satellite ID must match.
 *  2. Satellite health must be 0 (healthy).
 *  3. |t - Toe| must be within the maximum age threshold (default 2 hours).
 *  4. Among qualifying records, the one with the smallest |t - Toe| is chosen.
 *
 * If multiple records have the same Toe, the one with the larger IODE is
 * preferred (more recent upload).
 */
class EphemerisSelector
{
public:
    //--------------------------------------------------------------------------//
    //                         Public Interface                                 //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Select the best broadcast ephemeris for a satellite at a given time.
     *
     * \param[in] ephemerides         Vector of all parsed GPS ephemeris records.
     * \param[in] satelliteId         The satellite to find an ephemeris for.
     * \param[in] timeOfWeekSeconds   GPS time of week [seconds] of the observation.
     * \param[in] maximumAgeSeconds   Maximum allowed |t - Toe| [seconds].
     *                                Default is 7200 (2 hours, standard GPS validity).
     *
     * \return Pointer to the best matching ephemeris, or nullptr if none qualifies.
     */
    static const rinex::GpsBroadcastEphemeris *selectBestEphemerisForSatellite(
        const std::vector<rinex::GpsBroadcastEphemeris> &ephemerides,
        SatId satelliteId,
        double timeOfWeekSeconds,
        double maximumAgeSeconds = 7200.0);
};

} // namespace gnss
