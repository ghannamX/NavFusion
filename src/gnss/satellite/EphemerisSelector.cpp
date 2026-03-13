#include "satellite/EphemerisSelector.hpp"
#include "common/wgs84Constants.hpp"

#include <cmath>
#include <limits>

namespace gnss
{

//------------------------------------------------------------------------------//
//                        Ephemeris Selection                                    //
//------------------------------------------------------------------------------//

const rinex::GpsBroadcastEphemeris *EphemerisSelector::selectBestEphemerisForSatellite(
    const std::vector<rinex::GpsBroadcastEphemeris> &ephemerides,
    SatId satelliteId,
    double timeOfWeekSeconds,
    double maximumAgeSeconds)
{
    const rinex::GpsBroadcastEphemeris *bestEphemeris = nullptr;
    double smallestTimeDifference = std::numeric_limits<double>::max();

    for (const auto &ephemeris : ephemerides)
    {
        // Must match satellite
        if (!(ephemeris.satelliteId == satelliteId))
        {
            continue;
        }

        // Must be healthy
        if (ephemeris.satelliteHealth != 0.0)
        {
            continue;
        }

        // Compute |t - Toe| with GPS week rollover handling
        double timeDifference = timeOfWeekSeconds - ephemeris.timeOfEphemeris;
        if (timeDifference > wgs84::HALF_WEEK_SECONDS)
        {
            timeDifference -= wgs84::SECONDS_PER_WEEK;
        }
        else if (timeDifference < -wgs84::HALF_WEEK_SECONDS)
        {
            timeDifference += wgs84::SECONDS_PER_WEEK;
        }

        const double absoluteTimeDifference = std::fabs(timeDifference);

        // Must be within maximum age
        if (absoluteTimeDifference > maximumAgeSeconds)
        {
            continue;
        }

        // Prefer the closest Toe; break ties by larger IODE (more recent upload)
        if (absoluteTimeDifference < smallestTimeDifference
            || (absoluteTimeDifference == smallestTimeDifference
                && bestEphemeris != nullptr
                && ephemeris.issueOfDataEphemeris > bestEphemeris->issueOfDataEphemeris))
        {
            smallestTimeDifference = absoluteTimeDifference;
            bestEphemeris = &ephemeris;
        }
    }

    return bestEphemeris;
}

} // namespace gnss
