#include "time/GnssTime.hpp"
#include "common/wgs84Constants.hpp"

#include <cmath>

namespace gnss
{

//------------------------------------------------------------------------------//
//                     GPS Time Conversions                                     //
//------------------------------------------------------------------------------//

GpsTime DateTime::convertToGpsTime() const noexcept
{
    // Convert calendar date to Modified Julian Day (MJD).
    // Algorithm from Meeus, "Astronomical Algorithms", valid for dates after
    // March 1900 when using integer arithmetic.

    int y = year_;
    int m = month_;

    if (m <= 2)
    {
        y -= 1;
        m += 12;
    }

    const int julianDayNumber =
        static_cast<int>(365.25 * (y + 4716))
      + static_cast<int>(30.6001 * (m + 1))
      + day_ - 1537;

    // MJD = JD - 2400000.5, but since julianDayNumber is the integer JD,
    // we use MJD = julianDayNumber - 2400001 for the integer part.
    const int modifiedJulianDay = julianDayNumber - 2400001;

    // GPS epoch: January 6, 1980 = MJD 44244
    const int daysSinceGpsEpoch = modifiedJulianDay - 44244;

    const int gpsWeek = daysSinceGpsEpoch / 7;
    const int dayOfWeek = daysSinceGpsEpoch - gpsWeek * 7;

    const double secondsOfWeek =
        static_cast<double>(dayOfWeek) * 86400.0
      + static_cast<double>(hour_) * 3600.0
      + static_cast<double>(minute_) * 60.0
      + second_;

    return GpsTime{gpsWeek, secondsOfWeek};
}

double DateTime::convertToTotalGpsSeconds() const noexcept
{
    const auto gpsTime = convertToGpsTime();
    return static_cast<double>(gpsTime.weekNumber) * wgs84::SECONDS_PER_WEEK
         + gpsTime.timeOfWeek;
}

} // namespace gnss
