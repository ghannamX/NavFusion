#include "time/GnssTime.hpp"
#include "common/wgs84Constants.hpp"

#include <cmath>

namespace gnss
{

//------------------------------------------------------------------------------//
//                     GpsTime Arithmetic                                       //
//------------------------------------------------------------------------------//

GpsTime GpsTime::addSeconds(double seconds) const noexcept
{
    double newTimeOfWeek = timeOfWeek + seconds;
    int    newWeek       = weekNumber;

    // Handle week rollovers in both directions.
    while (newTimeOfWeek >= wgs84::SECONDS_PER_WEEK)
    {
        newTimeOfWeek -= wgs84::SECONDS_PER_WEEK;
        newWeek += 1;
    }
    while (newTimeOfWeek < 0.0)
    {
        newTimeOfWeek += wgs84::SECONDS_PER_WEEK;
        newWeek -= 1;
    }

    return GpsTime{newWeek, newTimeOfWeek};
}

double GpsTime::differenceInSeconds(const GpsTime & other) const noexcept
{
    const double thisTotal  = static_cast<double>(weekNumber) * wgs84::SECONDS_PER_WEEK
                            + timeOfWeek;
    const double otherTotal = static_cast<double>(other.weekNumber) * wgs84::SECONDS_PER_WEEK
                            + other.timeOfWeek;
    return thisTotal - otherTotal;
}

//------------------------------------------------------------------------------//
//                     DateTime → GPS Time                                      //
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

    // MJD = JD - 2400000.5; since julianDayNumber is the integer JD,
    // we use MJD = julianDayNumber - 2400001 for the integer part.
    const int modifiedJulianDay = julianDayNumber - 2400001;

    // GPS epoch: January 6, 1980 = MJD 44244
    const int daysSinceGpsEpoch = modifiedJulianDay - 44244;

    const int gpsWeek   = daysSinceGpsEpoch / 7;
    const int dayOfWeek = daysSinceGpsEpoch - gpsWeek * 7;

    const double secondsOfWeek =
        static_cast<double>(dayOfWeek) * 86400.0
      + static_cast<double>(hour_)   * 3600.0
      + static_cast<double>(minute_) * 60.0
      + second_;

    return GpsTime{gpsWeek, secondsOfWeek};
}

//------------------------------------------------------------------------------//
//                     GPS Time → DateTime                                      //
//------------------------------------------------------------------------------//

DateTime DateTime::fromGpsTime(const GpsTime & gpsTime) noexcept
{
    // Total days and remaining seconds since GPS epoch (MJD 44244).
    const int    totalDays      = gpsTime.weekNumber * 7
                                + static_cast<int>(gpsTime.timeOfWeek / 86400.0);
    const double remainingSeconds = gpsTime.timeOfWeek
                                  - static_cast<double>(static_cast<int>(
                                        gpsTime.timeOfWeek / 86400.0)) * 86400.0;

    // MJD of the date part.
    const int modifiedJulianDay = 44244 + totalDays;

    // Convert MJD → Gregorian calendar (Algorithm 199, Meeus §7).
    // JD integer = MJD + 2400001 (the 0.5 day offset is absorbed into the integer JD).
    const int julianDayNumber = modifiedJulianDay + 2400001;

    const int alpha = static_cast<int>((julianDayNumber - 1867216.25) / 36524.25);
    const int a     = julianDayNumber + 1 + alpha - alpha / 4;
    const int b     = a + 1524;
    const int c     = static_cast<int>((b - 122.1) / 365.25);
    const int d     = static_cast<int>(365.25 * c);
    const int e     = static_cast<int>((b - d) / 30.6001);

    const int day   = b - d - static_cast<int>(30.6001 * e);
    const int month = (e < 14) ? (e - 1) : (e - 13);
    const int year  = (month > 2) ? (c - 4716) : (c - 4715);

    // Decompose remaining seconds into h, m, s.
    const int    hour   = static_cast<int>(remainingSeconds / 3600.0);
    const double afterH = remainingSeconds - static_cast<double>(hour) * 3600.0;
    const int    minute = static_cast<int>(afterH / 60.0);
    const double second = afterH - static_cast<double>(minute) * 60.0;

    DateTime dt;
    dt.year_   = year;
    dt.month_  = month;
    dt.day_    = day;
    dt.hour_   = hour;
    dt.minute_ = minute;
    dt.second_ = second;
    return dt;
}

//------------------------------------------------------------------------------//
//                     Total GPS Seconds                                        //
//------------------------------------------------------------------------------//

double DateTime::convertToTotalGpsSeconds() const noexcept
{
    const auto gpsTime = convertToGpsTime();
    return static_cast<double>(gpsTime.weekNumber) * wgs84::SECONDS_PER_WEEK
         + gpsTime.timeOfWeek;
}

double DateTime::differenceInSeconds(const DateTime & other) const noexcept
{
    return convertToTotalGpsSeconds() - other.convertToTotalGpsSeconds();
}

//------------------------------------------------------------------------------//
//                     Leap Second Utilities                                    //
//------------------------------------------------------------------------------//

namespace
{

/*!
 * \brief Returns the number of GPS–UTC leap seconds for a given GPST date.
 *
 * Leap second table (GPS is ahead of UTC by ΔtLS seconds):
 *
 *   GPST date of insertion   ΔtLS
 *   1981-07-01 00:00:01        1
 *   1982-07-01 00:00:02        2
 *   1983-07-01 00:00:03        3
 *   1985-07-01 00:00:04        4
 *   1988-01-01 00:00:05        5
 *   1990-01-01 00:00:06        6
 *   1991-01-01 00:00:07        7
 *   1992-07-01 00:00:08        8
 *   1993-07-01 00:00:09        9
 *   1994-07-01 00:00:10       10
 *   1996-01-01 00:00:11       11
 *   1997-07-01 00:00:12       12
 *   1999-01-01 00:00:13       13
 *   2006-01-01 00:00:14       14
 *   2009-01-01 00:00:15       15
 *   2012-07-01 00:00:16       16
 *   2015-07-01 00:00:17       17
 *   2017-01-01 00:00:18       18  ← current as of 2026
 *
 * The table is encoded as (year, month, day, leapSeconds) tuples.
 * Each entry is the first GPST moment that carries that ΔtLS value.
 */
int leapSecondsForGpstDate(int year, int month, int day)
{
    struct LeapEntry
    {
        int year;
        int month;
        int day;
        int leapSeconds;
    };

    static constexpr LeapEntry table[] = {
        {2017, 1, 1, 18},
        {2015, 7, 1, 17},
        {2012, 7, 1, 16},
        {2009, 1, 1, 15},
        {2006, 1, 1, 14},
        {1999, 1, 1, 13},
        {1997, 7, 1, 12},
        {1996, 1, 1, 11},
        {1994, 7, 1, 10},
        {1993, 7, 1,  9},
        {1992, 7, 1,  8},
        {1991, 1, 1,  7},
        {1990, 1, 1,  6},
        {1988, 1, 1,  5},
        {1985, 7, 1,  4},
        {1983, 7, 1,  3},
        {1982, 7, 1,  2},
        {1981, 7, 1,  1},
    };

    // Walk from newest to oldest; return the first entry that is <= the input date.
    for (const auto & entry : table)
    {
        const bool after =  year > entry.year
                         || (year == entry.year && month > entry.month)
                         || (year == entry.year && month == entry.month && day >= entry.day);
        if (after)
        {
            return entry.leapSeconds;
        }
    }
    return 0;
}

} // anonymous namespace

//------------------------------------------------------------------------------//
//                     GPST ↔ UTC Conversions                                  //
//------------------------------------------------------------------------------//

DateTime DateTime::convertGpstToUtc(const DateTime & gpst) noexcept
{
    // t_UTC = t_GPST − ΔtLS
    const int leapSeconds = leapSecondsForGpstDate(gpst.year_, gpst.month_, gpst.day_);

    // Subtract leap seconds via GPS time arithmetic to handle day/week rollovers.
    const GpsTime gpstTime = gpst.convertToGpsTime();
    const GpsTime utcTime  = gpstTime.addSeconds(-static_cast<double>(leapSeconds));
    return DateTime::fromGpsTime(utcTime);
}

DateTime DateTime::convertUtcToGpst(const DateTime & utc) noexcept
{
    // t_GPST = t_UTC + ΔtLS
    // Use UTC date for the lookup — close enough for all historical leap seconds
    // because the difference is only ever a whole number of seconds.
    const int leapSeconds = leapSecondsForGpstDate(utc.year_, utc.month_, utc.day_);

    const GpsTime utcTime  = utc.convertToGpsTime();
    const GpsTime gpstTime = utcTime.addSeconds(static_cast<double>(leapSeconds));
    return DateTime::fromGpsTime(gpstTime);
}

} // namespace gnss
