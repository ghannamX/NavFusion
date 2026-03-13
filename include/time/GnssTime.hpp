#pragma once

#include <compare>
#include <cstdint>
#include <utility>

namespace gnss
{

//------------------------------------------------------------------------------//
//                       GPS Week and Seconds of Week                           //
//------------------------------------------------------------------------------//

/*!
 * \brief GPS time expressed as week number and seconds within that week.
 *
 * GPS Week 0 started at midnight UTC on January 6, 1980.
 * Seconds of week range from 0.0 to 604799.999...
 * The week number is continuous (not modulo 1024).
 */
struct GpsTime
{
    int    weekNumber = 0;   /*!< Continuous GPS week number (0 = Jan 6 1980). */
    double timeOfWeek = 0.0; /*!< Seconds elapsed within the GPS week [0, 604800). */

    //--------------------------------------------------------------------------//
    //                          Arithmetic                                      //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Return a new GpsTime advanced by the given number of seconds.
     *
     * Handles week rollovers when the offset crosses a week boundary.
     * Used by the transmission time solver to step backwards by the signal
     * travel time on each iteration.
     *
     * \param[in] seconds  Time offset to add [s]. May be negative.
     * \return New GpsTime = this + seconds, with week boundaries handled.
     */
    GpsTime addSeconds(double seconds) const noexcept;

    /*!
     * \brief Difference in seconds between this epoch and another.
     *
     * Returns (this - other) in seconds.
     *
     * \param[in] other  Epoch to subtract.
     * \return Signed time difference [s]: positive if this > other.
     */
    double differenceInSeconds(const GpsTime & other) const noexcept;
};

//------------------------------------------------------------------------------//
//                              Date and Time                                   //
//------------------------------------------------------------------------------//

/*!
 * \brief Calendar date and time representation for GNSS epochs.
 *
 * Stores a timestamp exactly as read from RINEX 3 observation and navigation
 * files — year, month, day, hour, minute, and fractional second. The time
 * system (GPST, UTC, Galileo System Time, BeiDou Time) is declared in the
 * RINEX file header and is not encoded here.
 *
 * ## Conversions
 *
 * DateTime → GpsTime uses the Modified Julian Day (MJD) algorithm:
 *
 *   JD  = INT(365.25 * (y + 4716)) + INT(30.6001 * (m + 1)) + d − 1537
 *   MJD = JD − 2400001
 *   daysSinceGpsEpoch = MJD − 44244          (GPS epoch = MJD 44244)
 *   week = daysSinceGpsEpoch / 7
 *   ToW  = (daysSinceGpsEpoch mod 7) * 86400 + h*3600 + min*60 + sec
 *
 * GpsTime → DateTime inverts this via:
 *
 *   MJD = 44244 + week*7 + floor(ToW / 86400)
 *   JD  = MJD + 2400001
 *   then Gregorian calendar from JD (Algorithm 199, Meeus)
 *
 * GPST → UTC subtracts the accumulated leap seconds:
 *
 *   t_UTC = t_GPST − ΔtLS
 *
 * where ΔtLS is looked up from the historical leap second table.
 * As of 2017-01-01, ΔtLS = 18 s (GPS is 18 s ahead of UTC).
 *
 * The three-way comparison operator enables sorting epoch vectors and
 * building time-ordered observation archives.
 */
class DateTime
{
public:
    //--------------------------------------------------------------------------//
    //                          Member Variables                                //
    //--------------------------------------------------------------------------//

    std::int32_t year_   = 0;   /*!< Calendar year (e.g. 2026) */
    std::int32_t month_  = 0;   /*!< Month of year  : 1 (January) – 12 (December) */
    std::int32_t day_    = 0;   /*!< Day of month   : 1-based */
    std::int32_t hour_   = 0;   /*!< Hour of day    : 0–23 */
    std::int32_t minute_ = 0;   /*!< Minute of hour : 0–59 */
    double       second_ = 0.0;
    /*!< Second of minute with sub-second precision : 0.0 – 59.9999999.
     *   RINEX 3 epoch records provide 7 decimal places (100-ns resolution). */

    //--------------------------------------------------------------------------//
    //                        Validation                                        //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Returns true if all calendar fields are within physically valid ranges.
     *
     * Checks:
     *  - year   >= 1980 (GPS Week 0 started on January 6, 1980)
     *  - month  in [1, 12]
     *  - day    in [1, 31]
     *  - hour   in [0, 23]
     *  - minute in [0, 59]
     *  - second in [0.0, 61.0) — allows for positive leap seconds
     *
     * A default-constructed DateTime (all fields zero) returns false.
     *
     * \return true if all fields are within the valid GNSS epoch ranges.
     */
    bool isValidEpoch() const noexcept
    {
        if (year_   < 1980) return false;
        if (month_  < 1 || month_  > 12) return false;
        if (day_    < 1 || day_    > 31) return false;
        if (hour_   < 0 || hour_   > 23) return false;
        if (minute_ < 0 || minute_ > 59) return false;
        if (second_ < 0.0 || second_ >= 61.0) return false;
        return true;
    }

    //--------------------------------------------------------------------------//
    //                     GPS Time Conversions                                 //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Convert this calendar epoch to GPS week number and seconds of week.
     *
     * Assumes the stored time is already in GPS time system (GPST). No leap
     * second correction is applied.
     *
     * \return GpsTime with continuous week number and seconds within that week.
     */
    GpsTime convertToGpsTime() const noexcept;

    /*!
     * \brief Construct a DateTime from GPS week number and seconds of week.
     *
     * Inverts convertToGpsTime(). The result is in GPST (no leap second added).
     *
     * \param[in] gpsTime  GPS week number and seconds of week.
     * \return Equivalent calendar DateTime in GPST.
     */
    static DateTime fromGpsTime(const GpsTime & gpsTime) noexcept;

    /*!
     * \brief Convert this calendar epoch to total GPS seconds since the GPS epoch.
     *
     * Returns week * 604800.0 + timeOfWeek. Useful for computing time
     * differences between two epochs without dealing with week boundaries.
     *
     * \return Total GPS seconds since January 6, 1980 00:00:00 GPST.
     */
    double convertToTotalGpsSeconds() const noexcept;

    /*!
     * \brief Signed time difference in seconds: this − other.
     *
     * Converts both epochs to total GPS seconds and subtracts.
     *
     * \param[in] other  Epoch to subtract.
     * \return (this − other) in seconds. Positive if this is later.
     */
    double differenceInSeconds(const DateTime & other) const noexcept;

    //--------------------------------------------------------------------------//
    //                     UTC Conversions                                      //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Convert a GPST DateTime to UTC by subtracting accumulated leap seconds.
     *
     * ## Leap second model
     *
     *   t_UTC = t_GPST − ΔtLS
     *
     * where ΔtLS is the integer number of leap seconds inserted between UTC and
     * GPST since the GPS epoch. The table below lists all leap second events:
     *
     *   Date (UTC)       ΔtLS
     *   1981-07-01        1
     *   1982-07-01        2
     *   1983-07-01        3
     *   1985-07-01        4
     *   1988-01-01        5
     *   1990-01-01        6
     *   1991-01-01        7
     *   1992-07-01        8
     *   1993-07-01        9
     *   1994-07-01       10
     *   1996-01-01       11
     *   1997-07-01       12
     *   1999-01-01       13
     *   2006-01-01       14
     *   2009-01-01       15
     *   2012-07-01       16
     *   2015-07-01       17
     *   2017-01-01       18  ← current as of 2026
     *
     * \param[in] gpst  Calendar epoch in GPS time system.
     * \return Equivalent UTC DateTime.
     */
    static DateTime convertGpstToUtc(const DateTime & gpst) noexcept;

    /*!
     * \brief Convert a UTC DateTime to GPST by adding accumulated leap seconds.
     *
     * Inverse of convertGpstToUtc().
     *
     * \param[in] utc  Calendar epoch in UTC.
     * \return Equivalent GPST DateTime.
     */
    static DateTime convertUtcToGpst(const DateTime & utc) noexcept;

    //--------------------------------------------------------------------------//
    //                       Comparison Operators                               //
    //--------------------------------------------------------------------------//

    /*! \brief Three-way comparison for sorting and range checking of epochs. */
    auto operator<=>(const DateTime &) const noexcept = default;

    /*! \brief Equality comparison. */
    bool operator==(const DateTime &) const noexcept = default;
};

} // namespace gnss
