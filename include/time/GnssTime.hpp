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
    int    weekNumber    = 0;   /*!< Continuous GPS week number (0 = Jan 6 1980). */
    double timeOfWeek    = 0.0; /*!< Seconds elapsed within the GPS week [0, 604800). */
};

//------------------------------------------------------------------------------//
//                              Date and Time                                   //
//------------------------------------------------------------------------------//

/*!
 * \brief Calendar date and time representation for GNSS epochs.
 *
 * Stores a timestamp exactly as read from RINEX 3 observation and navigation
 * files — year, month, day, hour, minute, and fractional second. The time system
 * (GPST, UTC, Galileo System Time, BeiDou Time) is declared in the RINEX file header
 * and is not encoded here.
 *
 * Provides conversion to GPS week and seconds of week for orbit/clock computations.
 *
 * The three-way comparison operator enables sorting epoch vectors and
 * building time-ordered observation archives.
 */
class DateTime
{
public:
    //--------------------------------------------------------------------------//
    //                          Member Variables                                 //
    //--------------------------------------------------------------------------//

    std::int32_t year_   = 0;     /*!< Calendar year (e.g. 2023) */
    std::int32_t month_  = 0;     /*!< Month of year  : 1 (January) - 12 (December) */
    std::int32_t day_    = 0;     /*!< Day of month   : 1-based */
    std::int32_t hour_   = 0;     /*!< Hour of day    : 0-23 */
    std::int32_t minute_ = 0;     /*!< Minute of hour : 0-59 */
    double       second_ = 0.0;
    /*!< Second of minute with sub-second precision : 0.0 - 59.9999999.
     *   RINEX 3 epoch records provide 7 decimal places (100-ns resolution). */

    //--------------------------------------------------------------------------//
    //                        Validation Methods                                //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Returns true if all calendar fields are within physically valid ranges.
     *
     * Checks that the epoch could correspond to a real GNSS measurement:
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
    //                     GPS Time Conversions                                  //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Convert this calendar epoch to GPS week number and seconds of week.
     *
     * Uses the algorithm: compute Modified Julian Day (MJD) from the calendar
     * date, then convert to GPS week and seconds relative to the GPS epoch
     * (January 6, 1980 = MJD 44244).
     *
     * Assumes the stored time is already in GPS time system (GPST). No leap
     * second correction is applied.
     *
     * \return GPS week number (continuous) and seconds within that week.
     */
    GpsTime convertToGpsTime() const noexcept;

    /*!
     * \brief Convert this calendar epoch to total GPS seconds since the GPS epoch.
     *
     * Returns (week * 604800.0 + seconds). Useful for computing time
     * differences between two epochs.
     *
     * \return Total GPS seconds since January 6, 1980 00:00:00 GPST.
     */
    double convertToTotalGpsSeconds() const noexcept;

    //--------------------------------------------------------------------------//
    //                       Comparison Operators                                //
    //--------------------------------------------------------------------------//

    /*! \brief Three-way comparison for sorting and range checking of epochs. */
    auto operator<=>(const DateTime &) const noexcept = default;

    /*! \brief Equality comparison. */
    bool operator==(const DateTime &) const noexcept = default;
};

} // namespace gnss
