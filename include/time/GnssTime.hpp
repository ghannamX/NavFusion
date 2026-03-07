#pragma once

#include <compare>
#include <cstdint>

namespace gnss
{

//------------------------------------------------------------//
//                       GNSS Calendar Time                   //
//------------------------------------------------------------//

/*!
 * \brief Minimal GNSS epoch timestamp in calendar format.
 *
 * Stores a GNSS timestamp exactly as read from RINEX 3 observation and navigation
 * files — year, month, day, hour, minute, and fractional second. The time system
 * (GPST, UTC, Galileo System Time, BeiDou Time) is declared in the RINEX file header
 * and is not encoded here.
 *
 * This struct will be extended in Phase 1 of the implementation with:
 *  - GPS week number and time-of-week (ToW)
 *  - UTC ↔ GPST conversion (requires leap-second table)
 *  - Galileo System Time (GST) and BeiDou Time (BDT) conversions
 *  - Julian date and Modified Julian Date (MJD) for orbit interpolation
 *
 * The three-way comparison operator enables sorting epoch vectors and
 * building time-ordered observation archives.
 */
struct GnssTime
{
    //------------------------------------------------------------//
    //                      Member Variables                      //
    //------------------------------------------------------------//

    std::int32_t year   = 0; /*!< Calendar year (e.g. 2023) */
    std::int32_t month  = 0; /*!< Month of year  : 1 (January) – 12 (December) */
    std::int32_t day    = 0; /*!< Day of month   : 1-based */
    std::int32_t hour   = 0; /*!< Hour of day    : 0–23 */
    std::int32_t minute = 0; /*!< Minute of hour : 0–59 */
    double       second = 0.0;
    /*!< Second of minute with sub-second precision : 0.0 – 59.9999999.
     *   RINEX 3 epoch records provide 7 decimal places (100-ns resolution). */

    //------------------------------------------------------------//
    //                     Validation Methods                     //
    //------------------------------------------------------------//

    /*!
     * \brief Returns \c true if all calendar fields are within physically valid ranges.
     *
     * Checks that the epoch could correspond to a real GNSS measurement:
     *  - \b year   >= 1980 — GPS Week 0 started on January 6, 1980; no GNSS
     *                        data exists before this date.
     *  - \b month  in [1, 12]
     *  - \b day    in [1, 31]  — upper bound is conservative; month-specific
     *                            day-count validation is deferred to a full
     *                            calendar library in a later phase.
     *  - \b hour   in [0, 23]
     *  - \b minute in [0, 59]
     *  - \b second in [0.0, 60.0) — the open upper bound allows for a positive
     *                               leap second (second == 60.0 is valid during
     *                               a UTC leap-second insertion event).
     *
     * A default-constructed \c GnssTime (all fields zero) returns \c false.
     *
     * \return \c true if all fields are within the valid GNSS epoch ranges.
     */
    bool isValidEpoch() const noexcept
    {
        if (year   < 1980) return false;   // before GPS Week 0 (January 6, 1980)
        if (month  < 1 || month  > 12) return false;
        if (day    < 1 || day    > 31) return false;
        if (hour   < 0 || hour   > 23) return false;
        if (minute < 0 || minute > 59) return false;
        if (second < 0.0 || second >= 61.0) return false;  // 60.0 allowed for leap seconds
        return true;
    }

    //------------------------------------------------------------//
    //                    Comparison Operators                    //
    //------------------------------------------------------------//

    /*! \brief Three-way comparison for sorting and range checking of epochs. */
    auto operator<=>(const GnssTime &) const noexcept = default;

    /*! \brief Equality comparison. */
    bool operator==(const GnssTime &) const noexcept = default;
};

} // namespace gnss
