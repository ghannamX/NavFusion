#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace gnss
{

//------------------------------------------------------------//
//                   Observation Measurement                  //
//------------------------------------------------------------//

/*!
 * \brief A single RINEX 3 observation value together with its quality indicators.
 *
 * Represents one 16-character measurement field from a RINEX 3 satellite data line:
 *  - 14 characters: numeric value — pseudorange / carrier phase in metres,
 *                   Doppler in Hz, or signal-to-noise ratio in dB-Hz.
 *  - 1  character : Loss-of-Lock Indicator (LLI) — see RINEX 3 spec Table 7.
 *  - 1  character : signal strength on a 1–9 scale — see RINEX 3 spec Table 8.
 *
 * A \c value of NaN (the default) indicates the observation was absent —
 * i.e. the 14-character numeric field was entirely blank in the RINEX file.
 *
 * Memory layout: 10 bytes (8-byte double + 2 bytes), naturally aligned.
 */
struct ObsMeasurement
{
    //------------------------------------------------------------//
    //                      Member Variables                      //
    //------------------------------------------------------------//

    double value = std::numeric_limits<double>::quiet_NaN();
    /*!< Observation value in metres (pseudorange / carrier phase),
     *   Hz (Doppler), or dB-Hz (signal strength).
     *   NaN indicates the field was blank in the RINEX record. */

    std::uint8_t lossOfLockIndicator = 0;
    /*!< RINEX 3 Loss-of-Lock Indicator (LLI) field.
     *   Bit 0 : cycle slip or loss of lock since previous epoch.
     *   Bit 1 : half-cycle ambiguity — carrier phase is valid in magnitude
     *           but the sign of the half-cycle is unknown.
     *   Bits 2–7 : reserved (zero).
     *   A blank LLI character in the file is decoded as 0. */

    std::uint8_t signalStrength = 0;
    /*!< Receiver signal strength indicator (RINEX 3 S field).
     *   Scale: 1 (minimum) to 9 (maximum).
     *   0 = not known or not applicable.
     *   A blank S character in the file is decoded as 0. */

    //------------------------------------------------------------//
    //                  Quality Indicator Methods                 //
    //------------------------------------------------------------//

    /*!
     * \brief Returns \c true if a numeric measurement value is present.
     *
     * A return value of \c false means the 14-character observation field was blank
     * in the RINEX file, indicating the receiver did not track this signal during
     * the epoch.
     *
     * \return \c true if \c value is not NaN.
     */
    bool hasMeasurementValue() const noexcept
    {
        return !std::isnan(value);
    }

    /*!
     * \brief Returns \c true if a cycle slip or loss of lock occurred on this signal.
     *
     * Corresponds to bit 0 of the RINEX 3 LLI field. When set, the carrier phase
     * integer ambiguity must be re-initialised before this measurement can be used
     * in PPP or RTK carrier phase processing.
     *
     * \return \c true if LLI bit 0 is set.
     */
    bool hasLossOfLock() const noexcept
    {
        return (lossOfLockIndicator & 0x01) != 0;
    }

    /*!
     * \brief Returns \c true if a half-cycle ambiguity is present on the carrier phase.
     *
     * Corresponds to bit 1 of the RINEX 3 LLI field. When set, the carrier phase
     * measurement is valid in magnitude but the sign of the half-cycle is unknown.
     * This requires special handling in integer ambiguity resolution.
     *
     * \return \c true if LLI bit 1 is set.
     */
    bool isHalfCycleAmbiguity() const noexcept
    {
        return (lossOfLockIndicator & 0x02) != 0;
    }
};

} // namespace gnss
