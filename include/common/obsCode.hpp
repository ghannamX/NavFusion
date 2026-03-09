#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

namespace gnss
{

//------------------------------------------------------------------------------//
//                            Observation Code                                  //
//------------------------------------------------------------------------------//

/*!
 * \brief RINEX 3 three-character observation code: [type][band][attribute].
 *
 * Each GNSS signal measurement is uniquely identified by three characters:
 *
 *  type      : measurement type
 *   - C = pseudorange (code measurement, in metres)
 *   - L = carrier phase (in cycles)
 *   - D = Doppler frequency shift (in Hz)
 *   - S = signal-to-noise ratio (in dB-Hz)
 *
 *  band      : frequency band digit — 1, 2, 5, 6, 7, 8, 9
 *
 *  attribute : tracking mode / channel — C, W, Q, X, P, A, S, L, I, D, B, Z, Y, M, N
 *
 * The three characters are packed into a uint32_t (one char per byte, 4th byte = 0)
 * for O(1) hash and single-integer equality test — no heap allocation, no string overhead.
 * The design is open-ended: any future RINEX band digit or attribute letter is representable
 * without modifying this struct.
 *
 * Examples:
 *  - C1C  GPS L1 C/A pseudorange
 *  - L2W  GPS L2 P(Y)-code carrier phase
 *  - D5Q  GPS L5 Q-channel Doppler
 *  - S1C  GPS L1 C/A signal-to-noise ratio
 */
struct ObsCode
{
    //--------------------------------------------------------------------------//
    //                            Member Variables                               //
    //--------------------------------------------------------------------------//

    char type; /*!< Measurement type   : C | L | D | S */
    char band; /*!< Frequency band     : '1' | '2' | '5' | '6' | '7' | '8' | '9' */
    char attr; /*!< Signal attribute   : C | W | Q | X | P | A | S | L | I | D | B | Z | Y | M | N */

    //--------------------------------------------------------------------------//
    //                          Packing and Comparison                           //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Pack the observation code into a uint32_t for fast hash and comparison.
     *
     * Byte 0 = type, byte 1 = band, byte 2 = attr, byte 3 = 0 (padding).
     *
     * \return Packed 32-bit representation of this observation code.
     */
    constexpr std::uint32_t packIntoUint32() const noexcept
    {
        return static_cast<std::uint32_t>(static_cast<unsigned char>(type))
             | (static_cast<std::uint32_t>(static_cast<unsigned char>(band)) << 8)
             | (static_cast<std::uint32_t>(static_cast<unsigned char>(attr)) << 16);
    }

    constexpr bool operator==(const ObsCode &) const noexcept = default;

    //--------------------------------------------------------------------------//
    //                          RINEX 3 Serialisation                            //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Parse a RINEX 3 3-character observation code string.
     *
     * Accepts strings of the form "C1C", "L2W", "D5Q", "S1C", etc.
     * No validation is applied to the individual character values so that
     * future RINEX revisions introducing new band digits or attribute letters
     * are handled transparently.
     * Returns std::nullopt only if the input is shorter than 3 characters.
     *
     * \param[in] rinex3String  View over the 3-character observation code.
     * \return                  Parsed ObsCode, or std::nullopt on malformed input.
     */
    static std::optional<ObsCode> parseObsCodeFromRinex3String(
        std::string_view rinex3String) noexcept
    {
        if (rinex3String.size() < 3)
            return std::nullopt;

        return ObsCode{rinex3String[0], rinex3String[1], rinex3String[2]};
    }

    /*!
     * \brief Format the observation code as a RINEX 3 3-character string.
     *
     * \return 3-character observation code string (e.g. "C1C", "L2W", "D5Q").
     */
    std::string formatObsCodeAsRinex3String() const
    {
        return std::string{type, band, attr};
    }
};

//------------------------------------------------------------------------------//
//                      Named Observation Code Constants                         //
//------------------------------------------------------------------------------//

/*!
 * \brief Predefined ObsCode constants for frequently referenced GNSS signals.
 *
 * Used by observation models, cycle-slip detectors, and linear combination
 * modules (IF, MW, GF) to avoid magic strings throughout the processing pipeline.
 * All values correspond to RINEX 3.04 observation code definitions (Table A2).
 */
namespace obsCodes
{
// GPS L1 band  (f1 = 1575.42 MHz)
inline constexpr ObsCode C1C{'C', '1', 'C'}; /*!< GPS L1 C/A pseudorange */
inline constexpr ObsCode L1C{'L', '1', 'C'}; /*!< GPS L1 C/A carrier phase */
inline constexpr ObsCode D1C{'D', '1', 'C'}; /*!< GPS L1 C/A Doppler */
inline constexpr ObsCode S1C{'S', '1', 'C'}; /*!< GPS L1 C/A signal strength */
inline constexpr ObsCode C1W{'C', '1', 'W'}; /*!< GPS L1 P(Y)-code pseudorange */
inline constexpr ObsCode L1W{'L', '1', 'W'}; /*!< GPS L1 P(Y)-code carrier phase */
inline constexpr ObsCode S1W{'S', '1', 'W'}; /*!< GPS L1 P(Y)-code signal strength */

// GPS L2 band  (f2 = 1227.60 MHz)
inline constexpr ObsCode C2W{'C', '2', 'W'}; /*!< GPS L2 P(Y)-code pseudorange */
inline constexpr ObsCode L2W{'L', '2', 'W'}; /*!< GPS L2 P(Y)-code carrier phase */
inline constexpr ObsCode D2W{'D', '2', 'W'}; /*!< GPS L2 P(Y)-code Doppler */
inline constexpr ObsCode S2W{'S', '2', 'W'}; /*!< GPS L2 P(Y)-code signal strength */
inline constexpr ObsCode C2X{'C', '2', 'X'}; /*!< GPS L2C (M+L) pseudorange */
inline constexpr ObsCode L2X{'L', '2', 'X'}; /*!< GPS L2C (M+L) carrier phase */

// GPS L5 band  (f5 = 1176.45 MHz — civil safety-of-life signal)
inline constexpr ObsCode C5Q{'C', '5', 'Q'}; /*!< GPS L5 Q-channel pseudorange */
inline constexpr ObsCode L5Q{'L', '5', 'Q'}; /*!< GPS L5 Q-channel carrier phase */
inline constexpr ObsCode D5Q{'D', '5', 'Q'}; /*!< GPS L5 Q-channel Doppler */
inline constexpr ObsCode S5Q{'S', '5', 'Q'}; /*!< GPS L5 Q-channel signal strength */
inline constexpr ObsCode C5X{'C', '5', 'X'}; /*!< GPS L5 I+Q pseudorange */
inline constexpr ObsCode L5X{'L', '5', 'X'}; /*!< GPS L5 I+Q carrier phase */

// Galileo E1 band  (1575.42 MHz — same centre frequency as GPS L1)
inline constexpr ObsCode C1X{'C', '1', 'X'}; /*!< Galileo E1 B+C pseudorange */
inline constexpr ObsCode L1X{'L', '1', 'X'}; /*!< Galileo E1 B+C carrier phase */

// Galileo E5a band  (1176.45 MHz — same centre frequency as GPS L5)
inline constexpr ObsCode C5I{'C', '5', 'I'}; /*!< Galileo E5a I-channel pseudorange */
inline constexpr ObsCode L5I{'L', '5', 'I'}; /*!< Galileo E5a I-channel carrier phase */

// BeiDou B1I band  (1561.098 MHz)
inline constexpr ObsCode C2I{'C', '2', 'I'}; /*!< BeiDou B1I pseudorange */
inline constexpr ObsCode L2I{'L', '2', 'I'}; /*!< BeiDou B1I carrier phase */
} // namespace obsCodes

} // namespace gnss

//------------------------------------------------------------------------------//
//                    std::hash Specialisation for ObsCode                       //
//------------------------------------------------------------------------------//

/*!
 * \brief Hash specialisation enabling gnss::ObsCode as an unordered_map key.
 *
 * Delegates to std::hash<uint32_t> via ObsCode::packIntoUint32().
 */
template <>
struct std::hash<gnss::ObsCode>
{
    std::size_t operator()(gnss::ObsCode code) const noexcept
    {
        return std::hash<std::uint32_t>{}(code.packIntoUint32());
    }
};
