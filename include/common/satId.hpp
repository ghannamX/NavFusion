#pragma once

#include <cstdint>
#include <format>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

namespace gnss
{

//------------------------------------------------------------//
//                  Constellation System                      //
//------------------------------------------------------------//

/*!
 * \brief RINEX 3 single-character satellite constellation identifier.
 *
 * Each enumerator stores the exact ASCII character used in RINEX 3 system codes,
 * allowing a direct cast to \c char for string formatting and file serialisation.
 */
enum class Constellation : char
{
    GPS     = 'G', /*!< Global Positioning System (USA) */
    GLONASS = 'R', /*!< GLObal NAvigation Satellite System (Russia) */
    Galileo = 'E', /*!< Galileo (European Union) */
    BeiDou  = 'C', /*!< BeiDou Navigation Satellite System (China) */
    QZSS    = 'J', /*!< Quasi-Zenith Satellite System (Japan) */
    SBAS    = 'S', /*!< Satellite-Based Augmentation System */
    NavIC   = 'I', /*!< Navigation with Indian Constellation (India) */
};

//------------------------------------------------------------//
//                    Satellite Identifier                    //
//------------------------------------------------------------//

/*!
 * \brief Compact satellite identifier combining constellation system and PRN number.
 *
 * Packs system and PRN into a \c uint16_t (upper byte = constellation char,
 * lower byte = PRN) to enable O(1) hashing and single-integer comparison across
 * all processing modules.
 *
 * RINEX 3 format: 3-character string — system letter + 2-digit PRN.
 * Examples: "G01" (GPS PRN 1), "E05" (Galileo PRN 5), "C21" (BeiDou PRN 21).
 */
struct SatId
{
    //------------------------------------------------------------//
    //                     Member Variables                       //
    //------------------------------------------------------------//

    Constellation system; /*!< Constellation to which this satellite belongs */
    std::uint8_t  prn;    /*!< 1-based PRN number (GPS: 1–32, Galileo: 1–36, BeiDou: 1–63) */

    //------------------------------------------------------------//
    //                   Packing and Comparison                   //
    //------------------------------------------------------------//

    /*!
     * \brief Pack the satellite identifier into a \c uint16_t.
     *
     * Upper byte = constellation char value, lower byte = PRN.
     * Used by the hash specialisation and comparison operators.
     *
     * \return Packed 16-bit representation of this satellite identifier.
     */
    constexpr std::uint16_t packIntoUint16() const noexcept
    {
        return (static_cast<std::uint16_t>(static_cast<unsigned char>(system)) << 8)
             | static_cast<std::uint16_t>(prn);
    }

    constexpr bool operator==(const SatId &) const noexcept = default;

    constexpr bool operator<(const SatId &other) const noexcept
    {
        return packIntoUint16() < other.packIntoUint16();
    }

    //------------------------------------------------------------//
    //                   RINEX 3 Serialisation                    //
    //------------------------------------------------------------//

    /*!
     * \brief Parse a RINEX 3 3-character satellite identifier string.
     *
     * Accepts strings of the form "G01", "R07", "E36", "C21", etc.
     * Returns \c std::nullopt if the string is shorter than 3 characters,
     * the system character is unrecognised, the PRN digits are non-numeric,
     * or the PRN value is zero.
     *
     * \param[in] rinex3String  View over the 3-character satellite identifier.
     * \return                  Parsed \c SatId, or \c std::nullopt on malformed input.
     */
    static std::optional<SatId> parseSatelliteIdFromRinex3String(
        std::string_view rinex3String) noexcept
    {
        if (rinex3String.size() < 3)
            return std::nullopt;

        Constellation sys;
        switch (rinex3String[0])
        {
            case 'G': sys = Constellation::GPS;     break;
            case 'R': sys = Constellation::GLONASS; break;
            case 'E': sys = Constellation::Galileo; break;
            case 'C': sys = Constellation::BeiDou;  break;
            case 'J': sys = Constellation::QZSS;    break;
            case 'S': sys = Constellation::SBAS;    break;
            case 'I': sys = Constellation::NavIC;   break;
            default:  return std::nullopt;
        }

        if (rinex3String[1] < '0' || rinex3String[1] > '9')
            return std::nullopt;
        if (rinex3String[2] < '0' || rinex3String[2] > '9')
            return std::nullopt;

        const auto parsedPrn = static_cast<std::uint8_t>(
            (rinex3String[1] - '0') * 10 + (rinex3String[2] - '0'));

        if (parsedPrn == 0)
            return std::nullopt;

        return SatId{sys, parsedPrn};
    }

    /*!
     * \brief Format the satellite identifier as a RINEX 3 3-character string.
     *
     * Produces strings of the form "G01", "E05", "C21", etc.
     *
     * \return 3-character RINEX 3 satellite identifier string.
     */
    std::string formatSatelliteIdAsRinex3String() const
    {
        return std::format("{}{:02d}", static_cast<char>(system), static_cast<int>(prn));
    }
};

} // namespace gnss

//------------------------------------------------------------//
//              std::hash Specialisation for SatId            //
//------------------------------------------------------------//

/*!
 * \brief Hash specialisation enabling \c gnss::SatId as an \c unordered_map key.
 *
 * Delegates to \c std::hash<uint16_t> via \c SatId::packIntoUint16().
 */
template <>
struct std::hash<gnss::SatId>
{
    std::size_t operator()(gnss::SatId satelliteId) const noexcept
    {
        return std::hash<std::uint16_t>{}(satelliteId.packIntoUint16());
    }
};
