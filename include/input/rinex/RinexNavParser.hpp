#pragma once

#include "input/rinex/rinexNavTypes.hpp"
#include "input/rinex/rinexParseResult.hpp"
#include <cstdint>
#include <filesystem>
#include <istream>
#include <string>
#include <string_view>
#include <vector>

namespace gnss::rinex
{

//------------------------------------------------------------------------------//
//                             Parser Options                                    //
//------------------------------------------------------------------------------//

/*!
 * \brief Configuration for the RINEX 3 navigation file parser.
 */
struct RinexNavParserOptions
{
    //--------------------------------------------------------------------------//
    //                            Member Variables                               //
    //--------------------------------------------------------------------------//

    std::vector<Constellation> constellationFilter;
    /*!< If non-empty, only ephemeris records of these constellations are stored.
     *   An empty vector accepts all constellations present in the file.
     *   Example: { Constellation::GPS } for GPS-only processing. */

    std::size_t expectedRecordCount = 0;
    /*!< Pre-allocation hint for the ephemeris vectors. Set to the approximate
     *   number of records in the file to avoid repeated heap reallocation.
     *   0 means no pre-allocation. */

    bool skipMalformedRecords = false;
    /*!< If true, malformed navigation records are silently skipped rather than
     *   causing the parse to fail. Useful for files with minor formatting deviations.
     *   Default: false (strict parsing). */
};

//------------------------------------------------------------------------------//
//                          RINEX 3 Nav Parser                                   //
//------------------------------------------------------------------------------//

/*!
 * \brief Stateless parser for RINEX 3.03 and 3.04 navigation message files.
 *
 * Construct once with options, then call the parse methods as many times as needed.
 * Supports both file-path input and stream input for unit testability.
 *
 * Currently parses GPS broadcast ephemeris records (8-line Keplerian format).
 * Non-GPS records (GLONASS, Galileo, BeiDou, etc.) are skipped with a log message.
 *
 * RINEX 3 navigation file structure:
 *  - Header: records with a 60-character content field followed by a 20-character
 *    label (columns 60-79). Terminated by the END OF HEADER record.
 *  - Data records: each begins with a satellite ID + epoch line (line 0),
 *    followed by N broadcast orbit lines. For GPS: 7 orbit lines (8 lines total),
 *    each containing 4 x D19.12 Fortran double-precision fields.
 *
 * Reference: RINEX: The Receiver Independent Exchange Format, Version 3.04.
 *  IGS / RTCM-SC104, November 2021, Section 4 (Navigation Message Files).
 */
class RinexNavParser
{
public:
    //--------------------------------------------------------------------------//
    //                    Constructor / Destructor / Move                         //
    //--------------------------------------------------------------------------//

    /*! \brief Construct a parser with the given options. */
    explicit RinexNavParser(RinexNavParserOptions options = {}) noexcept;

    RinexNavParser(const RinexNavParser &)            = delete;
    RinexNavParser &operator=(const RinexNavParser &) = delete;
    RinexNavParser(RinexNavParser &&)                 = default;
    RinexNavParser &operator=(RinexNavParser &&)      = default;

    //--------------------------------------------------------------------------//
    //                         Public Parse Methods                              //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Parse a complete RINEX 3 navigation file from the filesystem.
     *
     * \param[in] filePath  Path to the RINEX 3 navigation file.
     * \return              ParseResult<RinexNavData> — success or a ParseError.
     */
    ParseResult<RinexNavData> parseNavigationFile(
        const std::filesystem::path &filePath) const;

    /*!
     * \brief Parse a complete RINEX 3 navigation file from an open stream.
     *
     * \param[in] stream      Open input stream positioned at the start of the file.
     * \param[in] sourceName  Label used in error descriptions to identify the data source.
     * \return                ParseResult<RinexNavData> — success or a ParseError.
     */
    ParseResult<RinexNavData> parseNavigationStream(
        std::istream    &stream,
        std::string_view sourceName = "<stream>") const;

    /*!
     * \brief Parse only the header section of a RINEX 3 navigation file.
     *
     * Skips all data records. Useful for inspecting metadata and ionospheric
     * coefficients without loading the full file.
     *
     * \param[in] filePath  Path to the RINEX 3 navigation file.
     * \return              ParseResult<RinexNavHeader> — success or a ParseError.
     */
    ParseResult<RinexNavHeader> parseNavigationFileHeader(
        const std::filesystem::path &filePath) const;

    /*!
     * \brief Parse only the header section from an open stream.
     *
     * \param[in] stream      Open input stream positioned at the start of the file.
     * \param[in] sourceName  Label used in error descriptions.
     * \return                ParseResult<RinexNavHeader> — success or a ParseError.
     */
    ParseResult<RinexNavHeader> parseNavigationStreamHeader(
        std::istream    &stream,
        std::string_view sourceName = "<stream>") const;

private:
    //--------------------------------------------------------------------------//
    //                           Private Members                                 //
    //--------------------------------------------------------------------------//

    RinexNavParserOptions options_;

    // Internal line-by-line parse state. Defined in the .cpp to keep the header clean.
    struct InternalParseState;

    //--------------------------------------------------------------------------//
    //                        Private Parse Helpers                               //
    //--------------------------------------------------------------------------//

    ParseResult<RinexNavHeader> parseHeaderSection(
        InternalParseState &state) const;

    ParseResult<RinexNavData> parseDataSection(
        InternalParseState &state,
        RinexNavHeader      header) const;

    ParseResult<GpsBroadcastEphemeris> parseGpsNavigationRecord(
        InternalParseState &state) const;

    int getOrbitLineCountForConstellation(char systemCharacter) const noexcept;

    bool isConstellationAccepted(Constellation system) const noexcept;
};

} // namespace gnss::rinex
