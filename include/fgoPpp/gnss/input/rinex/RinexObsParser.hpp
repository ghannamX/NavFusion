#pragma once

#include "fgoPpp/gnss/input/rinex/rinexObsTypes.hpp"
#include <cstdint>
#include <filesystem>
#include <istream>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace gnss::rinex
{

//------------------------------------------------------------//
//                    Parse Error Types                       //
//------------------------------------------------------------//

/*!
 * \brief Enumeration of RINEX 3 observation file parse error categories.
 */
enum class ParseErrorCode : std::uint8_t
{
    FileNotFound,               /*!< The specified file path does not exist */
    IoError,                    /*!< Low-level I/O failure on the stream */
    UnsupportedRinexVersion,    /*!< Version < 3.00 or >= 4.00 */
    MalformedHeaderRecord,      /*!< A header record could not be decoded */
    MissingEndOfHeader,         /*!< Stream ended before END OF HEADER was found */
    MalformedEpochHeaderRecord, /*!< The '>' epoch marker line could not be decoded */
    MalformedSatelliteDataLine, /*!< A satellite observation data line is invalid */
    UnexpectedEndOfFile,        /*!< Stream ended mid-epoch */
};

/*!
 * \brief Describes a parse failure with its location and a human-readable message.
 */
struct ParseError
{
    //------------------------------------------------------------//
    //                      Member Variables                      //
    //------------------------------------------------------------//

    ParseErrorCode code;
    /*!< Category of the error for programmatic handling. */

    std::uint32_t lineNumber = 0;
    /*!< 1-based line number in the source file or stream where the error was detected. */

    std::string description;
    /*!< Human-readable description of the error, including relevant file content. */
};

//------------------------------------------------------------//
//                       Parse Result                         //
//------------------------------------------------------------//

/*!
 * \brief Discriminated union of a successful parse result or a \c ParseError.
 *
 * Avoids exceptions for recoverable parse failures. Always check
 * \c parseSucceeded() before calling \c getResult() or \c moveResult().
 *
 * \tparam T  The type of the successfully parsed value.
 */
template <typename T>
class ParseResult
{
public:
    //------------------------------------------------------------//
    //                      Factory Methods                       //
    //------------------------------------------------------------//

    /*! \brief Construct a successful result holding \p value. */
    static ParseResult success(T value)
    {
        return ParseResult{std::move(value)};
    }

    /*! \brief Construct a failed result holding \p error. */
    static ParseResult failure(ParseError error)
    {
        return ParseResult{std::move(error)};
    }

    //------------------------------------------------------------//
    //                      Accessor Methods                      //
    //------------------------------------------------------------//

    /*!
     * \brief Returns \c true if parsing succeeded and a result value is available.
     * \return \c true on success, \c false on failure.
     */
    bool parseSucceeded() const noexcept
    {
        return std::holds_alternative<T>(storage_);
    }

    /*!
     * \brief Returns a const reference to the parsed value.
     *
     * Behaviour is undefined if \c parseSucceeded() returns \c false.
     *
     * \return Const reference to the parsed result.
     */
    const T &getResult() const &
    {
        return std::get<T>(storage_);
    }

    /*!
     * \brief Move the parsed value out of this result object.
     *
     * Behaviour is undefined if \c parseSucceeded() returns \c false.
     *
     * \return Rvalue reference to the parsed result.
     */
    T &&moveResult() &&
    {
        return std::get<T>(std::move(storage_));
    }

    /*!
     * \brief Returns a const reference to the parse error.
     *
     * Behaviour is undefined if \c parseSucceeded() returns \c true.
     *
     * \return Const reference to the \c ParseError.
     */
    const ParseError &getParseError() const
    {
        return std::get<ParseError>(storage_);
    }

private:
    explicit ParseResult(T val)        : storage_{std::move(val)} {}
    explicit ParseResult(ParseError e) : storage_{std::move(e)}   {}

    std::variant<T, ParseError> storage_;
};

//------------------------------------------------------------//
//                     Parser Options                         //
//------------------------------------------------------------//

/*!
 * \brief Configuration for the RINEX 3 observation file parser.
 */
struct RinexObsParserOptions
{
    //------------------------------------------------------------//
    //                      Member Variables                      //
    //------------------------------------------------------------//

    std::vector<Constellation> constellationFilter;
    /*!< If non-empty, only satellites of these constellations are stored.
     *   An empty vector accepts all constellations present in the file.
     *   Example: \c { Constellation::GPS } for GPS-only processing. */

    std::size_t expectedEpochCount = 0;
    /*!< Pre-allocation hint for the epochs vector. Set to the approximate number
     *   of epochs in the file to avoid repeated heap reallocation during parsing.
     *   0 means no pre-allocation. */

    bool skipMalformedSatelliteLines = false;
    /*!< If \c true, malformed satellite data lines are silently skipped rather than
     *   causing the parse to fail. Useful for files with minor formatting deviations.
     *   Default: \c false (strict parsing). */
};

//------------------------------------------------------------//
//                    RINEX 3 Obs Parser                      //
//------------------------------------------------------------//

/*!
 * \brief Stateless parser for RINEX 3.03 and 3.04 observation files.
 *
 * Construct once with options, then call the parse methods as many times as needed.
 * Supports both file-path input and stream input for unit testability.
 *
 * \section rinex3_format RINEX 3 observation file structure
 *  - \b Header: records with a 60-character content field followed by a 20-character
 *    label (columns 60–79). Terminated by the \c END OF HEADER record.
 *  - \b Epoch \b records: each starts with a \c '>' marker line, followed by one
 *    data line per satellite. Each satellite line begins with a 3-character satellite
 *    identifier, then N×16-character observation blocks
 *    (14-character value + 1-character LLI + 1-character signal strength),
 *    where N is given by the \c SYS / # / OBS TYPES header record.
 *
 * \section reference Reference
 *  RINEX: The Receiver Independent Exchange Format, Version 3.04.
 *  IGS / RTCM-SC104, November 2021.
 */
class RinexObsParser
{
public:
    //------------------------------------------------------------//
    //            Constructor / Destructor / Move                  //
    //------------------------------------------------------------//

    /*! \brief Construct a parser with the given options. */
    explicit RinexObsParser(RinexObsParserOptions options = {}) noexcept;

    RinexObsParser(const RinexObsParser &)            = delete;
    RinexObsParser &operator=(const RinexObsParser &) = delete;
    RinexObsParser(RinexObsParser &&)                 = default;
    RinexObsParser &operator=(RinexObsParser &&)      = default;

    //------------------------------------------------------------//
    //                     Public Parse Methods                    //
    //------------------------------------------------------------//

    /*!
     * \brief Parse a complete RINEX 3 observation file from the filesystem.
     *
     * \param[in] filePath  Path to the RINEX 3 observation file.
     * \return              \c ParseResult<RinexObsData> — success or a \c ParseError.
     */
    ParseResult<RinexObsData> parseObservationFile(
        const std::filesystem::path &filePath) const;

    /*!
     * \brief Parse a complete RINEX 3 observation file from an open stream.
     *
     * \param[in] stream      Open input stream positioned at the start of the file.
     * \param[in] sourceName  Label used in error descriptions to identify the data source.
     * \return                \c ParseResult<RinexObsData> — success or a \c ParseError.
     */
    ParseResult<RinexObsData> parseObservationStream(
        std::istream    &stream,
        std::string_view sourceName = "<stream>") const;

    /*!
     * \brief Parse only the header section of a RINEX 3 observation file.
     *
     * Skips all epoch data. Useful for inspecting metadata without loading the full file.
     *
     * \param[in] filePath  Path to the RINEX 3 observation file.
     * \return              \c ParseResult<RinexObsHeader> — success or a \c ParseError.
     */
    ParseResult<RinexObsHeader> parseObservationFileHeader(
        const std::filesystem::path &filePath) const;

    /*!
     * \brief Parse only the header section from an open stream.
     *
     * \param[in] stream      Open input stream positioned at the start of the file.
     * \param[in] sourceName  Label used in error descriptions.
     * \return                \c ParseResult<RinexObsHeader> — success or a \c ParseError.
     */
    ParseResult<RinexObsHeader> parseObservationStreamHeader(
        std::istream    &stream,
        std::string_view sourceName = "<stream>") const;

private:
    //------------------------------------------------------------//
    //                     Private Members                        //
    //------------------------------------------------------------//

    RinexObsParserOptions options_;

    // Internal line-by-line parse state. Defined in the .cpp to keep the header clean.
    struct InternalParseState;

    //------------------------------------------------------------//
    //                   Private Parse Helpers                    //
    //------------------------------------------------------------//

    ParseResult<RinexObsHeader> parseHeaderSection(
        InternalParseState &state) const;

    ParseResult<RinexObsData> parseEpochSection(
        InternalParseState &state,
        RinexObsHeader      header) const;

    ParseResult<RinexEpoch> parseSingleEpoch(
        InternalParseState   &state,
        const RinexObsHeader &header) const;

    ParseResult<SatelliteObservation> parseSatelliteDataLine(
        InternalParseState   &state,
        const RinexObsHeader &header,
        SatId                 satelliteId) const;

    void parseObservationTypesRecord(
        const std::string  &line,
        RinexObsHeader     &header,
        InternalParseState &state) const;

    bool isConstellationAccepted(Constellation system) const noexcept;
};

} // namespace gnss::rinex
