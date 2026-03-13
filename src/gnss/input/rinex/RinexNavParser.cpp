#include "input/rinex/RinexNavParser.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <spdlog/spdlog.h>

namespace gnss::rinex
{

//------------------------------------------------------------------------------//
//                        Internal Parse State                                   //
//------------------------------------------------------------------------------//

struct RinexNavParser::InternalParseState
{
    std::istream  &stream;
    std::string    line;
    std::uint32_t  lineNumber = 0;
    std::string    sourceName;

    explicit InternalParseState(std::istream &s, std::string_view name)
        : stream{s}, sourceName{name}
    {
    }

    bool readNextLine()
    {
        if (!std::getline(stream, line))
        {
            return false;
        }
        ++lineNumber;

        // Strip trailing CR (handles CRLF line endings on all platforms)
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }
        return true;
    }
};

//------------------------------------------------------------------------------//
//                          String Utility Helpers                                //
//------------------------------------------------------------------------------//

namespace
{

/*!
 * \brief Trim trailing whitespace from a string.
 */
std::string trimRight(std::string_view sv)
{
    const auto end = sv.find_last_not_of(" \t");
    if (end == std::string_view::npos)
    {
        return {};
    }
    return std::string(sv.substr(0, end + 1));
}

/*!
 * \brief Trim both leading and trailing whitespace from a string.
 */
std::string trimBoth(std::string_view sv)
{
    const auto start = sv.find_first_not_of(" \t");
    if (start == std::string_view::npos)
    {
        return {};
    }
    const auto end = sv.find_last_not_of(" \t");
    return std::string(sv.substr(start, end - start + 1));
}

/*!
 * \brief Safely extract a substring from a line, clamped to the line length.
 */
std::string_view safeSubstring(const std::string &line, std::size_t pos, std::size_t len)
{
    if (pos >= line.size())
    {
        return {};
    }
    return std::string_view(line).substr(pos, std::min(len, line.size() - pos));
}

/*!
 * \brief Extract the RINEX header label from columns 60-79 of a header line.
 */
std::string extractHeaderLabel(const std::string &line)
{
    if (line.size() < 60)
    {
        return {};
    }
    return trimRight(std::string_view(line).substr(60));
}

/*!
 * \brief Parse an integer from a fixed-width field, returning 0 if blank or invalid.
 */
int parseIntField(const std::string &line, std::size_t pos, std::size_t len)
{
    auto field = safeSubstring(line, pos, len);

    const auto start = field.find_first_not_of(' ');
    if (start == std::string_view::npos)
    {
        return 0;
    }
    const auto end = field.find_last_not_of(' ');
    field = field.substr(start, end - start + 1);

    int result = 0;
    auto [ptr, ec] = std::from_chars(field.data(), field.data() + field.size(), result);
    if (ec != std::errc{})
    {
        return 0;
    }
    return result;
}

/*!
 * \brief Parse a double from a fixed-width field, returning 0.0 if blank or invalid.
 */
double parseDoubleField(const std::string &line, std::size_t pos, std::size_t len)
{
    auto field = safeSubstring(line, pos, len);

    const auto start = field.find_first_not_of(' ');
    if (start == std::string_view::npos)
    {
        return 0.0;
    }
    const auto end = field.find_last_not_of(' ');
    field = field.substr(start, end - start + 1);

    double result = 0.0;
    auto [ptr, ec] = std::from_chars(field.data(), field.data() + field.size(), result);
    if (ec != std::errc{})
    {
        return 0.0;
    }
    return result;
}

/*!
 * \brief Parse a Fortran D19.12 double-precision field from a navigation record.
 *
 * RINEX 3 navigation records use Fortran D-notation for exponents
 * (e.g. "-1.234567890D+02"). This function extracts the 19-character field,
 * replaces 'D' or 'd' with 'E', trims whitespace, and parses as a double.
 *
 * \param[in] line   The full record line.
 * \param[in] pos    Start column of the 19-character field.
 * \return           Parsed double value, or 0.0 if the field is blank or invalid.
 */
double parseFortranDouble(const std::string &line, std::size_t pos)
{
    constexpr std::size_t fieldWidth = 19;

    auto field = safeSubstring(line, pos, fieldWidth);
    if (field.empty())
    {
        return 0.0;
    }

    // Copy to mutable buffer and replace Fortran 'D'/'d' exponent with 'E'/'e'
    char buffer[24] = {};
    std::size_t bufLen = 0;

    for (std::size_t i = 0; i < field.size() && bufLen < sizeof(buffer) - 1; ++i)
    {
        char ch = field[i];
        if (ch == 'D' || ch == 'd')
        {
            ch = 'E';
        }
        buffer[bufLen++] = ch;
    }

    // Trim leading whitespace
    std::size_t start = 0;
    while (start < bufLen && buffer[start] == ' ')
    {
        ++start;
    }

    // Trim trailing whitespace
    while (bufLen > start && buffer[bufLen - 1] == ' ')
    {
        --bufLen;
    }

    if (start >= bufLen)
    {
        return 0.0;
    }

    double result = 0.0;
    auto [ptr, ec] = std::from_chars(buffer + start, buffer + bufLen, result);
    if (ec != std::errc{})
    {
        return 0.0;
    }
    return result;
}

/*!
 * \brief Parse a constellation character to a Constellation enum.
 */
std::optional<Constellation> parseConstellationCharacter(char ch)
{
    switch (ch)
    {
        case 'G': return Constellation::GPS;
        case 'R': return Constellation::GLONASS;
        case 'E': return Constellation::Galileo;
        case 'C': return Constellation::BeiDou;
        case 'J': return Constellation::QZSS;
        case 'S': return Constellation::SBAS;
        case 'I': return Constellation::NavIC;
        default:  return std::nullopt;
    }
}

} // anonymous namespace

//------------------------------------------------------------------------------//
//                            Constructor                                         //
//------------------------------------------------------------------------------//

RinexNavParser::RinexNavParser(RinexNavParserOptions options) noexcept
    : options_{std::move(options)}
{
}

//------------------------------------------------------------------------------//
//                         Public Parse Methods                                  //
//------------------------------------------------------------------------------//

ParseResult<RinexNavData> RinexNavParser::parseNavigationFile(
    const std::filesystem::path &filePath) const
{
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        return ParseResult<RinexNavData>::failure(
            {ParseErrorCode::FileNotFound, 0,
             "Cannot open file: " + filePath.string()});
    }

    return parseNavigationStream(file, filePath.string());
}

ParseResult<RinexNavData> RinexNavParser::parseNavigationStream(
    std::istream    &stream,
    std::string_view sourceName) const
{
    InternalParseState state(stream, sourceName);

    auto headerResult = parseHeaderSection(state);
    if (!headerResult.parseSucceeded())
    {
        return ParseResult<RinexNavData>::failure(headerResult.getParseError());
    }

    auto header = std::move(headerResult).moveResult();

    spdlog::info("[RinexNavParser] Header parsed from '{}': version {:.2f}, "
                 "system '{}', Klobuchar {}",
                 state.sourceName, header.rinexVersion,
                 header.satelliteSystem,
                 header.klobucharIonosphere.isComplete() ? "complete" : "incomplete");

    return parseDataSection(state, std::move(header));
}

ParseResult<RinexNavHeader> RinexNavParser::parseNavigationFileHeader(
    const std::filesystem::path &filePath) const
{
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        return ParseResult<RinexNavHeader>::failure(
            {ParseErrorCode::FileNotFound, 0,
             "Cannot open file: " + filePath.string()});
    }

    return parseNavigationStreamHeader(file, filePath.string());
}

ParseResult<RinexNavHeader> RinexNavParser::parseNavigationStreamHeader(
    std::istream    &stream,
    std::string_view sourceName) const
{
    InternalParseState state(stream, sourceName);
    return parseHeaderSection(state);
}

//------------------------------------------------------------------------------//
//                          Header Section Parsing                               //
//------------------------------------------------------------------------------//

ParseResult<RinexNavHeader> RinexNavParser::parseHeaderSection(
    InternalParseState &state) const
{
    RinexNavHeader header;
    bool foundEndOfHeader = false;

    while (state.readNextLine())
    {
        const auto label = extractHeaderLabel(state.line);

        if (label == "END OF HEADER")
        {
            foundEndOfHeader = true;
            break;
        }

        if (label == "RINEX VERSION / TYPE")
        {
            header.rinexVersion = parseDoubleField(state.line, 0, 9);

            if (header.rinexVersion < 3.0 || header.rinexVersion >= 4.0)
            {
                return ParseResult<RinexNavHeader>::failure(
                    {ParseErrorCode::UnsupportedRinexVersion, state.lineNumber,
                     "Unsupported RINEX version " +
                         std::to_string(header.rinexVersion) +
                         " (only 3.xx is supported)"});
            }

            if (state.line.size() > 20)
            {
                header.fileType = state.line[20];
            }
            if (state.line.size() > 40)
            {
                header.satelliteSystem = state.line[40];
            }
        }
        else if (label == "PGM / RUN BY / DATE")
        {
            header.programName = trimBoth(safeSubstring(state.line, 0, 20));
            header.runBy       = trimBoth(safeSubstring(state.line, 20, 20));
        }
        else if (label == "IONOSPHERIC CORR")
        {
            // Format: A4,1X,4D12.4
            // Identifier is in columns 0-3: "GPSA", "GPSB", "GAL ", "BDSA", etc.
            auto corrType = trimBoth(safeSubstring(state.line, 0, 4));

            if (corrType == "GPSA")
            {
                // 4 alpha coefficients, each D12.4, starting at column 5
                for (int i = 0; i < 4; ++i)
                {
                    header.klobucharIonosphere.alpha[i] =
                        parseFortranDouble(state.line, 5 + i * 12);
                }
                header.klobucharIonosphere.hasAlpha = true;
            }
            else if (corrType == "GPSB")
            {
                for (int i = 0; i < 4; ++i)
                {
                    header.klobucharIonosphere.beta[i] =
                        parseFortranDouble(state.line, 5 + i * 12);
                }
                header.klobucharIonosphere.hasBeta = true;
            }
            // GAL (NeQuick), BDSA, BDSB, etc. — skip for now
        }
        else if (label == "LEAP SECONDS")
        {
            header.leapSeconds = parseIntField(state.line, 0, 6);
        }
        // Other header records are silently skipped (COMMENT, TIME SYSTEM CORR, etc.)
    }

    if (!foundEndOfHeader)
    {
        return ParseResult<RinexNavHeader>::failure(
            {ParseErrorCode::MissingEndOfHeader, state.lineNumber,
             "Stream ended before END OF HEADER in '" +
                 state.sourceName + "'"});
    }

    return ParseResult<RinexNavHeader>::success(std::move(header));
}

//------------------------------------------------------------------------------//
//                          Data Section Parsing                                  //
//------------------------------------------------------------------------------//

ParseResult<RinexNavData> RinexNavParser::parseDataSection(
    InternalParseState &state,
    RinexNavHeader      header) const
{
    RinexNavData data;
    data.header = std::move(header);

    if (options_.expectedRecordCount > 0)
    {
        data.gpsEphemerides.reserve(options_.expectedRecordCount);
    }

    while (state.readNextLine())
    {
        // Skip blank lines
        if (state.line.empty())
        {
            continue;
        }

        // The first character identifies the constellation
        const char systemChar = state.line[0];

        // Determine constellation
        auto constellation = parseConstellationCharacter(systemChar);
        if (!constellation.has_value())
        {
            // Could be a blank or malformed line; skip
            spdlog::warn("[RinexNavParser] Line {}: unrecognised system character '{}', "
                         "skipping record",
                         state.lineNumber, systemChar);

            // Skip the orbit lines for an unknown 8-line record
            for (int i = 0; i < 7; ++i)
            {
                if (!state.readNextLine())
                {
                    break;
                }
            }
            continue;
        }

        // Check constellation filter
        if (!isConstellationAccepted(*constellation))
        {
            const int orbitLines = getOrbitLineCountForConstellation(systemChar);
            for (int i = 0; i < orbitLines; ++i)
            {
                if (!state.readNextLine())
                {
                    break;
                }
            }
            continue;
        }

        if (*constellation == Constellation::GPS)
        {
            auto recordResult = parseGpsNavigationRecord(state);
            if (!recordResult.parseSucceeded())
            {
                if (options_.skipMalformedRecords)
                {
                    spdlog::warn("[RinexNavParser] Line {}: skipping malformed GPS record: {}",
                                 state.lineNumber,
                                 recordResult.getParseError().description);
                    continue;
                }
                return ParseResult<RinexNavData>::failure(recordResult.getParseError());
            }
            data.gpsEphemerides.push_back(std::move(recordResult).moveResult());
        }
        else
        {
            // Skip non-GPS constellation records for now
            const int orbitLines = getOrbitLineCountForConstellation(systemChar);
            spdlog::debug("[RinexNavParser] Line {}: skipping {} record ({} orbit lines)",
                          state.lineNumber, systemChar, orbitLines);
            for (int i = 0; i < orbitLines; ++i)
            {
                if (!state.readNextLine())
                {
                    break;
                }
            }
        }
    }

    spdlog::info("[RinexNavParser] Parsed {} GPS ephemeris records from '{}'",
                 data.gpsEphemerides.size(), state.sourceName);

    return ParseResult<RinexNavData>::success(std::move(data));
}

//------------------------------------------------------------------------------//
//                     GPS Navigation Record Parsing                             //
//------------------------------------------------------------------------------//

ParseResult<GpsBroadcastEphemeris> RinexNavParser::parseGpsNavigationRecord(
    InternalParseState &state) const
{
    // The record identifier line (line 0) is already in state.line.
    // Format: A1,I2.2,1X,I4,4(1X,I2.2),3D19.12
    //
    //   Col 0     : 'G'
    //   Col 1-2   : PRN (2-digit, zero-padded)
    //   Col 3     : space
    //   Col 4-7   : year (4 digits)
    //   Col 8     : space
    //   Col 9-10  : month
    //   Col 11    : space
    //   Col 12-13 : day
    //   Col 14    : space
    //   Col 15-16 : hour
    //   Col 17    : space
    //   Col 18-19 : minute
    //   Col 20    : space
    //   Col 21-22 : second (integer)
    //   Col 23-41 : af0 (D19.12)
    //   Col 42-60 : af1 (D19.12)
    //   Col 61-79 : af2 (D19.12)

    const auto &line0 = state.line;
    const auto line0Number = state.lineNumber;

    if (line0.size() < 23)
    {
        return ParseResult<GpsBroadcastEphemeris>::failure(
            {ParseErrorCode::MalformedNavigationRecord, line0Number,
             "GPS nav record identifier line too short (< 23 chars)"});
    }

    GpsBroadcastEphemeris eph;

    // Parse satellite ID
    auto satId = SatId::parseSatelliteIdFromRinex3String(
        std::string_view(line0).substr(0, 3));

    if (!satId.has_value())
    {
        return ParseResult<GpsBroadcastEphemeris>::failure(
            {ParseErrorCode::MalformedNavigationRecord, line0Number,
             "Cannot parse GPS satellite ID from '" +
                 std::string(line0.substr(0, 3)) + "'"});
    }
    eph.satelliteId = *satId;

    // Parse epoch (Toc)
    eph.timeOfClock.year_   = parseIntField(line0, 4, 4);
    eph.timeOfClock.month_  = parseIntField(line0, 9, 2);
    eph.timeOfClock.day_    = parseIntField(line0, 12, 2);
    eph.timeOfClock.hour_   = parseIntField(line0, 15, 2);
    eph.timeOfClock.minute_ = parseIntField(line0, 18, 2);
    eph.timeOfClock.second_ = static_cast<double>(parseIntField(line0, 21, 2));

    if (!eph.timeOfClock.isValidEpoch())
    {
        return ParseResult<GpsBroadcastEphemeris>::failure(
            {ParseErrorCode::MalformedNavigationRecord, line0Number,
             "Invalid Toc epoch for " +
                 eph.satelliteId.formatSatelliteIdAsRinex3String()});
    }

    // SV clock parameters (line 0, cols 23+)
    eph.clockBias      = parseFortranDouble(line0, 23);
    eph.clockDrift     = parseFortranDouble(line0, 42);
    eph.clockDriftRate = parseFortranDouble(line0, 61);

    // --- Broadcast Orbit Lines 1-7 ---
    // Each line: 4X,4D19.12 (4-space indent, then 4 x 19-char fields)
    // Fields at columns: 4, 23, 42, 61

    // Line 1: IODE, Crs, Delta_n, M0
    if (!state.readNextLine())
    {
        return ParseResult<GpsBroadcastEphemeris>::failure(
            {ParseErrorCode::UnexpectedEndOfFile, state.lineNumber,
             "Unexpected end of file in GPS nav record line 1"});
    }
    eph.issueOfDataEphemeris       = parseFortranDouble(state.line, 4);
    eph.sinCorrectionToOrbitRadius = parseFortranDouble(state.line, 23);
    eph.meanMotionDifference       = parseFortranDouble(state.line, 42);
    eph.meanAnomalyAtReference     = parseFortranDouble(state.line, 61);

    // Line 2: Cuc, e, Cus, sqrt(A)
    if (!state.readNextLine())
    {
        return ParseResult<GpsBroadcastEphemeris>::failure(
            {ParseErrorCode::UnexpectedEndOfFile, state.lineNumber,
             "Unexpected end of file in GPS nav record line 2"});
    }
    eph.cosCorrectionToLatitude    = parseFortranDouble(state.line, 4);
    eph.eccentricity               = parseFortranDouble(state.line, 23);
    eph.sinCorrectionToLatitude    = parseFortranDouble(state.line, 42);
    eph.squareRootOfSemiMajorAxis  = parseFortranDouble(state.line, 61);

    // Line 3: Toe, Cic, OMEGA0, Cis
    if (!state.readNextLine())
    {
        return ParseResult<GpsBroadcastEphemeris>::failure(
            {ParseErrorCode::UnexpectedEndOfFile, state.lineNumber,
             "Unexpected end of file in GPS nav record line 3"});
    }
    eph.timeOfEphemeris              = parseFortranDouble(state.line, 4);
    eph.cosCorrectionToInclination   = parseFortranDouble(state.line, 23);
    eph.longitudeOfAscendingNode     = parseFortranDouble(state.line, 42);
    eph.sinCorrectionToInclination   = parseFortranDouble(state.line, 61);

    // Line 4: i0, Crc, omega, OMEGA_DOT
    if (!state.readNextLine())
    {
        return ParseResult<GpsBroadcastEphemeris>::failure(
            {ParseErrorCode::UnexpectedEndOfFile, state.lineNumber,
             "Unexpected end of file in GPS nav record line 4"});
    }
    eph.inclinationAngle             = parseFortranDouble(state.line, 4);
    eph.cosCorrectionToOrbitRadius   = parseFortranDouble(state.line, 23);
    eph.argumentOfPerigee            = parseFortranDouble(state.line, 42);
    eph.rateOfRightAscension         = parseFortranDouble(state.line, 61);

    // Line 5: IDOT, Codes on L2, GPS Week, L2 P data flag
    if (!state.readNextLine())
    {
        return ParseResult<GpsBroadcastEphemeris>::failure(
            {ParseErrorCode::UnexpectedEndOfFile, state.lineNumber,
             "Unexpected end of file in GPS nav record line 5"});
    }
    eph.rateOfInclination  = parseFortranDouble(state.line, 4);
    eph.codesOnL2Channel   = parseFortranDouble(state.line, 23);
    eph.gpsWeekNumber      = parseFortranDouble(state.line, 42);
    eph.l2PDataFlag        = parseFortranDouble(state.line, 61);

    // Line 6: SV accuracy, SV health, TGD, IODC
    if (!state.readNextLine())
    {
        return ParseResult<GpsBroadcastEphemeris>::failure(
            {ParseErrorCode::UnexpectedEndOfFile, state.lineNumber,
             "Unexpected end of file in GPS nav record line 6"});
    }
    eph.signalAccuracy     = parseFortranDouble(state.line, 4);
    eph.satelliteHealth    = parseFortranDouble(state.line, 23);
    eph.totalGroupDelay    = parseFortranDouble(state.line, 42);
    eph.issueOfDataClock   = parseFortranDouble(state.line, 61);

    // Line 7: Transmission time, Fit interval (may be short/absent)
    if (!state.readNextLine())
    {
        return ParseResult<GpsBroadcastEphemeris>::failure(
            {ParseErrorCode::UnexpectedEndOfFile, state.lineNumber,
             "Unexpected end of file in GPS nav record line 7"});
    }
    eph.transmissionTime = parseFortranDouble(state.line, 4);
    eph.fitInterval      = parseFortranDouble(state.line, 23);
    // Fields 3-4 on line 7 are spare — not parsed

    return ParseResult<GpsBroadcastEphemeris>::success(std::move(eph));
}

//------------------------------------------------------------------------------//
//                   Orbit Line Count per Constellation                          //
//------------------------------------------------------------------------------//

int RinexNavParser::getOrbitLineCountForConstellation(char systemCharacter) const noexcept
{
    switch (systemCharacter)
    {
        case 'G': return 7; // GPS:     7 orbit lines (8 total with ID line)
        case 'E': return 7; // Galileo: 7 orbit lines
        case 'C': return 7; // BeiDou:  7 orbit lines
        case 'J': return 7; // QZSS:    7 orbit lines
        case 'I': return 7; // NavIC:   7 orbit lines
        case 'R': return 3; // GLONASS: 3 orbit lines (4 total)
        case 'S': return 3; // SBAS:    3 orbit lines (4 total)
        default:  return 7; // Default to 7 for unknown constellations
    }
}

//------------------------------------------------------------------------------//
//                       Constellation Filter                                    //
//------------------------------------------------------------------------------//

bool RinexNavParser::isConstellationAccepted(Constellation system) const noexcept
{
    if (options_.constellationFilter.empty())
    {
        return true;
    }

    return std::find(options_.constellationFilter.begin(),
                     options_.constellationFilter.end(),
                     system) != options_.constellationFilter.end();
}

} // namespace gnss::rinex
