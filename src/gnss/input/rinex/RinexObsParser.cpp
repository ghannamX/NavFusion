#include "input/rinex/RinexObsParser.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <sstream>
#include <spdlog/spdlog.h>

namespace gnss::rinex
{

//------------------------------------------------------------------------------//
//                        Internal Parse State                                   //
//------------------------------------------------------------------------------//

struct RinexObsParser::InternalParseState
{
    std::istream   &stream;
    std::string     line;
    std::uint32_t   lineNumber      = 0;
    std::string     sourceName;

    // Used by SYS / # / OBS TYPES continuation-line tracking
    Constellation   currentObsTypeConstellation = Constellation::GPS;
    int             currentObsTypeExpectedCount = 0;

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
 * \brief Trim trailing whitespace from a string in-place.
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
 * \brief Parse a double from a fixed-width field, returning 0.0 if blank or invalid.
 */
double parseDoubleField(const std::string &line, std::size_t pos, std::size_t len)
{
    auto field = safeSubstring(line, pos, len);

    // Trim whitespace
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
 * \brief Extract the RINEX header label from columns 60–79 of a header line.
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

RinexObsParser::RinexObsParser(RinexObsParserOptions options) noexcept
    : options_{std::move(options)}
{
}

//------------------------------------------------------------------------------//
//                         Public Parse Methods                                  //
//------------------------------------------------------------------------------//

ParseResult<RinexObsData> RinexObsParser::parseObservationFile(
    const std::filesystem::path &filePath) const
{
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        return ParseResult<RinexObsData>::failure(
            {ParseErrorCode::FileNotFound, 0,
             "Cannot open file: " + filePath.string()});
    }

    return parseObservationStream(file, filePath.string());
}

ParseResult<RinexObsData> RinexObsParser::parseObservationStream(
    std::istream    &stream,
    std::string_view sourceName) const
{
    InternalParseState state(stream, sourceName);

    auto headerResult = parseHeaderSection(state);
    if (!headerResult.parseSucceeded())
    {
        return ParseResult<RinexObsData>::failure(headerResult.getParseError());
    }

    auto header = std::move(headerResult).moveResult();

    spdlog::info("[RinexObsParser] Header parsed from '{}': version {:.2f}, "
                 "{} constellation(s), marker '{}'",
                 state.sourceName, header.rinexVersion,
                 header.observationTypes.size(), header.markerName);

    return parseEpochSection(state, std::move(header));
}

ParseResult<RinexObsHeader> RinexObsParser::parseObservationFileHeader(
    const std::filesystem::path &filePath) const
{
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        return ParseResult<RinexObsHeader>::failure(
            {ParseErrorCode::FileNotFound, 0,
             "Cannot open file: " + filePath.string()});
    }

    return parseObservationStreamHeader(file, filePath.string());
}

ParseResult<RinexObsHeader> RinexObsParser::parseObservationStreamHeader(
    std::istream    &stream,
    std::string_view sourceName) const
{
    InternalParseState state(stream, sourceName);
    return parseHeaderSection(state);
}

//------------------------------------------------------------------------------//
//                          Header Section Parsing                               //
//------------------------------------------------------------------------------//

ParseResult<RinexObsHeader> RinexObsParser::parseHeaderSection(
    InternalParseState &state) const
{
    RinexObsHeader header;
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
                return ParseResult<RinexObsHeader>::failure(
                    {ParseErrorCode::UnsupportedRinexVersion, state.lineNumber,
                     "Unsupported RINEX version " +
                         std::to_string(header.rinexVersion) +
                         " (only 3.xx is supported)"});
            }

            if (state.line.size() > 20)
            {
                header.fileType = state.line[20];
            }
        }
        else if (label == "PGM / RUN BY / DATE")
        {
            header.programName = trimBoth(safeSubstring(state.line, 0, 20));
            header.runBy       = trimBoth(safeSubstring(state.line, 20, 20));
        }
        else if (label == "MARKER NAME")
        {
            header.markerName = trimBoth(safeSubstring(state.line, 0, 60));
        }
        else if (label == "MARKER NUMBER")
        {
            header.markerNumber = trimBoth(safeSubstring(state.line, 0, 20));
        }
        else if (label == "MARKER TYPE")
        {
            header.markerType = trimBoth(safeSubstring(state.line, 0, 20));
        }
        else if (label == "OBSERVER / AGENCY")
        {
            header.observerName = trimBoth(safeSubstring(state.line, 0, 20));
            header.agencyName   = trimBoth(safeSubstring(state.line, 20, 40));
        }
        else if (label == "REC # / TYPE / VERS")
        {
            header.receiverSerialNumber    = trimBoth(safeSubstring(state.line, 0, 20));
            header.receiverType            = trimBoth(safeSubstring(state.line, 20, 20));
            header.receiverFirmwareVersion = trimBoth(safeSubstring(state.line, 40, 20));
        }
        else if (label == "ANT # / TYPE")
        {
            header.antennaSerialNumber = trimBoth(safeSubstring(state.line, 0, 20));
            header.antennaType         = trimBoth(safeSubstring(state.line, 20, 20));
        }
        else if (label == "APPROX POSITION XYZ")
        {
            header.approximatePosition.x = parseDoubleField(state.line, 0, 14);
            header.approximatePosition.y = parseDoubleField(state.line, 14, 14);
            header.approximatePosition.z = parseDoubleField(state.line, 28, 14);
        }
        else if (label == "ANTENNA: DELTA H/E/N")
        {
            header.antennaDeltaOffset.height = parseDoubleField(state.line, 0, 14);
            header.antennaDeltaOffset.east   = parseDoubleField(state.line, 14, 14);
            header.antennaDeltaOffset.north  = parseDoubleField(state.line, 28, 14);
        }
        else if (label == "SYS / # / OBS TYPES")
        {
            parseObservationTypesRecord(state.line, header, state);
        }
        else if (label == "INTERVAL")
        {
            header.nominalSamplingInterval = parseDoubleField(state.line, 0, 10);
        }
        else if (label == "TIME OF FIRST OBS")
        {
            header.timeOfFirstObservation.year   = parseIntField(state.line, 0, 6);
            header.timeOfFirstObservation.month  = parseIntField(state.line, 6, 6);
            header.timeOfFirstObservation.day    = parseIntField(state.line, 12, 6);
            header.timeOfFirstObservation.hour   = parseIntField(state.line, 18, 6);
            header.timeOfFirstObservation.minute = parseIntField(state.line, 24, 6);
            header.timeOfFirstObservation.second = parseDoubleField(state.line, 30, 13);
        }
        else if (label == "TIME OF LAST OBS")
        {
            header.timeOfLastObservation.year   = parseIntField(state.line, 0, 6);
            header.timeOfLastObservation.month  = parseIntField(state.line, 6, 6);
            header.timeOfLastObservation.day    = parseIntField(state.line, 12, 6);
            header.timeOfLastObservation.hour   = parseIntField(state.line, 18, 6);
            header.timeOfLastObservation.minute = parseIntField(state.line, 24, 6);
            header.timeOfLastObservation.second = parseDoubleField(state.line, 30, 13);
        }
        else if (label == "LEAP SECONDS")
        {
            header.leapSeconds = parseIntField(state.line, 0, 6);
        }
        // Other header records are silently skipped (COMMENT, SIGNAL STRENGTH UNIT, etc.)
    }

    if (!foundEndOfHeader)
    {
        return ParseResult<RinexObsHeader>::failure(
            {ParseErrorCode::MissingEndOfHeader, state.lineNumber,
             "Stream ended before END OF HEADER in '" +
                 state.sourceName + "'"});
    }

    return ParseResult<RinexObsHeader>::success(std::move(header));
}

//------------------------------------------------------------------------------//
//                    SYS / # / OBS TYPES Record Parsing                         //
//------------------------------------------------------------------------------//

void RinexObsParser::parseObservationTypesRecord(
    const std::string  &line,
    RinexObsHeader     &header,
    InternalParseState &state) const
{
    // RINEX 3 SYS / # / OBS TYPES format: A1, 2X, I3, 13(1X, A3)
    //   Col 0     : system character (G, R, E, C, J, S, I) or blank for continuation
    //   Col 3-5   : number of observation types (first line only)
    //   Col 6+    : 4-char fields (1 space + 3-char obs code), up to 13 per line

    const char systemChar = line.empty() ? ' ' : line[0];

    if (systemChar != ' ')
    {
        // First line of a new constellation block
        auto constellation = parseConstellationCharacter(systemChar);
        if (!constellation.has_value())
        {
            spdlog::warn("[RinexObsParser] Line {}: unknown system character '{}' "
                         "in SYS / # / OBS TYPES, skipping",
                         state.lineNumber, systemChar);
            return;
        }

        state.currentObsTypeConstellation  = *constellation;
        state.currentObsTypeExpectedCount = parseIntField(line, 3, 3);

        // Ensure the vector exists (may already from a prior malformed record)
        header.observationTypes[*constellation];
    }

    // Parse observation codes from this line (first or continuation)
    auto &codeList = header.observationTypes[state.currentObsTypeConstellation];

    // Up to 13 codes per line, starting at column 6, each 4 chars wide (space + 3-char code)
    // Format: A1, 2X, I3, 13(1X, A3) — descriptors begin at index 6
    constexpr int maxCodesPerLine = 13;
    constexpr int startColumn     = 6;
    constexpr int fieldWidth      = 4;

    for (int i = 0; i < maxCodesPerLine; ++i)
    {
        // Stop if we have already collected the expected count
        if (static_cast<int>(codeList.size()) >= state.currentObsTypeExpectedCount)
        {
            break;
        }

        const std::size_t fieldStart = startColumn + i * fieldWidth;
        auto field = safeSubstring(line, fieldStart, fieldWidth);
        if (field.size() < 4)
        {
            break;
        }

        // The code is in characters 1-3 of the 4-char field (char 0 is a space)
        auto obsCode = ObsCode::parseObsCodeFromRinex3String(field.substr(1, 3));
        if (obsCode.has_value())
        {
            codeList.push_back(*obsCode);
        }
    }
}

//------------------------------------------------------------------------------//
//                          Epoch Section Parsing                                //
//------------------------------------------------------------------------------//

ParseResult<RinexObsData> RinexObsParser::parseEpochSection(
    InternalParseState &state,
    RinexObsHeader      header) const
{
    RinexObsData data;
    data.header = std::move(header);

    if (options_.expectedEpochCount > 0)
    {
        data.epochs.reserve(options_.expectedEpochCount);
    }

    while (state.readNextLine())
    {
        // Skip blank lines between epochs
        if (state.line.empty())
        {
            continue;
        }

        // Epoch records start with '>'
        if (state.line[0] != '>')
        {
            // Some files have trailing content after last epoch; skip gracefully
            continue;
        }

        auto epochResult = parseSingleEpoch(state, data.header);
        if (!epochResult.parseSucceeded())
        {
            return ParseResult<RinexObsData>::failure(epochResult.getParseError());
        }

        data.epochs.push_back(std::move(epochResult).moveResult());
    }

    spdlog::info("[RinexObsParser] Parsed {} epochs, {} total satellite-epoch "
                 "observations from '{}'",
                 data.epochs.size(), data.getTotalSatelliteEpochCount(),
                 state.sourceName);

    return ParseResult<RinexObsData>::success(std::move(data));
}

//------------------------------------------------------------------------------//
//                         Single Epoch Parsing                                  //
//------------------------------------------------------------------------------//

ParseResult<RinexEpoch> RinexObsParser::parseSingleEpoch(
    InternalParseState   &state,
    const RinexObsHeader &header) const
{
    // The epoch header line is already in state.line (starts with '>')
    // RINEX 3 epoch header format:
    //   Col 0     : '>'
    //   Col 2-5   : year
    //   Col 6     : space
    //   Col 7-8   : month (right-justified, space-padded)
    //   Col 9     : space
    //   Col 10-11 : day
    //   Col 12    : space
    //   Col 13-14 : hour
    //   Col 15    : space
    //   Col 16-17 : minute
    //   Col 18-28 : seconds (F11.7)
    //   Col 29-31 : epoch flag (2 spaces + digit, or space + 2-digit)
    //   Col 32-34 : number of satellites (right-justified)
    //   Col 41-56 : receiver clock offset (optional, F15.12)

    const auto &epochLine = state.line;
    const auto epochLineNumber = state.lineNumber;

    if (epochLine.size() < 35)
    {
        return ParseResult<RinexEpoch>::failure(
            {ParseErrorCode::MalformedEpochHeaderRecord, epochLineNumber,
             "Epoch header line too short (< 35 chars)"});
    }

    RinexEpoch epoch;

    epoch.time.year   = parseIntField(epochLine, 2, 4);
    epoch.time.month  = parseIntField(epochLine, 7, 2);
    epoch.time.day    = parseIntField(epochLine, 10, 2);
    epoch.time.hour   = parseIntField(epochLine, 13, 2);
    epoch.time.minute = parseIntField(epochLine, 16, 2);
    epoch.time.second = parseDoubleField(epochLine, 18, 11);

    if (!epoch.time.isValidEpoch())
    {
        return ParseResult<RinexEpoch>::failure(
            {ParseErrorCode::MalformedEpochHeaderRecord, epochLineNumber,
             "Invalid epoch timestamp at line " +
                 std::to_string(epochLineNumber)});
    }

    const int flagValue = parseIntField(epochLine, 29, 3);
    if (flagValue < 0 || flagValue > 6)
    {
        return ParseResult<RinexEpoch>::failure(
            {ParseErrorCode::MalformedEpochHeaderRecord, epochLineNumber,
             "Invalid epoch flag value " + std::to_string(flagValue)});
    }
    epoch.flag = static_cast<EpochFlag>(flagValue);

    const int numSatellites = parseIntField(epochLine, 32, 3);

    // Receiver clock offset (optional, cols 41-56)
    if (epochLine.size() > 41)
    {
        auto clkField = safeSubstring(epochLine, 41, 15);
        if (clkField.find_first_not_of(" \t") != std::string_view::npos)
        {
            epoch.receiverClockOffset    = parseDoubleField(epochLine, 41, 15);
            epoch.hasReceiverClockOffset = true;
        }
    }

    // Skip non-observation epochs (header records follow, events, etc.)
    if (epoch.flag != EpochFlag::NormalObservation &&
        epoch.flag != EpochFlag::PowerFailure)
    {
        // Read and discard the lines that follow special epochs
        for (int i = 0; i < numSatellites; ++i)
        {
            if (!state.readNextLine())
            {
                break;
            }
        }
        epoch.satellites.clear();
        return ParseResult<RinexEpoch>::success(std::move(epoch));
    }

    // Parse satellite data lines
    epoch.satellites.reserve(numSatellites);

    for (int satIndex = 0; satIndex < numSatellites; ++satIndex)
    {
        if (!state.readNextLine())
        {
            return ParseResult<RinexEpoch>::failure(
                {ParseErrorCode::UnexpectedEndOfFile, state.lineNumber,
                 "Expected " + std::to_string(numSatellites) +
                     " satellite lines but stream ended after " +
                     std::to_string(satIndex)});
        }

        // Parse satellite ID from first 3 characters
        if (state.line.size() < 3)
        {
            if (options_.skipMalformedSatelliteLines)
            {
                continue;
            }
            return ParseResult<RinexEpoch>::failure(
                {ParseErrorCode::MalformedSatelliteDataLine, state.lineNumber,
                 "Satellite data line too short (< 3 chars)"});
        }

        auto satId = SatId::parseSatelliteIdFromRinex3String(
            std::string_view(state.line).substr(0, 3));

        if (!satId.has_value())
        {
            if (options_.skipMalformedSatelliteLines)
            {
                continue;
            }
            return ParseResult<RinexEpoch>::failure(
                {ParseErrorCode::MalformedSatelliteDataLine, state.lineNumber,
                 "Cannot parse satellite ID from '" +
                     std::string(state.line.substr(0, 3)) + "'"});
        }

        // Check constellation filter
        if (!isConstellationAccepted(satId->system))
        {
            continue;
        }

        auto satObsResult = parseSatelliteDataLine(state, header, *satId);
        if (!satObsResult.parseSucceeded())
        {
            if (options_.skipMalformedSatelliteLines)
            {
                continue;
            }
            return ParseResult<RinexEpoch>::failure(satObsResult.getParseError());
        }

        epoch.satellites.push_back(std::move(satObsResult).moveResult());
    }

    return ParseResult<RinexEpoch>::success(std::move(epoch));
}

//------------------------------------------------------------------------------//
//                      Satellite Data Line Parsing                              //
//------------------------------------------------------------------------------//

ParseResult<SatelliteObservation> RinexObsParser::parseSatelliteDataLine(
    InternalParseState   &state,
    const RinexObsHeader &header,
    SatId                 satelliteId) const
{
    SatelliteObservation satObs;
    satObs.satelliteId = satelliteId;

    // Look up the number of observation types for this constellation
    const int numObsTypes = header.getObservationCodeCount(satelliteId.system);
    satObs.observations.resize(numObsTypes);

    // Each observation occupies 16 characters: 14-char value + 1-char LLI + 1-char SNR
    // Satellite ID occupies the first 3 characters
    constexpr int satIdWidth = 3;
    constexpr int obsFieldWidth = 16;

    for (int obsIndex = 0; obsIndex < numObsTypes; ++obsIndex)
    {
        const std::size_t fieldStart = satIdWidth + obsIndex * obsFieldWidth;

        // If the line is shorter than this field, the observation is absent
        if (fieldStart >= state.line.size())
        {
            break;
        }

        auto field = safeSubstring(state.line, fieldStart, obsFieldWidth);

        // Parse the 14-character numeric value
        if (field.size() >= 14)
        {
            auto valueField = field.substr(0, 14);
            const auto nonSpace = valueField.find_first_not_of(' ');
            if (nonSpace != std::string_view::npos)
            {
                // Trim for from_chars
                valueField = valueField.substr(nonSpace);
                const auto lastNonSpace = valueField.find_last_not_of(' ');
                if (lastNonSpace != std::string_view::npos)
                {
                    valueField = valueField.substr(0, lastNonSpace + 1);
                }

                double obsValue = 0.0;
                auto [ptr, ec] = std::from_chars(
                    valueField.data(), valueField.data() + valueField.size(), obsValue);

                if (ec == std::errc{})
                {
                    satObs.observations[obsIndex].value = obsValue;
                }
            }
        }

        // Parse LLI (character at position 14 within the field)
        if (field.size() >= 15 && field[14] != ' ')
        {
            const char lliChar = field[14];
            if (lliChar >= '0' && lliChar <= '7')
            {
                satObs.observations[obsIndex].lossOfLockIndicator =
                    static_cast<std::uint8_t>(lliChar - '0');
            }
        }

        // Parse signal strength (character at position 15 within the field)
        if (field.size() >= 16 && field[15] != ' ')
        {
            const char snrChar = field[15];
            if (snrChar >= '0' && snrChar <= '9')
            {
                satObs.observations[obsIndex].signalStrength =
                    static_cast<std::uint8_t>(snrChar - '0');
            }
        }
    }

    return ParseResult<SatelliteObservation>::success(std::move(satObs));
}

//------------------------------------------------------------------------------//
//                       Constellation Filter                                    //
//------------------------------------------------------------------------------//

bool RinexObsParser::isConstellationAccepted(Constellation system) const noexcept
{
    if (options_.constellationFilter.empty())
    {
        return true;
    }

    return std::find(options_.constellationFilter.begin(),
                     options_.constellationFilter.end(),
                     system) != options_.constellationFilter.end();
}

//------------------------------------------------------------------------------//
//                     RinexObsHeader Method Implementations                      //
//------------------------------------------------------------------------------//

int RinexObsHeader::getObservationSlotIndex(
    Constellation system, ObsCode code) const noexcept
{
    const auto it = observationTypes.find(system);
    if (it == observationTypes.end())
    {
        return -1;
    }

    const auto &codes = it->second;
    for (int i = 0; i < static_cast<int>(codes.size()); ++i)
    {
        if (codes[i] == code)
        {
            return i;
        }
    }
    return -1;
}

int RinexObsHeader::getObservationCodeCount(Constellation system) const noexcept
{
    const auto it = observationTypes.find(system);
    if (it == observationTypes.end())
    {
        return 0;
    }
    return static_cast<int>(it->second.size());
}

//------------------------------------------------------------------------------//
//                     RinexObsData Method Implementations                       //
//------------------------------------------------------------------------------//

std::size_t RinexObsData::getTotalSatelliteEpochCount() const noexcept
{
    std::size_t count = 0;
    for (const auto &epoch : epochs)
    {
        count += epoch.satellites.size();
    }
    return count;
}

} // namespace gnss::rinex
