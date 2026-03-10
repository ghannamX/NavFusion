#pragma once

#include <cstdint>
#include <string>
#include <variant>

namespace gnss::rinex
{

//------------------------------------------------------------------------------//
//                            Parse Error Types                                  //
//------------------------------------------------------------------------------//

/*!
 * \brief Enumeration of RINEX 3 file parse error categories.
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
    MalformedNavigationRecord,  /*!< A navigation message record could not be decoded */
    UnexpectedEndOfFile,        /*!< Stream ended mid-record */
};

/*!
 * \brief Describes a parse failure with its location and a human-readable message.
 */
struct ParseError
{
    //--------------------------------------------------------------------------//
    //                            Member Variables                               //
    //--------------------------------------------------------------------------//

    ParseErrorCode code;
    /*!< Category of the error for programmatic handling. */

    std::uint32_t lineNumber = 0;
    /*!< 1-based line number in the source file or stream where the error was detected. */

    std::string description;
    /*!< Human-readable description of the error, including relevant file content. */
};

//------------------------------------------------------------------------------//
//                              Parse Result                                     //
//------------------------------------------------------------------------------//

/*!
 * \brief Discriminated union of a successful parse result or a ParseError.
 *
 * Avoids exceptions for recoverable parse failures. Always check
 * parseSucceeded() before calling getResult() or moveResult().
 *
 * \param T  The type of the successfully parsed value.
 */
template <typename T>
class ParseResult
{
public:
    //--------------------------------------------------------------------------//
    //                           Factory Methods                                 //
    //--------------------------------------------------------------------------//

    /*! \brief Construct a successful result holding value. */
    static ParseResult success(T value)
    {
        return ParseResult{std::move(value)};
    }

    /*! \brief Construct a failed result holding error. */
    static ParseResult failure(ParseError error)
    {
        return ParseResult{std::move(error)};
    }

    //--------------------------------------------------------------------------//
    //                           Accessor Methods                                //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Returns true if parsing succeeded and a result value is available.
     * \return true on success, false on failure.
     */
    bool parseSucceeded() const noexcept
    {
        return std::holds_alternative<T>(storage_);
    }

    /*!
     * \brief Returns a const reference to the parsed value.
     *
     * Behaviour is undefined if parseSucceeded() returns false.
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
     * Behaviour is undefined if parseSucceeded() returns false.
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
     * Behaviour is undefined if parseSucceeded() returns true.
     *
     * \return Const reference to the ParseError.
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

} // namespace gnss::rinex
