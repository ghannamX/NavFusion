#pragma once

#include "fgoPpp/gnss/common/obsMeasurement.hpp"
#include "fgoPpp/gnss/common/obsCode.hpp"
#include "fgoPpp/gnss/common/satId.hpp"
#include "fgoPpp/gnss/time/GnssTime.hpp"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace gnss::rinex
{

//------------------------------------------------------------//
//                        Epoch Flag                          //
//------------------------------------------------------------//

/*!
 * \brief RINEX 3 epoch flag values (column 29 of the epoch header record).
 *
 * Only \c NormalObservation (flag = 0) represents a usable set of satellite
 * measurements. All other flags carry special meaning and the satellite data
 * lines that follow must be interpreted accordingly.
 *
 * Reference: RINEX 3.04 specification, Section 3.3, Table 3.
 */
enum class EpochFlag : std::uint8_t
{
    NormalObservation    = 0, /*!< Standard epoch — all satellite data are valid */
    PowerFailure         = 1, /*!< Power failure between previous and current epoch */
    StartMovingAntenna   = 2, /*!< Start of moving-antenna event */
    NewSiteOccupation    = 3, /*!< New site occupation (same station, different position) */
    HeaderRecordFollows  = 4, /*!< Header records follow this epoch record */
    ExternalEvent        = 5, /*!< External event: significant time tag discontinuity */
    CycleSlipRecords     = 6, /*!< Cycle-slip records follow to repair earlier epochs */
};

//------------------------------------------------------------//
//              Per-Satellite Observation Data                //
//------------------------------------------------------------//

/*!
 * \brief All measurements for a single satellite within one receiver epoch.
 *
 * The \c observations vector is indexed by \b column \b slot, where slot 0
 * corresponds to the first observation code listed in
 * \c RinexObsHeader::observationTypes[system], slot 1 to the second, and so on.
 *
 * Obtain a slot index at startup via
 * \c RinexObsHeader::getObservationSlotIndex(system, code), then access
 * measurements at run time via \c observations[slot] — a direct vector index
 * with no hash overhead.
 *
 * An observation is absent (blank field in RINEX) when
 * \c observations[slot].hasMeasurementValue() returns \c false.
 */
struct SatelliteObservation
{
    //------------------------------------------------------------//
    //                      Member Variables                      //
    //------------------------------------------------------------//

    SatId                       satelliteId;
    /*!< Identifier of the satellite (constellation + PRN). */

    std::vector<ObsMeasurement> observations;
    /*!< Slot-indexed observation values.
     *   Size == observationTypes[system].size() for this satellite's constellation. */
};

//------------------------------------------------------------//
//                    One Receiver Epoch                      //
//------------------------------------------------------------//

/*!
 * \brief All satellite observations for a single receiver epoch.
 *
 * Satellites are stored in the order they appear in the RINEX file (generally
 * sorted by constellation then PRN, but not guaranteed by the format).
 * Sequential iteration over \c satellites is the primary access pattern in
 * the processing pipeline.
 */
struct RinexEpoch
{
    //------------------------------------------------------------//
    //                      Member Variables                      //
    //------------------------------------------------------------//

    GnssTime                          time;
    /*!< Epoch timestamp in the time system declared in the file header. */

    EpochFlag                         flag = EpochFlag::NormalObservation;
    /*!< Epoch flag indicating the nature of this record. */

    double                            receiverClockOffset = 0.0;
    /*!< Optional receiver clock offset in seconds, from the epoch header line.
     *   Valid only when \c hasReceiverClockOffset is \c true. */

    bool                              hasReceiverClockOffset = false;
    /*!< \c true if a receiver clock offset was present in the epoch header. */

    std::vector<SatelliteObservation> satellites;
    /*!< Observations for all tracked satellites in this epoch. */
};

//------------------------------------------------------------//
//                   Observation Type Map                     //
//------------------------------------------------------------//

/*!
 * \brief Maps each constellation to its ordered list of observation codes.
 *
 * Built from the RINEX 3 header record \c SYS / # / OBS TYPES.
 * The vector index for each constellation is the column slot used to
 * index into \c SatelliteObservation::observations.
 *
 * Example entry after parsing:
 * \code
 *   observationTypes[Constellation::GPS] = { C1C, L1C, D1C, S1C, C2W, L2W, C5Q, L5Q }
 *                                            slot 0  1   2   3   4   5   6   7
 * \endcode
 */
using ObservationTypeMap = std::unordered_map<Constellation, std::vector<ObsCode>>;

//------------------------------------------------------------//
//               Approximate Receiver Position                //
//------------------------------------------------------------//

/*!
 * \brief Approximate ECEF receiver position from the RINEX 3 header.
 *
 * Parsed from the \c APPROX POSITION XYZ header record.
 * WGS-84 / ITRF reference frame, coordinates in metres.
 * Used as initial position estimate for SPP and as reference for
 * station-specific corrections (solid Earth tides, ocean loading).
 */
struct ApproximatePosition
{
    double x = 0.0; /*!< X coordinate in the ECEF frame, metres */
    double y = 0.0; /*!< Y coordinate in the ECEF frame, metres */
    double z = 0.0; /*!< Z coordinate in the ECEF frame, metres */
};

//------------------------------------------------------------//
//               Antenna Phase Centre Offset                  //
//------------------------------------------------------------//

/*!
 * \brief Antenna delta offset from the Antenna Reference Point (ARP).
 *
 * Parsed from the RINEX 3 header record \c ANTENNA: DELTA H/E/N.
 * Represents the vector from the ARP to the marker (monument), given
 * in the local body frame of the receiver, in metres.
 */
struct AntennaDeltaOffset
{
    double height = 0.0; /*!< Vertical offset (up), metres */
    double east   = 0.0; /*!< Horizontal offset (east), metres */
    double north  = 0.0; /*!< Horizontal offset (north), metres */
};

//------------------------------------------------------------//
//                  RINEX 3 Observation Header                //
//------------------------------------------------------------//

/*!
 * \brief Parsed representation of all RINEX 3 observation file header records.
 *
 * The \c observationTypes field is the most critical member: it defines the
 * column structure for every satellite data line in the file and must be
 * fully populated before any epoch records are processed.
 *
 * All string fields are trimmed of leading and trailing whitespace during parsing.
 */
struct RinexObsHeader
{
    //------------------------------------------------------------//
    //                      Member Variables                      //
    //------------------------------------------------------------//

    double              rinexVersion          = 0.0;
    char                fileType              = 'O'; /*!< Always 'O' for observation files */
    std::string         programName;                 /*!< From PGM / RUN BY / DATE */
    std::string         runBy;                       /*!< From PGM / RUN BY / DATE */
    std::string         markerName;                  /*!< From MARKER NAME */
    std::string         markerNumber;                /*!< From MARKER NUMBER */
    std::string         markerType;                  /*!< From MARKER TYPE */
    std::string         observerName;                /*!< From OBSERVER / AGENCY */
    std::string         agencyName;                  /*!< From OBSERVER / AGENCY */
    std::string         receiverSerialNumber;        /*!< From REC # / TYPE / VERS */
    std::string         receiverType;                /*!< From REC # / TYPE / VERS */
    std::string         receiverFirmwareVersion;     /*!< From REC # / TYPE / VERS */
    std::string         antennaSerialNumber;         /*!< From ANT # / TYPE */
    std::string         antennaType;                 /*!< From ANT # / TYPE */
    ApproximatePosition approximatePosition;         /*!< From APPROX POSITION XYZ */
    AntennaDeltaOffset  antennaDeltaOffset;          /*!< From ANTENNA: DELTA H/E/N */
    ObservationTypeMap  observationTypes;
    /*!< Constellation → ordered observation code list (column order).
     *   This is the key structure for slot-based per-satellite data access. */
    GnssTime            timeOfFirstObservation;      /*!< From TIME OF FIRST OBS */
    GnssTime            timeOfLastObservation;       /*!< From TIME OF LAST OBS (optional) */
    double              nominalSamplingInterval = 0.0;
    /*!< Nominal sampling interval in seconds from the INTERVAL header record.
     *   0.0 means the record was absent or the interval is irregular. */
    int                 leapSeconds = 0;             /*!< From LEAP SECONDS, 0 if absent */

    //------------------------------------------------------------//
    //                    Slot Lookup Methods                     //
    //------------------------------------------------------------//

    /*!
     * \brief Return the 0-based column slot index for the given constellation and code.
     *
     * The slot index is the position of \p code in the
     * \c observationTypes[system] vector, which equals the index into
     * \c SatelliteObservation::observations for any satellite of that constellation.
     *
     * This method performs a linear scan over the code list for the given system,
     * which is bounded in practice to at most ~15 codes for any constellation and
     * is therefore effectively O(1).
     *
     * \param[in] system  Constellation of the satellite.
     * \param[in] code    Observation code to locate.
     * \return            0-based slot index, or -1 if \p system or \p code is not found.
     */
    int getObservationSlotIndex(Constellation system, ObsCode code) const noexcept;

    /*!
     * \brief Return the total number of observation codes declared for a constellation.
     *
     * Equivalent to \c observationTypes.at(system).size() but safe when the
     * constellation is absent from the file.
     *
     * \param[in] system  Constellation to query.
     * \return            Number of observation codes, or 0 if \p system is not present.
     */
    int getObservationCodeCount(Constellation system) const noexcept;
};

//------------------------------------------------------------//
//                Complete RINEX 3 Observation File           //
//------------------------------------------------------------//

/*!
 * \brief Top-level result of parsing a RINEX 3 observation file.
 *
 * The \c epochs vector is in chronological order matching the file.
 * Sequential iteration over epochs is the primary access pattern in
 * all downstream processing modules (cycle-slip detection, pseudorange
 * modelling, carrier phase modelling, FGO factor assembly).
 */
struct RinexObsData
{
    //------------------------------------------------------------//
    //                      Member Variables                      //
    //------------------------------------------------------------//

    RinexObsHeader          header;
    /*!< Parsed file header containing metadata and observation type definitions. */

    std::vector<RinexEpoch> epochs;
    /*!< All parsed epochs in chronological order. */

    //------------------------------------------------------------//
    //                     Statistics Methods                     //
    //------------------------------------------------------------//

    /*!
     * \brief Return the total number of satellite-epoch pairs across all epochs.
     *
     * Counts every \c SatelliteObservation entry summed over all epochs.
     * Useful for progress reporting and pre-allocation in downstream processors.
     *
     * \return Total satellite-epoch observation count.
     */
    std::size_t getTotalSatelliteEpochCount() const noexcept;
};

} // namespace gnss::rinex
