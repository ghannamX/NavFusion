#pragma once

#include <Eigen/Core>

namespace gnss
{

//------------------------------------------------------------------------------//
//                    Sagnac (Earth Rotation) Correction                         //
//------------------------------------------------------------------------------//

/*!
 * \brief Applies the Sagnac effect correction for Earth rotation during signal travel.
 *
 * During the signal travel time from satellite to receiver (approximately
 * 67-87 ms), the Earth rotates. The satellite ECEF position at transmission
 * time must be rotated into the ECEF frame at reception time.
 *
 * The rotation matrix around the Z-axis by angle omega_e * tau is:
 *
 *     | cos(theta)   sin(theta)   0 |
 *     | -sin(theta)  cos(theta)   0 |
 *     | 0            0            1 |
 *
 * where theta = omega_e * tau, omega_e is the WGS-84 Earth rotation rate,
 * and tau is the signal travel time.
 *
 * For GPS orbit altitude, theta is approximately 5e-9 rad, giving a
 * correction of approximately 30 metres in satellite position.
 */
class SagnacCorrection
{
public:
    //--------------------------------------------------------------------------//
    //                         Public Interface                                 //
    //--------------------------------------------------------------------------//

    /*!
     * \brief Rotate satellite ECEF position to account for Earth rotation
     *        during signal travel time.
     *
     * \param[in] satellitePositionAtTransmission  Satellite ECEF position at
     *            transmission time [m].
     * \param[in] signalTravelTimeSeconds  Signal travel time from satellite
     *            to receiver [seconds].
     *
     * \return Satellite ECEF position corrected for Earth rotation [m].
     */
    static Eigen::Vector3d applySagnacCorrectionToSatellitePosition(
        const Eigen::Vector3d &satellitePositionAtTransmission,
        double signalTravelTimeSeconds);
};

} // namespace gnss
