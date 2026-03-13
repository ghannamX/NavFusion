#include "satellite/SagnacCorrection.hpp"
#include "common/wgs84Constants.hpp"

#include <cmath>

namespace gnss
{

//------------------------------------------------------------------------------//
//                   Sagnac (Earth Rotation) Correction                          //
//------------------------------------------------------------------------------//

Eigen::Vector3d SagnacCorrection::applySagnacCorrectionToSatellitePosition(
    const Eigen::Vector3d &satellitePositionAtTransmission,
    double signalTravelTimeSeconds)
{
    const double rotationAngle =
        wgs84::EARTH_ROTATION_RATE * signalTravelTimeSeconds;

    const double cosTheta = std::cos(rotationAngle);
    const double sinTheta = std::sin(rotationAngle);

    const double correctedX =
        cosTheta * satellitePositionAtTransmission.x()
      + sinTheta * satellitePositionAtTransmission.y();

    const double correctedY =
       -sinTheta * satellitePositionAtTransmission.x()
      + cosTheta * satellitePositionAtTransmission.y();

    return Eigen::Vector3d{
        correctedX,
        correctedY,
        satellitePositionAtTransmission.z()
    };
}

} // namespace gnss
