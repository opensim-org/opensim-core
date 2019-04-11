#include "IMUDataUtilities.h"

namespace OpenSim {

const std::string IMUDataUtilities::Orientations{ "orientations" };         // name of table for orientation data
const std::string IMUDataUtilities::LinearAccelerations{ "linear_accelerations" };  // name of table for acceleration data
const std::string IMUDataUtilities::MagneticHeading{ "magnetic_heading" };      // name of table for data from Magnetometer (Magnetic Heading)
const std::string IMUDataUtilities::AngularVelocity{ "angular_velocity" };      // name of table for gyro data (AngularVelocity)

}
