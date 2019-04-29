#include "IMUDataReader.h"

namespace OpenSim {

const std::string IMUDataReader::Orientations{ "orientations" };         // name of table for orientation data
const std::string IMUDataReader::LinearAccelerations{ "linear_accelerations" };  // name of table for acceleration data
const std::string IMUDataReader::MagneticHeading{ "magnetic_heading" };      // name of table for data from Magnetometer (Magnetic Heading)
const std::string IMUDataReader::AngularVelocity{ "angular_velocity" };      // name of table for gyro data (AngularVelocity)

}
