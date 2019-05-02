#include "IMUDataReader.h"

namespace OpenSim {

    const std::string IMUDataReader::Orientations{ "orientations" };         // name of table for orientation data
    const std::string IMUDataReader::LinearAccelerations{ "linear_accelerations" };  // name of table for acceleration data
    const std::string IMUDataReader::MagneticHeading{ "magnetic_heading" };      // name of table for data from Magnetometer (Magnetic Heading)
    const std::string IMUDataReader::AngularVelocity{ "angular_velocity" };      // name of table for gyro data (AngularVelocity)

    DataAdapter::OutputTables IMUDataReader::createTablesFromMatrices(double dataRate,
        const std::vector<std::string>& labels, const std::vector<double>& times,
        const SimTK::Matrix_<SimTK::Quaternion>& rotationsData,
        const SimTK::Matrix_<SimTK::Vec3>& linearAccelerationData,
        const SimTK::Matrix_<SimTK::Vec3>& magneticHeadingData,
        const SimTK::Matrix_<SimTK::Vec3>& angularVelocityData) const {

        DataAdapter::OutputTables tables{};

        auto orientationTable = std::make_shared<TimeSeriesTableQuaternion>(times, rotationsData, labels);
        orientationTable->updTableMetaData()
            .setValueForKey("DataRate", std::to_string(dataRate));
        tables.emplace(Orientations, orientationTable);

        std::vector<double> emptyTimes;
        bool foundLinearAccelerationData = linearAccelerationData.nrow()>0;
        auto accelerationTable = (foundLinearAccelerationData ?
            std::make_shared<TimeSeriesTableVec3>(times, linearAccelerationData, labels) :
            std::make_shared<TimeSeriesTableVec3>(emptyTimes, linearAccelerationData, labels));
        accelerationTable->updTableMetaData()
            .setValueForKey("DataRate", std::to_string(dataRate));
        tables.emplace(LinearAccelerations, accelerationTable);
        bool foundMagneticHeadingData = magneticHeadingData.nrow()>0;
        auto magneticHeadingTable = (foundMagneticHeadingData ?
            std::make_shared<TimeSeriesTableVec3>(times, magneticHeadingData, labels) :
            std::make_shared<TimeSeriesTableVec3>(emptyTimes, magneticHeadingData, labels));
        magneticHeadingTable->updTableMetaData()
            .setValueForKey("DataRate", std::to_string(dataRate));
        tables.emplace(MagneticHeading, magneticHeadingTable);

        bool foundAngularVelocityData = angularVelocityData.nrow()>0;
        auto angularVelocityTable = (foundAngularVelocityData ?
            std::make_shared<TimeSeriesTableVec3>(times, angularVelocityData, labels) :
            std::make_shared<TimeSeriesTableVec3>(emptyTimes, angularVelocityData, labels));
        angularVelocityTable->updTableMetaData()
            .setValueForKey("DataRate", std::to_string(dataRate));
        tables.emplace(AngularVelocity, angularVelocityTable);

        return tables;

    }
}
