#include <fstream>
#include "Simbody.h"
#include "Exception.h"
#include "FileAdapter.h"
#include "TimeSeriesTable.h"
#include "APDMDataReader.h"

namespace OpenSim {

const std::string APDMDataReader::Orientations{ "orientations" };
const std::string APDMDataReader::LinearAccelerations{ "linear_accelerations" };
const std::string APDMDataReader::MagneticHeading{ "magnetic_heading" };
const std::string APDMDataReader::AngularVelocity{ "angular_velocity" };

APDMDataReader* APDMDataReader::clone() const {
    return new APDMDataReader{*this};
}

DataAdapter::OutputTables 
APDMDataReader::extendRead(const std::string& fileName) const {

    OPENSIM_THROW_IF(fileName.empty(),
        EmptyFileName);

    std::ifstream in_stream{ fileName };
    OPENSIM_THROW_IF(!in_stream.good(),
        FileDoesNotExist,
        fileName);

    OPENSIM_THROW_IF(in_stream.peek() == std::ifstream::traits_type::eof(),
        FileIsEmpty,
        fileName);

    std::vector<std::string> labels;
    // files specified by prefix + file name exist
    double dataRate = SimTK::NaN;
    int packetCounterIndex = -1;
    int accIndex = -1;
    int gyroIndex = -1;
    int magIndex = -1;
    int rotationsIndex = -1;

    int n_imus = _settings.getProperty_ExperimentalSensors().size();
    int last_size = 1024;
    // Will read data into pre-allocated Matrices in-memory rather than appendRow
    // on the fly to avoid the overhead of 
    SimTK::Matrix_<SimTK::Quaternion> rotationsData{ last_size, n_imus };
    SimTK::Matrix_<SimTK::Vec3> linearAccelerationData{ last_size, n_imus };
    SimTK::Matrix_<SimTK::Vec3> magneticHeadingData{ last_size, n_imus };
    SimTK::Matrix_<SimTK::Vec3> angularVelocityData{ last_size, n_imus };
    std::vector<double> times;
    times.resize(last_size);
    // Format looks like this:
    // Header Line 1: Test Name:, $String,,,,,..
    // Header Line 2: Sample Rate:, $Value, Hz,,,,,
    // Labels Line 3: Time {SensorName/Acceleration/X,SensorName/Acceleration/Y,SensorName/Acceleration/Z,....} repeated per sensor
    // Units Line 4: s,{m/s^2,m/s^2,m/s^2....} repeated 
    int header_lines = 4;
    std::string line;
    // Line 1
    std::getline(in_stream, line);
    std::vector<std::string> tokens = FileAdapter::tokenize(line, ",");
    std::string trialName = tokens[1]; // May contain spaces
    // Line 2
    std::getline(in_stream, line);
    tokens = FileAdapter::tokenize(line, ",");
    dataRate = std::stod(tokens[1]);
    // Line 3, find columns for IMUs
    std::getline(in_stream, line);
    tokens = FileAdapter::tokenize(line, ",");
    //OPENSIM_THROW_IF((tokens[0] != "Time"), UnexpectedColumnLabel);
    // Will create a table to map 
    // internally keep track of what data was found in input files
    bool foundLinearAccelerationData = (accIndex != -1);
    bool foundMagneticHeadingData = (magIndex != -1);
    bool foundAngularVelocityData = (gyroIndex != -1);

    // If no Orientation data is available or dataRate can't be deduced we'll abort completely
    OPENSIM_THROW_IF((rotationsIndex == -1 || SimTK::isNaN(dataRate)),
        TableMissingHeader);
    
    // For all tables, will create row, stitch values from different sensors then append
    bool done = false;
    double time = 0.0;
    double timeIncrement = 1 / dataRate;
    int rowNumber = 0;
    while (!done){
        // Make vectors one per table
        TimeSeriesTableQuaternion::RowVector
            orientation_row_vector{ n_imus, SimTK::Quaternion() };
        TimeSeriesTableVec3::RowVector
            accel_row_vector{ n_imus, SimTK::Vec3(SimTK::NaN) };
        TimeSeriesTableVec3::RowVector
            magneto_row_vector{ n_imus, SimTK::Vec3(SimTK::NaN) };
        TimeSeriesTableVec3::RowVector
            gyro_row_vector{ n_imus, SimTK::Vec3(SimTK::NaN) };
        // Cycle through the filles collating values
        int imu_index = 0;
        // parse gyro info from imuStream
        std::vector<std::string> nextRow = FileAdapter::getNextLine(in_stream, "\t\r");
        if (nextRow.empty()) {
            done = true;
            break;
        }
        if (foundLinearAccelerationData)
            accel_row_vector[imu_index] = SimTK::Vec3(std::stod(nextRow[accIndex]),
                std::stod(nextRow[accIndex + 1]), std::stod(nextRow[accIndex + 2]));
        if (foundMagneticHeadingData)
            magneto_row_vector[imu_index] = SimTK::Vec3(std::stod(nextRow[magIndex]),
                std::stod(nextRow[magIndex + 1]), std::stod(nextRow[magIndex + 2]));
        if (foundAngularVelocityData)
            gyro_row_vector[imu_index] = SimTK::Vec3(std::stod(nextRow[gyroIndex]),
                std::stod(nextRow[gyroIndex + 1]), std::stod(nextRow[gyroIndex + 2]));
        // Create Mat33 then convert into Quaternion
        SimTK::Mat33 imu_matrix{ SimTK::NaN };
        int matrix_entry_index = 0;
        for (int mcol = 0; mcol < 3; mcol++) {
            for (int mrow = 0; mrow < 3; mrow++) {
                imu_matrix[mrow][mcol] = std::stod(nextRow[rotationsIndex + matrix_entry_index]);
                matrix_entry_index++;
            }
        }
        // Convert imu_matrix to Quaternion
        SimTK::Rotation imu_rotation{ imu_matrix };
        orientation_row_vector[imu_index] = imu_rotation.convertRotationToQuaternion();

        if (done) 
            break;
        // append to the tables
        times[rowNumber] = time;
        if (foundLinearAccelerationData) 
            linearAccelerationData[rowNumber] =  accel_row_vector;
        if (foundMagneticHeadingData) 
            magneticHeadingData[rowNumber] = magneto_row_vector;
        if (foundAngularVelocityData) 
            angularVelocityData[rowNumber] = gyro_row_vector;
        rotationsData[rowNumber] = orientation_row_vector;
        time += timeIncrement;
        rowNumber++;
        if (std::remainder(rowNumber, last_size) == 0) {
            // resize all Data/Matrices, double the size  while keeping data
            int newSize = last_size*2;
            times.resize(newSize);
            // Repeat for Data matrices in use
            if (foundLinearAccelerationData) linearAccelerationData.resizeKeep(newSize, n_imus);
            if (foundMagneticHeadingData) magneticHeadingData.resizeKeep(newSize, n_imus);
            if (foundAngularVelocityData) angularVelocityData.resizeKeep(newSize, n_imus);
            rotationsData.resizeKeep(newSize, n_imus);
            last_size = newSize;
        }
    }
    // Trim Matrices in use to actual data and move into tables
    int actualSize = rowNumber;
    times.resize(actualSize);
    // Repeat for Data matrices in use and create Tables from them or size 0 for empty
    linearAccelerationData.resizeKeep(foundLinearAccelerationData? actualSize: 0, 
        n_imus);
    magneticHeadingData.resizeKeep(foundMagneticHeadingData?actualSize: 0, 
            n_imus);
    angularVelocityData.resizeKeep(foundAngularVelocityData?actualSize:0, 
        n_imus);
    rotationsData.resizeKeep(actualSize, n_imus);
    // Now create the tables from matrices
    // Create 4 tables for Rotations, LinearAccelerations, AngularVelocity, MagneticHeading
    // Tables could be empty if data is not present in file(s)
    DataAdapter::OutputTables tables{};
    auto orientationTable = std::make_shared<TimeSeriesTableQuaternion>(times, rotationsData, labels);
    orientationTable->updTableMetaData()
        .setValueForKey("DataRate", std::to_string(dataRate));
    tables.emplace(Orientations, orientationTable);

    std::vector<double> emptyTimes;
    auto accelerationTable = (foundLinearAccelerationData ?
        std::make_shared<TimeSeriesTableVec3>(times, linearAccelerationData, labels) :
        std::make_shared<TimeSeriesTableVec3>(emptyTimes, linearAccelerationData, labels));
    accelerationTable->updTableMetaData()
        .setValueForKey("DataRate", std::to_string(dataRate));
    tables.emplace(LinearAccelerations, accelerationTable);

    auto magneticHeadingTable = (foundMagneticHeadingData ?
        std::make_shared<TimeSeriesTableVec3>(times, magneticHeadingData, labels) :
        std::make_shared<TimeSeriesTableVec3>(emptyTimes, magneticHeadingData, labels));
    magneticHeadingTable->updTableMetaData()
        .setValueForKey("DataRate", std::to_string(dataRate));
    tables.emplace(MagneticHeading, magneticHeadingTable);

    auto angularVelocityTable = (foundAngularVelocityData ?
        std::make_shared<TimeSeriesTableVec3>(times, angularVelocityData, labels) :
        std::make_shared<TimeSeriesTableVec3>(emptyTimes, angularVelocityData, labels));
    angularVelocityTable->updTableMetaData()
        .setValueForKey("DataRate", std::to_string(dataRate));
    tables.emplace(AngularVelocity, angularVelocityTable);

    return tables;
}

}
