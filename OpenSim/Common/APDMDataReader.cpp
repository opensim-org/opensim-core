#include <fstream>
#include "Simbody.h"
#include "Exception.h"
#include "FileAdapter.h"
#include "TimeSeriesTable.h"
#include "APDMDataReader.h"

namespace OpenSim {

const std::vector<std::string> APDMDataReader::acceleration_labels{
        "/Acceleration/X", "/Acceleration/Y", "/Acceleration/Z"
}; 
const std::vector<std::string> APDMDataReader::angular_velocity_labels{
    "/Angular Velocity/X", "/Angular Velocity/Y","/Angular Velocity/Z"
};
const std::vector<std::string> APDMDataReader::magnetic_heading_labels{
    "/Magnetic Field/X", "/Magnetic Field/Y","/Magnetic Field/Z"
};
const std::vector<std::string> APDMDataReader::orientation_labels{
    "/Orientation/Scalar", "/Orientation/X", "/Orientation/Y","/Orientation/Z"
};

const std::string APDMDataReader::TimeLabel{ "Time" };

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

    std::vector<std::string> labels; // will be written to output tables
    
    double dataRate = SimTK::NaN;
    std::vector<int> accIndex;
    std::vector<int>  gyroIndex;
    std::vector<int>  magIndex;
    std::vector<int>  orientationsIndex;

    int n_imus = _settings.getProperty_ExperimentalSensors().size();
    int last_size = 1024;
    // Will read data into pre-allocated Matrices in-memory rather than appendRow
    // on the fly which copies the whole table on every call.
    SimTK::Matrix_<SimTK::Quaternion> rotationsData{ last_size, n_imus };
    SimTK::Matrix_<SimTK::Vec3> linearAccelerationData{ last_size, n_imus };
    SimTK::Matrix_<SimTK::Vec3> magneticHeadingData{ last_size, n_imus };
    SimTK::Matrix_<SimTK::Vec3> angularVelocityData{ last_size, n_imus };
    std::vector<double> times;
    times.resize(last_size);
    // We support two formats, they contain similar data but headers are different
    std::string line;
    // Line 1
    std::getline(in_stream, line);
    std::vector<std::string> tokens = FileAdapter::tokenize(line, ",");
    bool newFormat = false;
    if (tokens[0] == "Format=7") {
        newFormat = true;
        dataRate = 128; // Will fix after reading computing it from time column
        // Header Line 1:Format=7, [I1,,,$IMU1,,,,,,,,,,,]*
        // Header Line 2: Time,[Accelerometer,,,Gyroscope,,,Magnetometer,,,Barometer,Orientation,,,]*
        // Header Line 3: ,[X,Y,Z,X,Y,Z,X,Y,Z,,S,X,Y,Z]*
        std::vector<std::string> emptyLabels;
        emptyLabels.push_back("");
        // In this format there's no dataRate, either assumed or computed from Time column
        for (int imu_index = 0; imu_index < n_imus; ++imu_index) {
            std::string sensorName = _settings.get_ExperimentalSensors(imu_index).getName();
            labels.push_back(_settings.get_ExperimentalSensors(imu_index).get_name_in_model());
            find_start_column(tokens, emptyLabels, sensorName, accIndex, newFormat);
            if (accIndex[imu_index] != -1) {
                gyroIndex.push_back(accIndex[imu_index] + 3);
                magIndex.push_back(accIndex[imu_index] + 6);
                orientationsIndex.push_back(accIndex[imu_index] + 10);
            }
            else
                OPENSIM_THROW(Exception, "Data for sensor:" +sensorName + "was not found in data file "+ fileName+".");
        }
        // Line 2 unused
        std::getline(in_stream, line);
    }
    else {
        // Older Format looks like this:
        // Header Line 1: Test Name:, $String,,,,,..
        // Header Line 2: Sample Rate:, $Value, Hz,,,,,
        // Labels Line 3: Time {SensorName/Acceleration/X,SensorName/Acceleration/Y,SensorName/Acceleration/Z,....} repeated per sensor
        // Units Line 4: s,{m/s^2,m/s^2,m/s^2....} repeated 
        int header_lines = 4;

        std::string trialName = tokens[1]; // May contain spaces
        // Line 2
        std::getline(in_stream, line);
        tokens = FileAdapter::tokenize(line, ",");
        dataRate = std::stod(tokens[1]);
        // Line 3, find columns for IMUs
        std::getline(in_stream, line);
        tokens = FileAdapter::tokenize(line, ",");
        OPENSIM_THROW_IF((tokens[0] != TimeLabel), UnexpectedColumnLabel,
            fileName,
            TimeLabel,
            tokens[0]);

        for (int imu_index = 0; imu_index < n_imus; ++imu_index) {
            std::string sensorName = _settings.get_ExperimentalSensors(imu_index).getName();
            labels.push_back(_settings.get_ExperimentalSensors(imu_index).get_name_in_model());
            find_start_column(tokens, APDMDataReader::acceleration_labels, sensorName, accIndex);
            find_start_column(tokens, APDMDataReader::angular_velocity_labels, sensorName, gyroIndex);
            find_start_column(tokens, APDMDataReader::magnetic_heading_labels, sensorName, magIndex);
            find_start_column(tokens, APDMDataReader::orientation_labels, sensorName, orientationsIndex);
        }
    }
    // Will create a table to map 
    // internally keep track of what data was found in input files
    bool foundLinearAccelerationData = accIndex.size()>0;
    bool foundMagneticHeadingData = magIndex.size()>0;
    bool foundAngularVelocityData = gyroIndex.size()>0;

    // If no Orientation data is available we'll abort
    OPENSIM_THROW_IF((orientationsIndex.size() == 0),
        TableMissingHeader);
    // Line 4, Units unused
    std::getline(in_stream, line);

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
        std::vector<std::string> nextRow = FileAdapter::getNextLine(in_stream, ",");
        if (nextRow.empty()) {
            done = true;
            break;
        }
        // Cycle through the imus collating values
        for (int imu_index = 0; imu_index < n_imus; ++imu_index) {
            // parse gyro info from in_stream
           if (foundLinearAccelerationData)
                accel_row_vector[imu_index] = SimTK::Vec3(std::stod(nextRow[accIndex[imu_index]]),
                    std::stod(nextRow[accIndex[imu_index] + 1]), std::stod(nextRow[accIndex[imu_index] + 2]));
            if (foundMagneticHeadingData)
                magneto_row_vector[imu_index] = SimTK::Vec3(std::stod(nextRow[magIndex[imu_index]]),
                    std::stod(nextRow[magIndex[imu_index] + 1]), std::stod(nextRow[magIndex[imu_index] + 2]));
            if (foundAngularVelocityData)
                gyro_row_vector[imu_index] = SimTK::Vec3(std::stod(nextRow[gyroIndex[imu_index]]),
                    std::stod(nextRow[gyroIndex[imu_index] + 1]), std::stod(nextRow[gyroIndex[imu_index] + 2]));
            // Create Quaternion from values in file, assume order in file W, X, Y, Z
            orientation_row_vector[imu_index] = 
                SimTK::Quaternion(std::stod(nextRow[orientationsIndex[imu_index]]),
                    std::stod(nextRow[orientationsIndex[imu_index] + 1]),
                    std::stod(nextRow[orientationsIndex[imu_index] + 2]),
                    std::stod(nextRow[orientationsIndex[imu_index] + 3]));
        }
        // append to the tables
        times[rowNumber] = time;
        if (foundLinearAccelerationData) 
            linearAccelerationData[rowNumber] =  accel_row_vector;
        if (foundMagneticHeadingData) 
            magneticHeadingData[rowNumber] = magneto_row_vector;
        if (foundAngularVelocityData) 
            angularVelocityData[rowNumber] = gyro_row_vector;
        rotationsData[rowNumber] = orientation_row_vector;
        // We could get some indication of time from file or generate time based on rate
        // Here we use the latter mechanism.
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
    times.resize(rowNumber);
    // Repeat for Data matrices in use and create Tables from them or size 0 for empty
    linearAccelerationData.resizeKeep(foundLinearAccelerationData? rowNumber : 0,
        n_imus);
    magneticHeadingData.resizeKeep(foundMagneticHeadingData? rowNumber : 0,
            n_imus);
    angularVelocityData.resizeKeep(foundAngularVelocityData? rowNumber :0,
        n_imus);
    rotationsData.resizeKeep(rowNumber, n_imus);
    // Now create the tables from matrices
    // Create 4 tables for Rotations, LinearAccelerations, AngularVelocity, MagneticHeading
    // Tables could be empty if data is not present in file(s)
    DataAdapter::OutputTables tables = createTablesFromMatrices(dataRate, labels, times,
        rotationsData, linearAccelerationData, magneticHeadingData, angularVelocityData);
    return tables;
}

void APDMDataReader::find_start_column(std::vector<std::string> tokens,
                                       std::vector<std::string> search_labels, const std::string& sensorName,
                                       std::vector<int>& indices, bool newFormat) const {
    // Search for "sensorName/{search_labels} in tokens, append result to indices if found"
    std::string firstLabel = sensorName + search_labels[0];
    // look for first label, when found check/confirm the rest. Out of order is not supported
    int found_index = -1;
    std::vector<std::string>::iterator it =
            std::find(tokens.begin(), tokens.end(), firstLabel);
    if (it != tokens.end()) {
        found_index = static_cast<int>(std::distance(tokens.begin(), it));
        // now check the following indices for match with remaining search_labels
        bool match = true;
        for (int remaining = 1; remaining < (int)search_labels.size() && match;
             remaining++) {
            match = tokens[found_index + remaining].compare(
                    sensorName + search_labels[remaining]) == 0;
        }
        if (match) {
            if (newFormat) {
                // Three extra comma separated fields in header before imu name
                indices.push_back(found_index - 3);
            } else {
                indices.push_back(found_index);
            }
        } else { // first label found but the remaining didn't. Throw
            throw Exception{"Expected labels for sensor " + sensorName +
                            " were not found."};
        }
    }
}

} // namespace OpenSim
