#include <fstream>
#include "Simbody.h"
#include "Exception.h"
#include "FileAdapter.h"
#include "TimeSeriesTable.h"
#include "XsensDataReader.h"

namespace OpenSim {

XsensDataReader* XsensDataReader::clone() const {
    return new XsensDataReader{*this};
}

DataAdapter::OutputTables 
XsensDataReader::extendRead(const std::string& folderName) const {

    std::vector<std::ifstream*> imuStreams;
    std::vector<std::string> labels;
    // files specified by prefix + file name exist
    double dataRate = SimTK::NaN;
    
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
    
    std::string prefix = _settings.get_trial_prefix();
    std::map<std::string, std::string> headersKeyValuePairs;
    for (int index = 0; index < n_imus; ++index) {
        std::string prefix = _settings.get_trial_prefix();
        const ExperimentalSensor& nextItem = _settings.get_ExperimentalSensors(index);
        auto fileName = folderName + prefix + nextItem.getName() +".txt";
        auto* nextStream = new std::ifstream{ fileName };
        OPENSIM_THROW_IF(!nextStream->good(),
            FileDoesNotExist,
            fileName);
        // Add imu name to labels
        labels.push_back(nextItem.get_name_in_model());
        // Add corresponding stream to imuStreams
        imuStreams.push_back(nextStream);

        // Skip lines to get to data
        std::string line;
        auto commentLine = true;
        std::getline(*nextStream, line);
        auto isCommentLine = [](std::string aline) {
            return aline.substr(0, 2) == "//";
        };
        std::vector<std::string> tokens;
        while (commentLine) {
            // Comment lines of arbitrary number on the form // "key":"value"
            tokens =FileAdapter::tokenize(line.substr(2), ":"); //Skip leading 2 chars tokenize on ':'
            // Will populate map from first file/imu only, assume they all have same format
            if (tokens.size() == 2 && index == 0) {
                // Put values in map
                headersKeyValuePairs[tokens[0]] = tokens[1];
            }
            std::getline(*nextStream, line);
            commentLine = isCommentLine(line);
        }
        // Find indices for Acc_{X,Y,Z}, Gyr_{X,Y,Z},
        // Mag_{X,Y,Z}, Mat on first non-comment line
        tokens = FileAdapter::tokenize(line, "\t");
        
        if (accIndex == -1) accIndex = find_index(tokens, "Acc_X");
        if (gyroIndex == -1) gyroIndex = find_index(tokens, "Gyr_X");
        if (magIndex == -1) magIndex = find_index(tokens, "Mag_X");
        if (rotationsIndex == -1) rotationsIndex = find_index(tokens, "Mat[1][1]");
    }
    // Compute data rate based on key/value pair if available
    std::map<std::string, std::string>::iterator it =
            headersKeyValuePairs.find("Update Rate");
    if (it != headersKeyValuePairs.end())
        dataRate = std::stod(it->second);
    else
        dataRate = 40.0; // Need confirmation from XSens as later files don't specify rate
    // internally keep track of what data was found in input files
    bool foundLinearAccelerationData = (accIndex != -1);
    bool foundMagneticHeadingData = (magIndex != -1);
    bool foundAngularVelocityData = (gyroIndex != -1);

    // If no Orientation data is available we'll abort completely
    OPENSIM_THROW_IF((rotationsIndex == -1), TableMissingHeader);
    
    // For all tables, will create row, stitch values from different files then append,time and timestep
    // are based on the first file
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
        for (std::vector<std::ifstream*>::iterator it = imuStreams.begin();
            it != imuStreams.end();
            ++it, ++imu_index) {
            std::ifstream* nextStream = *it;
            // parse gyro info from imuStream
            std::vector<std::string> nextRow = FileAdapter::getNextLine(*nextStream, "\t\r");
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
        }
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

int XsensDataReader::find_index(std::vector<std::string>& tokens, const std::string& keyToMatch) {
    int returnIndex = -1;
    std::vector<std::string>::iterator it = std::find(tokens.begin(), tokens.end(), keyToMatch);
    if (it != tokens.end())
        returnIndex = static_cast<int>(std::distance(tokens.begin(), it));
    return returnIndex;
}

}
