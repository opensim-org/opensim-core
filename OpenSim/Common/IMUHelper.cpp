#include "Simbody.h"
#include "Exception.h"
#include "IMUHelper.h"

namespace OpenSim {

const std::string IMUHelper::_orientations{ "orientations" };
const std::string IMUHelper::_accelerations{ "accelerations" };
const std::string IMUHelper::_magnetometers{ "magnetometers" };
const std::string IMUHelper::_gyros{ "gyros" };

DataAdapter::OutputTables IMUHelper::readXsensTrial(const std::string& folderName, const std::string& prefix,
        const std::map<std::string, std::string>& filenameToModelIMUMap) {

    std::vector<std::ifstream*> imuStreams;
    std::vector<std::string> labels;
    // files specified by prefix + filenameToModelIMUMap.values exist
    double dataRate = SimTK::NaN;
    int packetCounterIndex = -1;
    int accIndex = -1;
    int gyrIndex = -1;
    int magIndex = -1;
    int rotationsIndex = -1;

    for (std::map<std::string, std::string>::const_iterator it = filenameToModelIMUMap.cbegin(); 
            it != filenameToModelIMUMap.cend();
            ++it) {
        auto fileName = folderName + prefix + it->first+".txt";
        auto* nextStream = new std::ifstream{ fileName };
        OPENSIM_THROW_IF(!nextStream->good(),
            FileDoesNotExist,
            fileName);
        // Add imu name to labels
        labels.push_back(it->second);
        // Add corresponding stream to imuStreams
        imuStreams.push_back(nextStream);

        // Skip lines to get to data
        std::string line;
        for (int j = 0; j < 6; j++) {
            std::getline(*nextStream, line);
            if (j == 1 && SimTK::isNaN(dataRate)) { // Extract Data rate from line 1
                std::vector<std::string> tokens = FileAdapter::tokenize(line, ", ");
                // find Update Rate: and parse into dataRate
                if (tokens.size() < 4) continue;
                if (tokens[1] == "Update" && tokens[2] == "Rate:") {
                    dataRate = std::stod(tokens[3]);
                }
            }
            if (j == 5) { // Find indices for PacketCounter, Acc_{X,Y,Z}, Gyr_{X,Y,Z}, Mag_{X,Y,Z} on line 5
                std::vector<std::string> tokens = FileAdapter::tokenize(line, "\t");
                if (packetCounterIndex == -1) packetCounterIndex = find_index(tokens, "PacketCounter");
                if (accIndex == -1) accIndex = find_index(tokens, "Acc_X");
                if (gyrIndex == -1) gyrIndex = find_index(tokens, "Gyr_X");
                if (magIndex == -1) magIndex = find_index(tokens, "Mag_X");
                if (rotationsIndex == -1) rotationsIndex = find_index(tokens, "Mat[1][1]");
            }
        }
    }
    // Create 4 tables for Rotations, Acc[elerations], Gyr[o], Mag[netometer] data
    DataAdapter::OutputTables tables{};
    size_t n_imus = filenameToModelIMUMap.size();

    auto orientationTable = std::make_shared<TimeSeriesTableQuaternion>();
    orientationTable->setColumnLabels(labels);
    orientationTable->updTableMetaData()
        .setValueForKey("DataRate", std::to_string(dataRate));
    tables.emplace(_orientations, orientationTable);

    auto accelerationTable = std::make_shared<TimeSeriesTableVec3>();
    accelerationTable->setColumnLabels(labels);
    accelerationTable->updTableMetaData()
        .setValueForKey("DataRate", std::to_string(dataRate));
    tables.emplace(_accelerations, accelerationTable);

    auto magnetometerTable = std::make_shared<TimeSeriesTableVec3>();
    magnetometerTable->setColumnLabels(labels);
    magnetometerTable->updTableMetaData()
        .setValueForKey("DataRate", std::to_string(dataRate));
    tables.emplace(_magnetometers, magnetometerTable);

    auto gyrosTable = std::make_shared<TimeSeriesTableVec3>();
    gyrosTable->setColumnLabels(labels);
    gyrosTable->updTableMetaData()
        .setValueForKey("DataRate", std::to_string(dataRate));
    tables.emplace(_gyros, gyrosTable);
    
    // For all tables, will create row, stitch values from different files then append,time and timestep
    // are based on the first file
    std::vector<std::string> row = FileAdapter::getNextLine(*imuStreams.at(0), "\t\r");
    double time = 0.0;
    double timeIncrement = 1 / dataRate;
    while (!row.empty()){
        // Make vectors one per table
        TimeSeriesTableQuaternion::RowVector
            orientation_row_vector{ static_cast<int>(n_imus), SimTK::Quaternion() };
        TimeSeriesTableVec3::RowVector
            accel_row_vector{ static_cast<int>(n_imus), SimTK::Vec3(SimTK::NaN) };
        TimeSeriesTableVec3::RowVector
            magneto_row_vector{ static_cast<int>(n_imus), SimTK::Vec3(SimTK::NaN) };
        TimeSeriesTableVec3::RowVector
            gyro_row_vector{ static_cast<int>(n_imus), SimTK::Vec3(SimTK::NaN) };
        // Cycle through the filles collating values
        int imu_index = 0;
        for (std::vector<std::ifstream*>::iterator it = imuStreams.begin();
            it != imuStreams.end();
            ++it, ++imu_index) {
            std::ifstream* nextStream = *it;
            // parse gyro info from imuStream
            std::vector<std::string> nextRow = FileAdapter::getNextLine(*nextStream, "\t\r");
            if (imu_index == 0) // Move row to next line
                row = nextRow;
            if (row.empty())
                break;
            accel_row_vector[imu_index] = SimTK::Vec3(std::stod(nextRow[accIndex]),
                std::stod(nextRow[accIndex + 1]), std::stod(nextRow[accIndex + 2]));
            magneto_row_vector[imu_index] = SimTK::Vec3(std::stod(nextRow[magIndex]),
                std::stod(nextRow[magIndex + 1]), std::stod(nextRow[magIndex + 2]));
            gyro_row_vector[imu_index] = SimTK::Vec3(std::stod(nextRow[gyrIndex]),
                std::stod(nextRow[gyrIndex + 1]), std::stod(nextRow[gyrIndex + 2]));
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
        if (row.empty()) 
            break;
        // append to the tables
        accelerationTable->appendRow(time, std::move(accel_row_vector));
        magnetometerTable->appendRow(time, std::move(magneto_row_vector));
        gyrosTable->appendRow(time, std::move(gyro_row_vector));
        orientationTable->appendRow(time, std::move(orientation_row_vector));
        time += timeIncrement;
    }

    return tables;
}

int IMUHelper::find_index(std::vector<std::string>& tokens, const std::string& keyToMatch) {
    int returnIndex = -1;
    std::vector<std::string>::iterator it = std::find(tokens.begin(), tokens.end(), keyToMatch);
    if (it != tokens.end())
        returnIndex = std::distance(tokens.begin(), it);
    return returnIndex;
}

}
/*
TimeSeriesTable_<SimTK::Quaternion> readRotationsFromXSensFiles(const std::string& directory)
{
auto currentDir = IO::getCwd();
IO::chDir(directory);

std::vector<std::string> imuFiles = {
"MT_012005D6_009-000_00B42268.txt",
"MT_012005D6_009-000_00B42279.txt",
"MT_012005D6_009-000_00B4227D.txt",
"MT_012005D6_009-000_00B4227C.txt",
"MT_012005D6_009-000_00B421ED.txt",
"MT_012005D6_009-000_00B421EE.txt",
"MT_012005D6_009-000_00B421EF.txt",
"MT_012005D6_009-000_00B421E6.txt"
};

std::vector<std::string> imu_names = {
"back",
"pelvis",
"r.tibia",
"r.femur",
"l.tibia",
"l.femur",
"r.foot",
"l.foot"
};

size_t n_imus = imuFiles.size();
assert(n_imus == imu_names.size());

std::vector<std::ifstream*> imuStreams;
std::string line;

std::vector<std::string> labels{ n_imus };

for (size_t i = 0; i < n_imus; i++) {
auto* imuStream = new std::ifstream(imuFiles[i]);

// skip the first six lines which are the data header
for (int j = 0; j < 6; j++) {
getline(*imuStream, line);
}

imuStreams.push_back(imuStream);
labels[i] = imu_names[i] + "_imu";
}

size_t chunk = 2000;

SimTK::Matrix_<SimTK::Quaternion> rotationsData{ int(chunk), int(n_imus) };
std::vector<double> times;
times.resize(chunk);

// Read imu data line by line
size_t nl{ 0 };

SimTK::Vec3 imu_acceleration{ SimTK::NaN };
SimTK::Mat33 imu_matrix{ SimTK::NaN };
long int val;

while (!(*imuStreams[0]).eof()) {
for (int i = 0; i < n_imus; ++i) {
// current imu's data stream
auto* imu_data = imuStreams[i];
// extract the first value, which is packet number
*imu_data >> val;
// extract the acceleration columns
*imu_data >> imu_acceleration;
// extract the rotation matrix as column vectors
for (int col = 0; col < 3; ++col) {
for (int row = 0; row < 3; ++row) {
*imu_data >> imu_matrix[row][col];
}
}

if (imu_matrix.isNaN()) {
cout << "Found NaN data at line " << nl << ", skipping to next line." << endl;
break;
}

SimTK::Rotation imu_rotation{ imu_matrix };
rotationsData[int(nl)][i] = imu_rotation.convertRotationToQuaternion();
}
times[nl] = 0.01*nl;
++nl;

if (std::remainder(nl, chunk) == 0) {
rotationsData.resizeKeep(int(nl + chunk), int(n_imus));
times.resize(nl + chunk);
}
}

// Trim data back to the actual data
rotationsData.resizeKeep(int(nl), int(n_imus));
times.resize(nl);


TimeSeriesTable_<SimTK::Quaternion> rotationsTable(times, rotationsData, labels);

STOFileAdapter_<SimTK::Quaternion>::write(rotationsTable, "imuOrientations.sto");

return rotationsTable;
}
*/