#include "XsensDataReader.h"

#include "CommonUtilities.h"
#include "Exception.h"
#include "FileAdapter.h"
#include "Simbody.h"
#include "TimeSeriesTable.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <unordered_map>
#include <vector>

namespace OpenSim {

XsensDataReader* XsensDataReader::clone() const {
    return new XsensDataReader{*this};
}

typedef struct XsensDataReader::XsensIMU {
    std::string name;
    size_t data_size;
    std::string rotation_format;
    std::map<std::string, std::string> comments;
    std::map<std::string, size_t> header;
    std::vector<SimTK::Vec3> acc;
    std::vector<SimTK::Vec3> gyr;
    std::vector<SimTK::Vec3> mag;
    std::vector<SimTK::Quaternion> quat;
    std::vector<SimTK::Rotation> euler;
    std::vector<SimTK::Mat33> rot_mat;
} XsensIMU;

DataAdapter::OutputTables XsensDataReader::extendRead(
        const std::string& folderName) const {

    // Valid delimiters for Xsens files
    const std::vector<std::string> delimiters = {",", ";", "|", "\t", ":", " "};

    // Valid headers for data file, header vector is in order
    const std::unordered_map<std::string, std::vector<std::string>> imu_h = {
            {"acc", {"Acc_X", "Acc_Y", "Acc_Z"}},
            {"gyr", {"Gyr_X", "Gyr_Y", "Gyr_Z"}},
            {"mag", {"Mag_X", "Mag_Y", "Mag_Z"}},
            {"quat", {"Quat_q0", "Quat_q1", "Quat_q2", "Quat_q3"}},
            {"euler", {"Roll", "Pitch", "Yaw"}},
            {"rot_mat", {"Mat[1][1]", "Mat[2][1]", "Mat[3][1]", "Mat[1][2]",
                                "Mat[2][2]", "Mat[3][2]", "Mat[1][3]",
                                "Mat[2][3]", "Mat[3][3]"}}};

    // files specified by prefix + file name exist
    double dataRate = _settings.get_sampling_rate();
    const std::string extension = _settings.get_trial_extension();
    const std::string prefix = _settings.get_trial_prefix();

    const int n_imus = _settings.getProperty_ExperimentalSensors().size();

    // Prepare all the file handles
    // format: name, filename
    std::vector<std::pair<std::string, std::string>> imuFiles;
    for (int index = 0; index < n_imus; ++index) {
        std::string prefix = _settings.get_trial_prefix();
        const ExperimentalSensor& nextItem =
                _settings.get_ExperimentalSensors(index);
        const std::filesystem::path fileName =
                std::filesystem::path(folderName) /
                (prefix + nextItem.getName() + extension);

        // Add corresponding pair to imuStreams
        const auto& p =
                std::make_pair(nextItem.get_name_in_model(), fileName.string());
        imuFiles.push_back(p);
    }

    std::vector<XsensIMU> imus(imuFiles.size());

    // Read each IMU file
    std::transform(imuFiles.begin(), imuFiles.end(), imus.begin(),
            [&imu_h, &delimiters](const auto& p) {
                // Open File
                const std::string& fileName = p.second;
                std::ifstream stream{fileName};
                OPENSIM_THROW_IF(!stream.good(), FileDoesNotExist, fileName);

                // Build IMU Sensor struct
                XsensIMU imu;
                imu.name = p.first;

                const auto isCommentLine = [](std::string aline) {
                    return aline.substr(0, 2) == "//";
                };
                std::string line;
                std::vector<std::string> tokens;
                while (std::getline(stream, line) && isCommentLine(line)) {
                    // Comment lines of arbitrary number on the form
                    // // "key":"value"
                    // Skip leading 2 chars tokenize on ':'
                    tokens = FileAdapter::tokenize(line.substr(2), ":");
                    if (tokens.size() == 2) {
                        imu.comments.insert({tokens[0], tokens[1]});
                    }
                }
                const auto delim = OpenSim::detectDelimiter(line,delimiters);
                OPENSIM_THROW_IF(delim == "", TableMissingHeader,
                        "No delimiter found for: " + imu.name +
                                " Please ensure that the data file is valid!");
                const std::string delimiter = delim;
                // This is the header
                // Find indices for Acc_{X,Y,Z}, Gyr_{X,Y,Z},
                // Mag_{X,Y,Z}, Mat on first non-comment line
                tokens = FileAdapter::tokenize(line, delimiter);
                // Process each header in the line
                for (const auto& token : tokens) {
                    size_t tokenIndex = std::distance(tokens.begin(),
                            std::find(tokens.begin(), tokens.end(), token));
                    imu.header.insert({token, tokenIndex});
                }

                const auto is_group_complete =
                        [&](const std::vector<std::string>& search_set,
                                const std::map<std::string, size_t>&
                                        found_headers) -> bool {
                    return std::all_of(search_set.begin(), search_set.end(),
                            [&](const auto& p) {
                                return found_headers.find(p) !=
                                       found_headers.end();
                            });
                };

                const auto& acc_h = imu_h.at("acc");
                const auto& gyr_h = imu_h.at("gyr");
                const auto& mag_h = imu_h.at("mag");
                const auto& quat_h = imu_h.at("quat");
                const auto& euler_h = imu_h.at("euler");
                const auto& rot_mat_h = imu_h.at("rot_mat");

                // internally keep track of what data was found in input
                const bool found_linear_acceleration =
                        is_group_complete(acc_h, imu.header);
                const bool found_magnetic_heading =
                        is_group_complete(mag_h, imu.header);
                const bool found_angular_velocity =
                        is_group_complete(gyr_h, imu.header);
                const bool found_quat = is_group_complete(quat_h, imu.header);
                const bool found_euler = is_group_complete(euler_h, imu.header);
                const bool found_rot_mat =
                        is_group_complete(rot_mat_h, imu.header);

                int rotation_reps_found =
                        found_quat + found_euler + found_rot_mat;

                // Determine the rotation format to be used in the order:
                // 1. Quaternion => (best for OpenSim)
                // 2. Rotation Matrix
                // 3. Euler Angles
                if (found_quat) {
                    imu.rotation_format = "rot_quaternion";
                } else if (found_rot_mat) {
                    imu.rotation_format = "rot_matrix";
                } else if (found_euler) {
                    imu.rotation_format = "rot_euler";
                }

                std::vector<std::string> next_row;
                size_t row_count = 0;
                while (!(next_row = FileAdapter::getNextLine(
                                 stream, delimiter + "\r"))
                                .empty()) {
                    // parse info from imuStream
                    if (found_linear_acceleration)
                        imu.acc.push_back(SimTK::Vec3(
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(acc_h[0])]),
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(acc_h[1])]),
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(acc_h[2])])));
                    if (found_magnetic_heading)
                        imu.mag.push_back(SimTK::Vec3(
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(mag_h[0])]),
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(mag_h[1])]),
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(mag_h[2])])));
                    if (found_angular_velocity)
                        imu.gyr.push_back(SimTK::Vec3(
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(gyr_h[0])]),
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(gyr_h[1])]),
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(gyr_h[2])])));
                    if (found_quat) {
                        imu.quat.push_back(SimTK::Quaternion(
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(quat_h[0])]),
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(quat_h[1])]),
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(quat_h[2])]),
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(quat_h[3])])));
                    }
                    if (found_euler) {
                        imu.euler.push_back(SimTK::Rotation(
                                SimTK::BodyOrSpaceType::BodyRotationSequence,
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(euler_h[0])]),
                                SimTK::XAxis,
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(euler_h[1])]),
                                SimTK::YAxis,
                                OpenSim::IO::stod(
                                        next_row[imu.header.at(euler_h[2])]),
                                SimTK::ZAxis));
                    }
                    if (found_rot_mat) {
                        // Create Mat33 then convert into Quaternion
                        SimTK::Mat33 imu_matrix{SimTK::NaN};
                        imu_matrix[0][0] = OpenSim::IO::stod(
                                next_row[imu.header.at(rot_mat_h[0])]);
                        imu_matrix[1][0] = OpenSim::IO::stod(
                                next_row[imu.header.at(rot_mat_h[1])]);
                        imu_matrix[2][0] = OpenSim::IO::stod(
                                next_row[imu.header.at(rot_mat_h[2])]);

                        imu_matrix[0][1] = OpenSim::IO::stod(
                                next_row[imu.header.at(rot_mat_h[3])]);
                        imu_matrix[1][1] = OpenSim::IO::stod(
                                next_row[imu.header.at(rot_mat_h[4])]);
                        imu_matrix[2][1] = OpenSim::IO::stod(
                                next_row[imu.header.at(rot_mat_h[5])]);

                        imu_matrix[0][2] = OpenSim::IO::stod(
                                next_row[imu.header.at(rot_mat_h[6])]);
                        imu_matrix[1][2] = OpenSim::IO::stod(
                                next_row[imu.header.at(rot_mat_h[7])]);
                        imu_matrix[2][2] = OpenSim::IO::stod(
                                next_row[imu.header.at(rot_mat_h[8])]);

                        imu.rot_mat.push_back(imu_matrix);
                    }
                    row_count++;
                }
                imu.data_size = row_count;
                return imu;
            });

    // Ensure all files have the same size, update rate, and rotation
    // format
    std::vector<size_t> line_lengths(imus.size());
    std::vector<double> update_rates(imus.size());
    std::vector<std::string> rotation_formats(imus.size());

    std::transform(imus.begin(), imus.end(), line_lengths.begin(),
            [](const auto& imu) { return imu.data_size; });

    std::transform(imus.begin(), imus.end(), update_rates.begin(),
            [&dataRate](const auto& imu) {
                const auto it = imu.comments.find("Update Rate");
                return (it != imu.comments.end())
                               ? OpenSim::IO::stod(it->second)
                               : dataRate;
            });

    std::transform(imus.begin(), imus.end(), rotation_formats.begin(),
            [](const auto& imu) { return imu.rotation_format; });

    // Check for uniformity
    OPENSIM_THROW_IF(
            std::adjacent_find(line_lengths.begin(), line_lengths.end(),
                    std::not_equal_to<>()) != line_lengths.end(),
            IOError,
            "All data files in the trial do not have the same line length!");

    OPENSIM_THROW_IF(
            std::adjacent_find(update_rates.begin(), update_rates.end(),
                    std::not_equal_to<>()) != update_rates.end(),
            IOError,
            "All data files in the trial do not have the same update rate!");

    OPENSIM_THROW_IF(
            std::adjacent_find(rotation_formats.begin(), rotation_formats.end(),
                    std::not_equal_to<>()) != rotation_formats.end(),
            IOError,
            "All data files in the trial do not have the same rotation "
            "format!");

    // These have all been checked for uniformity
    const size_t n_lines = line_lengths[0];
    const double sampling_rate = update_rates[0];
    const std::string rotation_format = rotation_formats[0];

    OPENSIM_THROW_IF(
            (n_lines > static_cast<size_t>(std::numeric_limits<int>::max())),
            IOError, "Too many lines present in the data files!");
    // Will read data into pre-allocated Matrices in-memory rather than
    // appendRow on the fly to avoid the overhead of
    SimTK::Matrix_<SimTK::Quaternion> rotationsData{
            static_cast<int>(n_lines), n_imus};
    SimTK::Matrix_<SimTK::Vec3> linearAccelerationData{
            static_cast<int>(n_lines), n_imus};
    SimTK::Matrix_<SimTK::Vec3> magneticHeadingData{
            static_cast<int>(n_lines), n_imus};
    SimTK::Matrix_<SimTK::Vec3> angularVelocityData{
            static_cast<int>(n_lines), n_imus};

    const bool has_acc = std::all_of(imus.begin(), imus.end(),
            [&n_lines](const auto& imu) { return imu.acc.size() == n_lines; });
    const bool has_gyr = std::all_of(imus.begin(), imus.end(),
            [&n_lines](const auto& imu) { return imu.gyr.size() == n_lines; });
    const bool has_mag = std::all_of(imus.begin(), imus.end(),
            [&n_lines](const auto& imu) { return imu.mag.size() == n_lines; });

    // We use a vector of row indices to transform over the rows (i.e., the time
    // steps)
    std::vector<int> row_indices(n_lines);
    std::iota(row_indices.begin(), row_indices.end(),
            0); // Fill with 0, 1, 2, ..., n_lines-1

    // For all tables, will create the row by stitching values from all read
    // files
    std::transform(row_indices.begin(), row_indices.end(), row_indices.begin(),
            [&imus, &n_imus, &rotation_format, &n_lines, &has_acc, &has_gyr,
                    &has_mag, &linearAccelerationData, &magneticHeadingData,
                    &angularVelocityData, &rotationsData](int j) {
                TimeSeriesTableQuaternion::RowVector orientation_row_vector(
                        n_imus, SimTK::Quaternion());
                TimeSeriesTableVec3::RowVector accel_row_vector(
                        n_imus, SimTK::Vec3(SimTK::NaN));
                TimeSeriesTableVec3::RowVector magneto_row_vector(
                        n_imus, SimTK::Vec3(SimTK::NaN));
                TimeSeriesTableVec3::RowVector gyro_row_vector(
                        n_imus, SimTK::Vec3(SimTK::NaN));
                const bool has_quat = std::all_of(
                        imus.begin(), imus.end(), [&n_lines](const auto& imu) {
                            return imu.quat.size() == n_lines;
                        });
                const bool has_euler = std::all_of(
                        imus.begin(), imus.end(), [&n_lines](const auto& imu) {
                            return imu.euler.size() == n_lines;
                        });
                const bool has_rot_mat = std::all_of(
                        imus.begin(), imus.end(), [&n_lines](const auto& imu) {
                            return imu.rot_mat.size() == n_lines;
                        });
                if (has_acc) {
                    std::transform(imus.begin(), imus.end(),
                            std::begin(accel_row_vector),
                            [j](const auto& imu) { return imu.acc[j]; });
                    linearAccelerationData[j] = accel_row_vector;
                }

                if (has_mag) {
                    std::transform(imus.begin(), imus.end(),
                            std::begin(magneto_row_vector),
                            [j](const auto& imu) { return imu.mag[j]; });
                    magneticHeadingData[j] = magneto_row_vector;
                }

                if (has_gyr) {
                    std::transform(imus.begin(), imus.end(),
                            std::begin(gyro_row_vector),
                            [j](const auto& imu) { return imu.gyr[j]; });
                    angularVelocityData[j] = gyro_row_vector;
                }

                if (rotation_format == "rot_quaternion" && has_quat) {
                    std::transform(imus.begin(), imus.end(),
                            std::begin(orientation_row_vector),
                            [j](const auto& imu) { return imu.quat[j]; });
                } else if (rotation_format == "rot_euler" && has_euler) {
                    std::transform(imus.begin(), imus.end(),
                            std::begin(orientation_row_vector),
                            [j](const auto& imu) {
                                return imu.euler[j]
                                        .convertRotationToQuaternion();
                            });
                } else if (rotation_format == "rot_matrix" && has_rot_mat) {
                    std::transform(imus.begin(), imus.end(),
                            std::begin(orientation_row_vector),
                            [j](const auto& imu) {
                                SimTK::Rotation imu_rotation{imu.rot_mat[j]};
                                return imu_rotation
                                        .convertRotationToQuaternion();
                            });
                }
                // Store the rows in the matrices
                rotationsData[j] = orientation_row_vector;

                return j; // Return the index required by std::transform
            });
    const double timeIncrement = 1.0 / sampling_rate;
    const auto times = createVectorLinspaceInterval(
            static_cast<int>(n_lines), 0.0, timeIncrement);
    // Zero data matrices if the data is not found
    if (!has_acc) { linearAccelerationData.resize(0, n_imus); }
    if (!has_mag) { magneticHeadingData.resize(0, n_imus); }
    if (!has_gyr) { angularVelocityData.resize(0, n_imus); }

    // Now create the tables from matrices
    // Create 4 tables for Rotations, LinearAccelerations,
    // AngularVelocity, MagneticHeading Tables could be empty if data is
    // not present in file(s)
    std::vector<std::string> labels;
    std::transform(imus.begin(), imus.end(), std::back_inserter(labels),
            [](auto const& p) { return p.name; });
    DataAdapter::OutputTables tables = createTablesFromMatrices(sampling_rate,
            labels, times, rotationsData, linearAccelerationData,
            magneticHeadingData, angularVelocityData);
    return tables;
}

} // namespace OpenSim
