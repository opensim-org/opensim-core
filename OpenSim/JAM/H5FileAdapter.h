#ifndef OPENSIM_H5_FILE_ADAPTER_H_
#define OPENSIM_H5_FILE_ADAPTER_H_
/* -------------------------------------------------------------------------- *
 *                            H5FileAdapter.h                                 *
 * -------------------------------------------------------------------------- *
 * Author(s): Colin Smith                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
//                         H5FileAdapter
//=============================================================================
/**
This class implements enables .h5 (HDF5, https://www.hdfgroup.org/) files to 
be written. The .h5 format is a binary file that can store large amounts of 
data in a compressed file size and can be more quickly written and read than
text file formats such as .sto or .mot. The file has a hierarchical structure
of groups (like a folder directory structure) than can contain subgroups, 
datasets, and metadata describing the datasets. 



@author Colin Smith

*/

#include "OpenSim/Common/FileAdapter.h"
#include "SimTKmath.h"
#include "H5Cpp.h"
#include "hdf5_hl.h"
#include "osimJAMDLL.h"
#include "OpenSim/Common/TimeSeriesTable.h"
#include "OpenSim/Common/Array.h"

namespace OpenSim {

    class OSIMJAM_API H5FileAdapter : public FileAdapter{
    //class H5FileAdapter : public FileAdapter {
    public:
       H5FileAdapter();
       H5FileAdapter(const H5FileAdapter&) = default;
       H5FileAdapter(H5FileAdapter&&) = default;
       H5FileAdapter& operator=(const H5FileAdapter&) = default;
       H5FileAdapter& operator=(H5FileAdapter&&) = default;
       ~H5FileAdapter() = default;

       H5FileAdapter* clone() const override;


       void open(const std::string& file_name);

       void close();

       void createGroup(const std::string& new_group);

       void writeDataSet(const TimeSeriesTable& table, const std::string group_path);
       void writeDataSet2(const TimeSeriesTable& table, const std::string group_path);

       void writeDataSetVec3(const TimeSeriesTableVec3& table, const std::string group_path);

       void writeDataSetVector(const TimeSeriesTable& table, const std::string group_path);

       void writeDataSetSimTKVector(const SimTK::Vector& data_vector, const std::string dataset_path);

       void writeDataSetSimTKVectorVec3(const SimTK::Vector_<SimTK::Vec3>& data_vector, const std::string dataset_path);

       void writeDataSetSimTKMatrix(const SimTK::Matrix& data_matrix, const std::string dataset_path);

       void writeDataSetSimTKMatrixColumns(const SimTK::Matrix& data, std::vector<std::string> column_dataset_paths);

       void writeDataSetSimTKMatrixVec3Columns(const SimTK::Matrix_<SimTK::Vec3>& data, std::vector<std::string> column_dataset_paths);

       void writeTimeDataSet(const Array<double>& time);

       void writeStatesDataSet(const TimeSeriesTable& table);

       void writeComponentGroupDataSet(std::string group_name, std::vector<std::string> names,
           std::vector<std::string> output_double_names,
           std::vector<SimTK::Matrix> output_double_values);

       void writeComponentGroupDataSetVec3(std::string group_name,
           std::vector<std::string> names,
           std::vector<std::string> output_vec3_names,
           std::vector<SimTK::Matrix_<SimTK::Vec3>> output_vec3_values);

       void writeComponentGroupDataSetVector(std::string group_name,
           std::vector<std::string> names,
           std::vector<std::string> output_vector_names,
           std::vector<std::vector<SimTK::Matrix>> output_vector_values);

    protected:
        OutputTables extendRead(const std::string& fileName) const override;

        void extendWrite(const InputTables& tables, const std::string& fileName) const override;
    private:


    //Data
    private:
        H5::H5File _file;
        bool _time_is_empty;

    };

} // namespace OpenSim

#endif // OPENSIM_H5_FILE_ADAPTER_H_
