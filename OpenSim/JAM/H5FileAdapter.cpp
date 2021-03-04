/* -------------------------------------------------------------------------- *
 *                             H5FileAdapter.cpp                              *
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

#include "H5FileAdapter.h"
#include "JAMUtilities.h"
#include <fstream>

using namespace OpenSim;

H5FileAdapter::H5FileAdapter()
{
	_time_is_empty = true;
}

H5FileAdapter* H5FileAdapter::clone() const
{
	return new H5FileAdapter{ *this };
}

void H5FileAdapter::open(const std::string& file_name) 
{
	_file = H5::H5File(file_name, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
}

void H5FileAdapter::close() {
	_file.close();
}

void H5FileAdapter::createGroup(const std::string& group_name) {
    
    if(H5Lexists(_file.getId(), group_name.c_str(), H5P_DEFAULT ) == 0){
       _file.createGroup(group_name);
    }
}

void H5FileAdapter::writeDataSet(const TimeSeriesTable& table, const std::string group_path) 
{
    createGroup(group_path);

    std::vector<std::string> labels =  table.getColumnLabels();

	SimTK::Matrix data_matrix = table.getMatrix();

	hsize_t dim_data[1];
	dim_data[0] = data_matrix.nrow();
		
	for (int i = 0; i < data_matrix.ncol(); ++i) {
        std::string dataset_path = group_path + "/" + labels[i];

		H5::DataSpace dataspace(1, dim_data, dim_data);
		H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);

		H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

		//Allocate space for data
		double* data = (double*)malloc(dim_data[0] * sizeof(double));

		//Set Data Array
		for (int r = 0; r < dim_data[0]; ++r) {
            data[r] = data_matrix(r,i);
		}
		
		dataset.write(&data[0], datatype);

		//Free dynamically allocated memory
		free(data);
	}
}

void H5FileAdapter::writeDataSet2(const TimeSeriesTable& table, const std::string dataset_path) 
{
	SimTK::Matrix data_matrix = table.getMatrix();

	hsize_t dim_data[2];
	dim_data[0] = data_matrix.nrow();
	dim_data[1] = data_matrix.ncol();
		
	H5::DataSpace dataspace(2, dim_data, dim_data);
	H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
	H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

	//Allocate space for data
	double** data = (double**)malloc(dim_data[0] * sizeof(double*));
	data[0] = (double*)malloc(dim_data[1] * dim_data[0] * sizeof(double));
	for (int i = 1; i < dim_data[0]; i++) data[i] = data[0] + i*dim_data[1];

	//Set Data Array
	for (int r = 0; r < dim_data[0]; ++r) {
		for (int c = 0; c < dim_data[1]; ++c) {
			data[r][c] = data_matrix(r, c);
		}
	}

	dataset.write(&data[0][0], datatype);

	//Free dynamically allocated memory
	free(data[0]);
	free(data);
    
    // Write column names as attributes
    std::vector<std::string> att_vector = table.getColumnLabels();

    //DATASPACE
    H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
    const int RANK = 1;
    hsize_t dims[RANK];
    dims[0] = att_vector.size();  //The attribute will have 3 strings
    H5::DataSpace att_datspc(RANK, dims);

    //ATTRIBUTE
    H5::Attribute att(dataset.createAttribute("column_labels" , str_type, att_datspc));

    //Convert the vector into a C string array.
    //Because the input function ::write requires that.
    std::vector<const char *> cStrArray;
    for(int index = 0; index < att_vector.size(); ++index)
    {
        cStrArray.push_back(att_vector[index].c_str());
    }

    //WRITE DATA
    //att_vector must not change during this operation
    att.write(str_type, (void*)&cStrArray[0]);
}


void H5FileAdapter::writeDataSetVec3(const TimeSeriesTableVec3& table, const std::string dataset_path)
{
		
	SimTK::Matrix_<SimTK::Vec3> data_matrix = table.getMatrix();

	hsize_t dim_data[2];
	dim_data[0] = data_matrix.nrow();
	dim_data[1] = 3;

	for (int i = 0; i < data_matrix.ncol(); ++i) {

		H5::DataSpace dataspace(2, dim_data, dim_data);
		H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
		H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

		//Allocate space for data
		double** data = (double**)malloc(dim_data[0] * sizeof(double*));
		data[0] = (double*)malloc(dim_data[1] * dim_data[0] * sizeof(double));
		for (int i = 1; i < dim_data[0]; i++) data[i] = data[0] + i*dim_data[1];

		//Set Data Array
		for (int r = 0; r < dim_data[0]; ++r) {
			for (int c = 0; c < 3; ++c) {
				data[r][c] = data_matrix(r, i)(c);
			}
		}

		dataset.write(&data[0][0], datatype);

		//Free dynamically allocated memory
		free(data[0]);
		free(data);
	}
}

void H5FileAdapter::writeDataSetVector(const TimeSeriesTable& table, const std::string dataset_path)
{
	
	SimTK::Matrix data_matrix = table.getMatrix();

	hsize_t dim_data[2];
	dim_data[0] = data_matrix.nrow();
	dim_data[1] = data_matrix.ncol();
		
	H5::DataSpace dataspace(2, dim_data, dim_data);
	H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
	H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

	//Allocate space for data
	double** data = (double**)malloc(dim_data[0] * sizeof(double*));
	data[0] = (double*)malloc(dim_data[1] * dim_data[0] * sizeof(double));
	for (int i = 1; i < dim_data[0]; i++) data[i] = data[0] + i*dim_data[1];

	//Set Data Array
	for (int r = 0; r < dim_data[0]; ++r) {
		for (int c = 0; c < dim_data[1]; ++c) {
			data[r][c] = data_matrix(r, c);
		}
	}

	dataset.write(&data[0][0], datatype);

	//Free dynamically allocated memory
	free(data[0]);
	free(data);
		
}

void H5FileAdapter::writeStatesDataSet(const TimeSeriesTable& table) {
	std::string states_group_name = "/States";
	_file.createGroup(states_group_name);

	std::vector<std::string> labels = table.getColumnLabels();
	
	SimTK::Matrix data_matrix = table.getMatrix();

	hsize_t dim_data[1];
	dim_data[0] = data_matrix.nrow();

	for (int i = 0; i < data_matrix.ncol(); ++i) {
		SimTK::String label(labels[i]);

		label.replaceAllChar('/','_');
						
		std::string dataset_path = states_group_name + "/" + label;
			
		H5::DataSpace dataspace(1, dim_data, dim_data);
		H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
		H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

		//Allocate space for data
		double* data = (double*)malloc(dim_data[0] * sizeof(double));

		//Set Data Array
		for (int r = 0; r < dim_data[0]; ++r) {
			data[r] = data_matrix(r, i);
		}

		dataset.write(&data[0], datatype);

		//Free dynamically allocated memory
		free(data);
	}
}

void H5FileAdapter::writeComponentGroupDataSet(std::string group_name,
    std::vector<std::string> names,
	std::vector<std::string> output_double_names,
    std::vector<SimTK::Matrix> output_double_values)
{
    createGroup(group_name);

    int i = 0;
    for (std::string comp_name : names) {
        std::string comp_group = group_name + "/" + comp_name;
        createGroup(comp_group);

        int j = 0;
        for (std::string data_label : output_double_names) {

            SimTK::Vector data = output_double_values[i](j);
            writeDataSetSimTKVector(data, comp_group + "/" + data_label);
            j++;
        }
        i++;
    }
}

void H5FileAdapter::writeComponentGroupDataSetVec3(std::string group_name,
    std::vector<std::string> names,
	std::vector<std::string> output_vec3_names,
    std::vector<SimTK::Matrix_<SimTK::Vec3>> output_vec3_values)
{
    createGroup(group_name);

    int i = 0;
    for (std::string comp_name : names) {
        std::string comp_group = group_name + "/" + comp_name;
        createGroup(comp_group);

        int j = 0;
        for (std::string data_label : output_vec3_names) {

            SimTK::Vector_<SimTK::Vec3> data = output_vec3_values[i](j);
            writeDataSetSimTKVectorVec3(data, comp_group + "/" + data_label);
            j++;
        }
        i++;
    }
}

void H5FileAdapter::writeComponentGroupDataSetVector(std::string group_name,
    std::vector<std::string> names,
	std::vector<std::string> output_vector_names,
    std::vector<std::vector<SimTK::Matrix>> output_vector_values)
{
	createGroup(group_name);
    
    int i = 0;
    for (std::string comp_name : names) {
        std::string comp_group = group_name + "/" + comp_name;
		createGroup(comp_group);

        int j = 0;
        for (std::string data_label : output_vector_names) {

            SimTK::Matrix data = output_vector_values[i][j];
            writeDataSetSimTKMatrix(data, comp_group + "/" + data_label);
            j++;
        }
        i++;
    }
}
    
void H5FileAdapter::writeDataSetSimTKVector(const SimTK::Vector& data_vector, const std::string dataset_path) {
	hsize_t dim_data[1];
	dim_data[0] = data_vector.size();

	H5::DataSpace dataspace(1, dim_data, dim_data);
	H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
	H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

	//Allocate space for data
	double* data = (double*)malloc(dim_data[0] * sizeof(double));

	//Set Data Array
	for (int r = 0; r < dim_data[0]; ++r) {
		data[r] = data_vector(r);
	}

	dataset.write(&data[0], datatype);

	//Free dynamically allocated memory
	free(data);
}

void H5FileAdapter::writeDataSetSimTKVectorVec3(const SimTK::Vector_<SimTK::Vec3>& data_vector, const std::string dataset_path) {
	hsize_t dim_data[2];
	dim_data[0] = data_vector.nrow();
	dim_data[1] = 3;
		
	H5::DataSpace dataspace(2, dim_data, dim_data);
	H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
	H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

	//Allocate space for data
	double** data = (double**)malloc(dim_data[0] * sizeof(double*));
	data[0] = (double*)malloc(dim_data[1] * dim_data[0] * sizeof(double));
	for (int i = 1; i < dim_data[0]; i++) data[i] = data[0] + i*dim_data[1];

	//Set Data Array
	for (int r = 0; r < dim_data[0]; ++r) {
		for (int c = 0; c < 3; ++c) {
			data[r][c] = data_vector(r)(c);
		}
	}

	dataset.write(&data[0][0], datatype);

	//Free dynamically allocated memory
	free(data[0]);
	free(data);
}

void H5FileAdapter::writeDataSetSimTKMatrix(const SimTK::Matrix& data_matrix, const std::string dataset_path) {
	hsize_t dim_data[2];
	dim_data[0] = data_matrix.nrow();
	dim_data[1] = data_matrix.ncol();
		
	H5::DataSpace dataspace(2, dim_data, dim_data);
	H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
	H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

	//Allocate space for data
	double** data = (double**)malloc(dim_data[0] * sizeof(double*));
	data[0] = (double*)malloc(dim_data[1] * dim_data[0] * sizeof(double));
	for (int i = 1; i < dim_data[0]; i++) data[i] = data[0] + i*dim_data[1];

	//Set Data Array
	for (int r = 0; r < dim_data[0]; ++r) {
		for (int c = 0; c < dim_data[1]; ++c) {
			data[r][c] = data_matrix(r,c);
		}
	}

	dataset.write(&data[0][0], datatype);

	//Free dynamically allocated memory
	free(data[0]);
	free(data);
}

void H5FileAdapter::writeDataSetSimTKMatrixColumns(const SimTK::Matrix& data, std::vector<std::string> column_dataset_paths) {

	for (int i = 0; i < data.ncol(); ++i) {
		std::string path = column_dataset_paths[i];

		SimTK::Vector data_vec(data.nrow());
		for (int j = 0; j < data.nrow(); ++j) {
			data_vec(j) = data(j, i);
		}
		writeDataSetSimTKVector(data_vec, path);
	}	
}

void H5FileAdapter::writeDataSetSimTKMatrixVec3Columns(const SimTK::Matrix_<SimTK::Vec3>& data, std::vector<std::string> column_dataset_paths) {

	for (int i = 0; i < data.ncol(); ++i) {
		std::string path = column_dataset_paths[i];

		SimTK::Vector_<SimTK::Vec3> data_vec(data.nrow());
		for (int j = 0; j < data.nrow(); ++j) {
			data_vec(j) = data(j, i);
		}
		writeDataSetSimTKVectorVec3(data_vec, path);
	}
}

void H5FileAdapter::writeTimeDataSet(const Array<double>& time) {
	if (_time_is_empty) {

        hsize_t dim_time[1];
		dim_time[0] = time.size();

		H5::DataSpace time_dataspace(1, dim_time, dim_time);
		H5::PredType time_datatype(H5::PredType::NATIVE_DOUBLE);
		H5::DataSet dataset = _file.createDataSet("/time", time_datatype, time_dataspace);

		dataset.write(&time[0], time_datatype);

		_time_is_empty = false;
	}
}
	
H5FileAdapter::OutputTables H5FileAdapter::extendRead(const std::string& fileName) const 
{
    OutputTables output_tables{};
    return output_tables;
};

void H5FileAdapter::extendWrite(const InputTables& tables, const std::string& fileName) const
{
};
