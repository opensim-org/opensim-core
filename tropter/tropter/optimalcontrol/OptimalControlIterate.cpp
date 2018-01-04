// ----------------------------------------------------------------------------
// tropter: OptimalControlIterate.cpp
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include "OptimalControlIterate.h"
#include <tropter/Exception.hpp>

#include <fstream>

// For interpolating.
#include <unsupported/Eigen/Splines>

using namespace tropter;

OptimalControlIterate::OptimalControlIterate(const std::string& filepath) {
    TROPTER_THROW_IF(filepath.empty(), "filepath is empty.");

    std::ifstream f(filepath);
    TROPTER_THROW_IF(!f, "Could not read '%s'.", filepath);

    // Grab the number of state, control, and parameter variables.
    // -----------------------------------------------------------
    std::string line;
    TROPTER_THROW_IF(!std::getline(f, line) || line.find("num_states=") != 0,
            "Could not read num_states from '%s'.", filepath);
    std::string num_states_str = line.substr(line.find('=') + 1);
    int num_states = std::stoi(num_states_str);

    TROPTER_THROW_IF(!std::getline(f, line) || line.find("num_controls=") != 0,
            "Could not read num_controls from '%s'.", filepath);
    std::string num_controls_str = line.substr(line.find('=') + 1);
    int num_controls = std::stoi(num_controls_str);

    TROPTER_THROW_IF(!std::getline(f, line) || 
            line.find("num_parameters=") != 0,
            "Could not read num_parameters from '%s'.", filepath);
    std::string num_parameters_str = line.substr(line.find('=') + 1);
    int num_parameters = std::stoi(num_parameters_str);


    // Grab the column labels.
    // -----------------------
    std::getline(f, line);
    std::stringstream column_labels(line);
    std::string label;
    // Get each label from column_labels, using "," as the delimiter.
    std::getline(column_labels, label, ','); // skip over "time"
    TROPTER_THROW_IF(label != "time",
            "In '%s', expected first column to be 'time', but got '%s'.",
            filepath, label);
    int i_label = 0;
    while (std::getline(column_labels, label, ',')) {
        // States come first.
        if (i_label < num_states)
            state_names.push_back(label);
        else if (i_label < num_states + num_controls)
            control_names.push_back(label);
        else
            parameter_names.push_back(label);
        ++i_label;
    }
    TROPTER_THROW_IF(i_label != num_states + num_controls + num_parameters,
            "In '%s', expected %i columns but got %i columns.",
            // Add 1 for the time column.
            filepath, 1 + num_states + num_controls + num_parameters, 
            1 + i_label);

    // Get number of times.
    // --------------------
    // Remember the location where the data starts.
    const auto data_start = f.tellg();
    // Count lines.
    int num_times = 0;
    while (std::getline(f, line)) ++num_times;
    // Go back to the start of the data.
    f.clear();
    f.seekg(data_start, std::ios_base::beg);

    // Read in data.
    // -------------
    time.resize(num_times);
    // File and in-memory matrices are transposed.
    states.resize(num_states, num_times);
    controls.resize(num_controls, num_times);
    parameters.resize(num_parameters);
    std::string element;
    // For each line of data.
    int i_time = 0;
    while (std::getline(f, line)) {
        std::stringstream line_ss(line);
        // Grab the time for this row.
        std::getline(line_ss, element, ',');
        std::stringstream element_ss(element);
        element_ss >> time[i_time];
        // For each element in this row.
        int i_var = 0;
        while (std::getline(line_ss, element, ',')) {
            std::stringstream element_ss(element);
            if (i_var < num_states)
                element_ss >> states(i_var, i_time);
            else if (i_var < num_states + num_controls)
                element_ss >> controls(i_var - num_states, i_time);
            else 
                if (i_time == 0)
                    element_ss >> parameters(i_var - num_states - num_controls);
            ++i_var;
        }
        ++i_time;
    }

    f.close();
}

namespace {

    // We can use Eigen's Spline module for linear interpolation, though it's
    // not really meant for this.
    // https://eigen.tuxfamily.org/dox/unsupported/classEigen_1_1Spline.html
    // The independent variable must be between [0, 1].
    using namespace Eigen;
    RowVectorXd normalize(RowVectorXd x) {
        const double lower = x[0];
        const double denom = x.tail<1>()[0] - lower;
        for (Index i = 0; i < x.size(); ++i) {
            // We assume that x is non-decreasing.
            x[i] = (x[i] - lower) / denom;
        }
        return x;
    }

    MatrixXd interp1(const RowVectorXd& xin, const MatrixXd yin,
            const RowVectorXd& xout) {
        // Make sure we're not extrapolating.
        assert(xout[0] >= xin[0]);
        assert(xout.tail<1>()[0] <= xin.tail<1>()[0]);

        typedef Spline<double, 1> Spline1d;

        MatrixXd yout(yin.rows(), xout.size());
        RowVectorXd xin_norm = normalize(xin);
        RowVectorXd xout_norm = normalize(xout);
        for (Index irow = 0; irow < yin.rows(); ++irow) {
            const Spline1d spline = SplineFitting<Spline1d>::Interpolate(
                    yin.row(irow), // dependent variable.
                    1, // linear interp
                    xin_norm); // "knot points" (independent variable).
            for (Index icol = 0; icol < xout.size(); ++icol) {
                yout(irow, icol) = spline(xout_norm[icol]).value();
            }
        }
        return yout;
    }
}

OptimalControlIterate
OptimalControlIterate::interpolate(int desired_num_columns) const {
    if (time.size() == desired_num_columns) return *this;

    assert(desired_num_columns > 0);
    TROPTER_THROW_IF(!std::is_sorted(time.data(), time.data() + time.size()),
            "Expected time to be non-decreasing.");

    OptimalControlIterate out;
    out.state_names = state_names;
    out.control_names = control_names;
    out.time = Eigen::RowVectorXd::LinSpaced(desired_num_columns,
                                             time[0], time.tail<1>()[0]);
    out.states = interp1(time, states, out.time);
    out.controls = interp1(time, controls, out.time);
    return out;
}

/// Write the states and controls trajectories to a plain-text CSV file.
void OptimalControlIterate::write(const std::string& filepath) const {
    std::ofstream f(filepath);

    // Header.
    f << "num_states=" << states.rows() << std::endl;
    f << "num_controls=" << controls.rows() << std::endl;
    f << "num_parameters=" << parameters.rows() << std::endl;

    // Column labels.
    f << "time";
    if (state_names.size() == (size_t)states.rows() &&
            control_names.size() == (size_t)controls.rows() &&
            parameter_names.size() == (size_t)parameters.rows()) {
        for (int i_state = 0; i_state < states.rows(); ++i_state)
            f << "," << state_names[i_state];
        for (int i_control = 0; i_control < controls.rows(); ++i_control)
            f << "," << control_names[i_control];
        for (int i_parameter = 0; i_parameter < parameters.rows(); 
                ++i_parameter)
            f << "," << parameter_names[i_parameter];
    } else {
        for (int i_state = 0; i_state < states.rows(); ++i_state)
            f << ",state" << i_state;
        for (int i_control = 0; i_control < controls.rows(); ++i_control)
            f << ",control" << i_control;
        for (int i_parameter = 0; i_parameter < parameters.rows(); 
                ++i_parameter)
            f << ",parameter" << i_parameter;
    }
    f << std::endl;

    // Data.
    for (int i_mesh = 0; i_mesh < time.size(); ++i_mesh) {
        f << time[i_mesh];
        for (int i_state = 0; i_state < states.rows(); ++i_state)
            f << "," << states(i_state, i_mesh);
        for (int i_control = 0; i_control < controls.rows(); ++i_control)
            f << "," << controls(i_control, i_mesh);
        for (int i_parameter = 0; i_parameter < parameters.rows(); 
                ++i_parameter)
            if (i_mesh == 0) {
                f << "," << parameters(i_parameter);
            } else {
                f << "," << std::numeric_limits<double>::quiet_NaN();
            }
            
        f << std::endl;
    }
    f.close();
}
