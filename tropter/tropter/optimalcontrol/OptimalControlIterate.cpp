
#include "OptimalControlIterate.h"
#include <fstream>

using namespace tropter;

OptimalControlIterate::OptimalControlIterate(const std::string& filepath) {
    if (filepath.empty()) throw std::runtime_error("filepath is empty.");

    std::ifstream f(filepath);
    if (!f) throw std::runtime_error("Could not read '" + filepath + "'.");

    // Grab the number of state and control variables.
    // -----------------------------------------------
    std::string line;
    if (!std::getline(f, line) || line.find("num_states=") != 0)
        throw std::runtime_error("Could not read num_states.");
    std::string num_states_str = line.substr(line.find('=') + 1);
    int num_states = std::stoi(num_states_str);

    if (!std::getline(f, line) || line.find("num_controls=") != 0)
        throw std::runtime_error("Could not read num_controls.");
    std::string num_controls_str = line.substr(line.find('=') + 1);
    int num_controls = std::stoi(num_controls_str);

    // Grab the column labels.
    // -----------------------
    std::getline(f, line);
    std::stringstream column_labels(line);
    std::string label;
    // Get each label from column_labels, using "," as the delimiter.
    std::getline(column_labels, label, ','); // skip over "time"
    if (label != "time")
        throw std::runtime_error("Expected first column to be 'time', "
                                         "but got '" + label + "'.");
    int i_label = 0;
    while (std::getline(column_labels, label, ',')) {
        // States come first.
        if (i_label < num_states)
            state_names.push_back(label);
        else
            control_names.push_back(label);
        ++i_label;
    }
    if (i_label != num_states + num_controls) {
        // Add 1 for the time column.
        throw std::runtime_error("Expected " +
                std::to_string(1 + num_states + num_controls) +
                " columns but got " + std::to_string(1 + i_label) +
                " columns.");
    }

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
            else
                element_ss >> controls(i_var - num_states, i_time);
            ++i_var;
        }
        ++i_time;
    }

    f.close();
}

/// Write the states and controls trajectories to a plain-text CSV file.
void OptimalControlIterate::write(const std::string& filepath) const {
    std::ofstream f(filepath);

    // Header.
    f << "num_states=" << states.rows() << std::endl;
    f << "num_controls=" << controls.rows() << std::endl;

    // Column labels.
    f << "time";
    for (int i_state = 0; i_state < states.rows(); ++i_state) {
        f << "," << state_names[i_state];
    }
    for (int i_control = 0; i_control < controls.rows(); ++i_control) {
        f << "," << control_names[i_control];
    }
    f << std::endl;

    // Data.
    for (int i_mesh = 0; i_mesh < time.size(); ++i_mesh) {
        f << time[i_mesh];
        for (int i_state = 0; i_state< states.rows(); ++i_state) {
            f << "," << states(i_state, i_mesh);
        }
        for (int i_control = 0; i_control< controls.rows(); ++i_control) {
            f << "," << controls(i_control, i_mesh);
        }
        f << std::endl;
    }
    f.close();
}
