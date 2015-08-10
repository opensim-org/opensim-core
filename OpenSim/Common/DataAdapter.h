/* -------------------------------------------------------------------------- *
 *                            OpenSim:  DataAdapter.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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


#ifndef OPENSIM_COMMON_DATAADAPTER_H
#define OPENSIM_COMMON_DATAADAPTER_H

// Non-standard headers.
#include "SimTKcommon.h"
#include "OpenSim/Common/Exception.h"

// Standard headers.
#include <string>
#include <istream>
#include <unordered_map>
#include <memory>
#include <functional>
#include <vector>
#include <iterator>

#include <iostream>


namespace OpenSim {

namespace util {

std::vector<std::string> splitString(const std::string& str, 
                                     const std::string& delims) {
    using size_type = std::string::size_type;

    std::vector<std::string> tokens{};

    size_type token_start{0}, token_end{0};
    bool is_token{false};
    while(token_end < str.length()) {
        if(delims.find_first_of(str[token_end]) != std::string::npos) {
            if(is_token) {
                tokens.push_back(str.substr(token_start, 
                                            token_end - token_start));
                is_token = false;
            }
        } else {
            if(!is_token) {
                token_start = token_end;
                is_token = true;
            }
        }

        ++token_end;
    }

    return tokens;
}

}


// class DuplicateDataAdapter : public Exception {
// public:
//     DuplicateDataAdapter(const std::string& expl) : Exception(expl) {}
// };

class DataAdapterNotFound : public Exception {
public:
    DataAdapterNotFound(const std::string& expl) : Exception{expl} {}
};

class MetadataKeyValueLengthMismatch : public Exception {
public:
    MetadataKeyValueLengthMismatch(const std::string& expl) : Exception{expl} {}
};

class NumberOfColumnsMismatch : public Exception {
public:
    NumberOfColumnsMismatch(const std::string& expl) : Exception{expl} {}
};

class NumberOfRowsMismatch : public Exception {
public:
    NumberOfRowsMismatch(const std::string& expl) : Exception(expl) {}
};

class UnexpectedColumnLabel : public Exception {
public:
    UnexpectedColumnLabel(const std::string& expl) : Exception(expl) {}
};


class DataAdapter {
public:
    DataAdapter() : isRegistered_{false} {};
    DataAdapter(bool isRegistered) : isRegistered_{isRegistered} {}
    virtual ~DataAdapter() {}

    bool isRegistered() const {
        return isRegistered_;
    }

private:
    const bool isRegistered_;
};

using DataAdapterGenerator = 
    std::function<std::unique_ptr<DataAdapter> (std::istream&)>;
using RegisteredDataAdapters = 
    std::unordered_map<std::string, DataAdapterGenerator>;

RegisteredDataAdapters& registeredDataAdapters() {
    static RegisteredDataAdapters registered_data_adapters{};

    return registered_data_adapters;
}


DataAdapterGenerator getDataAdapterGenerator(const std::string& extension) {
    try {
        return registeredDataAdapters().at(extension);
    } catch(std::out_of_range&) {
        throw DataAdapterNotFound{"No DataAdapter was found among the "
                "registered DataAdapters for the extension: " + extension + 
                ". DataAdapters must be registered before use. If multiple "
                "DataAdapters registered for same extension, the latest "
                "registration is kept."};
    }
}


std::unique_ptr<DataAdapter> getDataAdapter(const std::string& extension,
                                            std::istream& inStream) {
    return getDataAdapterGenerator(extension)(inStream);
}


template<typename Adapter>
class DataAdapterBase : public DataAdapter {
protected:
    static bool registerThisDataAdapter() {
        auto adapter_generator = [] (std::istream& in_stream) {
            return std::unique_ptr<DataAdapter>{new Adapter{in_stream}};
        };
        auto result = 
            registeredDataAdapters().emplace(Adapter::getExtension(), 
                                             adapter_generator);
        return result.second;
    }
    static const bool isRegistered_;

    DataAdapterBase() : DataAdapter(isRegistered_) {}
};
template<typename Adapter>
const bool DataAdapterBase<Adapter>::isRegistered_{registerThisDataAdapter()};


class TRCAdapter : public DataAdapterBase<TRCAdapter> {
public:
    TRCAdapter(std::istream& in_stream) : in_stream_{in_stream} {
        // First line of the stream is considered the header.
        std::string line{};
        std::getline(in_stream_, line);
        metadata_.emplace("header", line);

        // Read the line containing metadata keys.
        std::getline(in_stream_, line);
        auto keys = util::splitString(line, delimiters_);

        // Read the line containing metadata values.
        std::getline(in_stream_, line);
        auto values = util::splitString(line, delimiters_);

        if(keys.size() != values.size())
            throw MetadataKeyValueLengthMismatch{"Number of metadata keys and"
                    " values do not match"};

        // Fill up the metadata container.
        for(std::size_t i = 0; i < keys.size(); ++i)
            metadata_.emplace(keys[i], values[i]);

        // Read the line containing column labels and fill up the column labels
        // container.
        std::getline(in_stream_, line);
        auto column_labels_ = util::splitString(line, delimiters_);

        // Column 0 should be the frame number. Check and get rid of it as it is
        // not used. The whole column is discarded as the data is read in.
        if(column_labels_[0] != frame_num_column_label_)
            throw UnexpectedColumnLabel{"Expected label for column 0 to be '" +
                    frame_num_column_label_ + "' but found it to be '" +
                    column_labels_[0] + "'."};
        column_labels_.erase(column_labels_.begin());

        // Column 0 (originally column 1 before removing frame number) should
        // now be the time column. Check and get rid of it. The data in this
        // column is maintained separately from rest of the data.
        if(column_labels_[0] != time_column_label_)
            throw UnexpectedColumnLabel{"Expected label for column 1 to be '" +
                    time_column_label_ + "' but found it to be '" +
                    column_labels_[0] + "'."};
        column_labels_.erase(column_labels_.begin());

        // Read in the next line of column labels containing (Xdd, Ydd, Zdd)
        // tuples where dd is a 1 or 2 digit subscript. For example --
        // X1, Y1, Z1, X2, Y2, Z2, ... so on.
        // Check and ignore these labels.
        std::getline(in_stream_, line);
        auto xyz_labels_found = util::splitString(line, delimiters_);
        auto num_markers_expected = 
            std::stoul(metadata_.at(num_markers_label_));
        decltype(xyz_labels_found) xyz_labels_expected{};
        xyz_labels_expected.reserve(num_markers_expected * 3);
        for(int i = 1; i <= num_markers_expected; ++i)
            for(auto& letter : {x_label_, y_label_, z_label_})
                xyz_labels_expected.push_back(letter + std::to_string(i));
        if(xyz_labels_found != xyz_labels_expected)
            throw UnexpectedColumnLabel{"Expected secondary column labels to "
                    "be of form X1, Y1, Z1, X2, Y2, Z2, ... so on. There must "
                    "be one (X,Y,Z) triplet per marker."};
        
        // Read the rows one at a time and fill up the time column container and
        // the data container.
        auto num_frames_expected = std::stoul(metadata_.at(num_frames_label_));
        data_.reserve(num_frames_expected);
        std::size_t row_num{data_starts_at_row_ - 1};
        while(std::getline(in_stream_, line)) {
            auto row = util::splitString(line, delimiters_);
            ++row_num;

            if(row.size() == 0)
                continue;

            if(row.size() != column_labels_.size() * 3 + 2)
                throw NumberOfColumnsMismatch{"There are " + 
                        std::to_string(column_labels_.size() * 3 + 2) + 
                        " column labels but row " + std::to_string(row_num) + 
                        " contains " + std::to_string(row.size()) + 
                        " columns."};

            // Column 0 is frame number, discard it. Column 1 is time, keep it.
            time_column_.push_back(std::stod(row[1]));

            // Columns 2 till the end are data.
            std::vector<SimTK::Vec3> row_vector{};
            row_vector.reserve(std::stoul(metadata_.at(num_markers_label_)));
            for(std::size_t c = 2; c < column_labels_.size() * 3 + 2; c += 3)
                row_vector.push_back({std::stod(row[c]),
                                      std::stod(row[c+1]),
                                      std::stod(row[c+2])});
                
            data_.push_back(std::move(row_vector));
        }

        if(data_.size() != num_frames_expected)
            throw NumberOfRowsMismatch{"Expected " + 
                    std::to_string(num_frames_expected) + " frames but found " +
                    std::to_string(data_.size()) + " frames."};
    }

    static std::string getExtension() {
        return "trc";
    }

    const std::unordered_map<std::string, std::string>& getMetaData() {
        return metadata_;
    }

    const std::vector<std::string>& getColumnLabels() {
        return column_labels_;
    }

    const std::vector<double>& getTime() {
        return time_column_;
    }

    const std::vector<std::vector<SimTK::Vec3>>& getData() {
        return data_;
    }

private:
    std::istream&                                in_stream_;
    std::unordered_map<std::string, std::string> metadata_;
    std::vector<std::string>                     column_labels_;
    std::vector<double>                          time_column_;
    std::vector<std::vector<SimTK::Vec3>>        data_;
    static const std::string                     delimiters_;
    static const std::string                     newline_;
    static const std::string                     frame_num_column_label_;
    static const std::string                     time_column_label_;
    static const std::string                     x_label_;
    static const std::string                     y_label_;
    static const std::string                     z_label_;
    static const std::string                     num_markers_label_;
    static const std::string                     num_frames_label_;
    static const unsigned                        data_starts_at_row_;
};
const std::string TRCAdapter::delimiters_{" \t"};
const std::string TRCAdapter::newline_{"\n"};
const std::string TRCAdapter::frame_num_column_label_{"Frame#"};
const std::string TRCAdapter::time_column_label_{"Time"};
const std::string TRCAdapter::x_label_{"X"};
const std::string TRCAdapter::y_label_{"Y"};
const std::string TRCAdapter::z_label_{"Z"};
const std::string TRCAdapter::num_markers_label_{"NumMarkers"};
const std::string TRCAdapter::num_frames_label_{"NumFrames"};
const unsigned    TRCAdapter::data_starts_at_row_{7};

} // namepsace OpenSim

#endif // OPENSIM_COMMON_DATAADAPTER_H
