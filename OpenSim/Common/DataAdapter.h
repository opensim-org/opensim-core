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
#include "TimeSeriesTable.h"


// Standard headers.
#include <string>
#include <unordered_map>
#include <memory>
#include <functional>
#include <vector>
#include <iterator>

#include <fstream>

#include <iostream>


namespace OpenSim {

class DataAdapter {
public:
    using RegisteredDataAdapters = 
        std::unordered_map<std::string, std::unique_ptr<DataAdapter>>;

    virtual DataAdapter* clone() const = 0;

    DataAdapter() = default;
    DataAdapter(const DataAdapter&) = default;

    virtual ~DataAdapter() {}

    static
    void registerDataAdapter(const std::string& identifier,
                             const DataAdapter& adapter) {
        if(registered_data_adapters.find(identifier) != 
           registered_data_adapters.end())
            throw Exception{"DataAdapter::registerDataAdapter() adapter for '" +
                    identifier + "' already registered."};

        auto kv = std::make_pair(identifier, 
                                 std::unique_ptr<DataAdapter>{adapter.clone()});

        registered_data_adapters.insert(std::move(kv));
    }

    static
    std::unique_ptr<DataAdapter> createAdapter(const std::string& identifier) {
        try {
            DataAdapter* adapter = 
                registered_data_adapters.at(identifier)->clone();
            return std::unique_ptr<DataAdapter>{adapter};
        } catch(std::out_of_range&) {
            throw Exception{"No DataAdapter was found among the "
                    "registered DataAdapters for the identifier: " + identifier 
                    + ". DataAdapters must be registered before use. If "
                    "multiple DataAdapters registered for same identifier, the "
                    "latest registration is kept."};
        }
    }

    virtual void prepareForReading(AbstractDataTable& datatable) = 0;

    virtual void read() = 0;

private:
    static RegisteredDataAdapters registered_data_adapters;
};
DataAdapter::RegisteredDataAdapters DataAdapter::registered_data_adapters{};


class FileAdapter : public DataAdapter {
public:
    FileAdapter() = default;

    static
    std::unique_ptr<FileAdapter> createAdapter(const std::string& identifier) {
        auto data_adapter_ptr = 
            DataAdapter::createAdapter(identifier).release();
        FileAdapter* file_adapter_ptr = 
            dynamic_cast<FileAdapter*>(data_adapter_ptr);
        return std::unique_ptr<FileAdapter>{file_adapter_ptr};
    }

    void setFilename(const std::string& filename) {
        filename_ = filename;
    }

    const std::string& getFilename() const {
        return filename_;
    }

    static
    std::string findExtension(const std::string& filename) {
        std::size_t found = filename.find_last_of('.');
        return found == std::string::npos ? 
            std::string{} : filename.substr(found + 1);
    }

    std::vector<std::string> tokenize(const std::string& str, 
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

protected:
    std::string filename_;
};


class TRCAdapter : public FileAdapter {
public:
    using Table = TimeSeriesTable_<SimTK::Vec3>;
    
    TRCAdapter* clone() const override {
        return new TRCAdapter{*this};
    }

    void prepareForReading(AbstractDataTable& datatable) override {
        table_ = &dynamic_cast<Table&>(datatable);
    }

    void read() override {
        if(filename_.empty())
            throw Exception{"Input filename is not set."};

        std::ifstream in_stream{filename_};

        // First line of the stream is considered the header.
        std::string line{};
        std::getline(in_stream, line);
        table_->insertMetaData("header", line);

        // Read the line containing metadata keys.
        std::getline(in_stream, line);
        auto keys = tokenize(line, delimiters_);

        // Read the line containing metadata values.
        std::getline(in_stream, line);
        auto values = tokenize(line, delimiters_);

        if(keys.size() != values.size())
            throw Exception{"Number of metadata keys and values do not match"};

        // Fill up the metadata container.
        for(std::size_t i = 0; i < keys.size(); ++i)
            table_->insertMetaData(keys[i], values[i]);

        // Read the line containing column labels and fill up the column labels
        // container.
        std::getline(in_stream, line);
        auto column_labels_ = tokenize(line, delimiters_);

        // Column 0 should be the frame number. Check and get rid of it as it is
        // not used. The whole column is discarded as the data is read in.
        if(column_labels_[0] != frame_num_column_label_)
            throw Exception{"Expected label for column 0 to be '" +
                    frame_num_column_label_ + "' but found it to be '" +
                    column_labels_[0] + "'."};
        column_labels_.erase(column_labels_.begin());

        // Column 0 (originally column 1 before removing frame number) should
        // now be the time column. Check and get rid of it. The data in this
        // column is maintained separately from rest of the data.
        if(column_labels_[0] != time_column_label_)
            throw Exception{"Expected label for column 1 to be '" +
                    time_column_label_ + "' but found it to be '" +
                    column_labels_[0] + "'."};
        column_labels_.erase(column_labels_.begin());

        // Read in the next line of column labels containing (Xdd, Ydd, Zdd)
        // tuples where dd is a 1 or 2 digit subscript. For example --
        // X1, Y1, Z1, X2, Y2, Z2, ... so on.
        // Check and ignore these labels.
        std::getline(in_stream, line);
        auto xyz_labels_found = tokenize(line, delimiters_);
        auto num_markers_expected = 
            std::stoul(table_->getMetaData<std::string>(num_markers_label_));
        decltype(xyz_labels_found) xyz_labels_expected{};
        xyz_labels_expected.reserve(num_markers_expected * 3);
        for(int i = 1; i <= num_markers_expected; ++i)
            for(auto& letter : {x_label_, y_label_, z_label_})
                xyz_labels_expected.push_back(letter + std::to_string(i));
        if(xyz_labels_found != xyz_labels_expected)
            throw Exception{"Expected secondary column labels to "
                    "be of form X1, Y1, Z1, X2, Y2, Z2, ... so on. There must "
                    "be one (X,Y,Z) triplet per marker."};
        
        // Read the rows one at a time and fill up the time column container and
        // the data container.
        auto num_frames_expected = 
            std::stoul(table_->getMetaData<std::string>(num_frames_label_));
        std::size_t row_num{data_starts_at_row_ - 1};
        while(std::getline(in_stream, line)) {
            auto row = tokenize(line, delimiters_);
            ++row_num;

            if(row.size() == 0)
                continue;

            if(row.size() != column_labels_.size() * 3 + 2)
                throw Exception{"There are " + 
                        std::to_string(column_labels_.size() * 3 + 2) + 
                        " column labels but row " + std::to_string(row_num) + 
                        " contains " + std::to_string(row.size()) + 
                        " columns."};

            // Columns 2 till the end are data.
            std::vector<SimTK::Vec3> row_vector{};
            auto num_markers_expected = 
               std::stoul(table_->getMetaData<std::string>(num_markers_label_));
            row_vector.reserve(num_markers_expected);
            for(std::size_t c = 2; c < column_labels_.size() * 3 + 2; c += 3)
                row_vector.push_back({std::stod(row[c]),
                                      std::stod(row[c+1]),
                                      std::stod(row[c+2])});
            
            // Column 1 is time.
            table_->addTimeAndRow(std::stod(row[1]), std::move(row_vector));
        }

        // if(table_->getNumRows() != num_frames_expected)
        //     throw Exception{"Expected " + std::to_string(num_frames_expected) + 
        //             " frames but found " + std::to_string(table_->getNumRows()) 
        //             + " frames."};
    }

    static std::string getIdentifier() {
        return "trc";
    }

private:
    Table* table_;

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
