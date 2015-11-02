/* -------------------------------------------------------------------------- *
 *                          OpenSim:  TRCFileAdapter.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
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

#ifndef OPENSIM_TRC_FILE_ADAPTER_H_
#define OPENSIM_TRC_FILE_ADAPTER_H_

#include "FileAdapter.h"

namespace OpenSim {

class TRCFileAdapter : public FileAdapter {
public:
    using Table = TimeSeriesTable_<SimTK::Vec3>;

    TRCFileAdapter()                                 = default;
    TRCFileAdapter(const TRCFileAdapter&)            = default;
    TRCFileAdapter(TRCFileAdapter&&)                 = default;
    TRCFileAdapter& operator=(const TRCFileAdapter&) = default;
    TRCFileAdapter& operator=(TRCFileAdapter&&)      = default;
    ~TRCFileAdapter()                                = default;
    
    TRCFileAdapter* clone() const override;

    std::unique_ptr<Table> read(const std::string& fileName) const;

    void write(const Table& table, 
               const std::string& fileName) const;

protected:
    OutputTables extendRead(const std::string& fileName) const override;

    void extendWrite(const InputTables& tables, 
                     const std::string& fileName) const override;
    
private:
    static const std::string              delimiter_write_;
    static const std::string              delimiters_read_;
    static const std::string              newline_;
    static const std::string              frame_num_column_label_;
    static const std::string              time_column_label_;
    static const std::string              x_label_;
    static const std::string              y_label_;
    static const std::string              z_label_;
    static const std::string              num_markers_label_;
    static const std::string              num_frames_label_;
    static const unsigned                 data_starts_at_row_;
    static const std::vector<std::string> metadata_keys_;
};

} // namespace OpenSim

#endif // OPENSIM_TRC_FILE_ADAPTER_H_
