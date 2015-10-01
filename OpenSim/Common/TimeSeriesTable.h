/* -------------------------------------------------------------------------- *
 *                            OpenSim:  TimeSeriesTable.h                     *
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

#ifndef OPENSIM_TIME_SERIES_DATA_TABLE_H_
#define OPENSIM_TIME_SERIES_DATA_TABLE_H_

/** \file
This file defines the TimeSeriesTable_ class, which is used by OpenSim to 
provide an in-memory container for data access and manipulation.              */

#include "OpenSim/Common/DataTable.h"


namespace OpenSim {

/** TimeSeriesTable_ is a DataTable_ where the independent column is time of 
type double. The time column is enforced to be strictly increasing.           */
template<typename ETY = SimTK::Real>
class TimeSeriesTable_ : public DataTable_<double, ETY> {
public:
    using RowVector = SimTK::RowVector_<ETY>;

    TimeSeriesTable_()                                   = default;
    TimeSeriesTable_(const TimeSeriesTable_&)            = default;
    TimeSeriesTable_(TimeSeriesTable_&&)                 = default;
    TimeSeriesTable_& operator=(const TimeSeriesTable_&) = default;
    TimeSeriesTable_& operator=(TimeSeriesTable_&&)      = default;
    ~TimeSeriesTable_()                                  = default;
    
    /** Construct a TimeSeriesTable_ from a DataTable_.                       */
    TimeSeriesTable_(const DataTable_<double, ETY>& datatable) : 
        DataTable_<double, ETY>{datatable} {
        using DT = DataTable_<double, ETY>;

        if(!std::is_sorted(DT::_indData.cbegin(), DT::_indData.cend()) ||
           std::adjacent_find(DT::_indData.cbegin(), DT::_indData.cend()) != 
           DT::_indData.cend()) {
            throw Exception{"Independent column is not strictly increasing."};
        }
    }

protected:
    void validateRow(size_t rowIndex,
                     const double& time, 
                     const RowVector& row) const override {
        using DT = DataTable_<double, ETY>;

        if(rowIndex > 0) {
            if(DT::_indData[rowIndex - 1] >= time)
                throw Exception{"Timestamp added for row " + 
                        std::to_string(rowIndex) + " is less than or equal to "
                        "the timestamp for row " + 
                        std::to_string(rowIndex - 1)};
        }

        if(rowIndex < DT::_indData.size() - 1) {
            if(DT::_indData[rowIndex + 1] <= time)
                throw Exception{"Timestamp added for row " +
                        std::to_string(rowIndex) + " is greater than or equal "
                        "to the timestamp for row " + 
                        std::to_string(rowIndex + 1)};
        }
    }
}; // TimeSeriesTable_

/** See TimeSeriesTable_ for details on the interface.                        */
using TimeSeriesTable = TimeSeriesTable_<SimTK::Real>;

} // namespace OpenSim

#endif // OPENSIM_TIME_SERIES_DATA_TABLE_H_
