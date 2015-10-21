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
 * Authors:                                                                   *
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

class InvalidTable : public Exception {
public:
    using Exception::Exception;
};

class TimeColumnNotIncreasing : public InvalidTable {
public:
    TimeColumnNotIncreasing(const std::string& file, 
                            size_t line,
                            const std::string& func) :
        InvalidTable(file, line, func) {
        std::string msg = "Time column is not strictly increasing";

        addMessage(msg);
    } 
};

class InvalidTimestamp : public InvalidRow {
public:
    using InvalidRow::InvalidRow;
};

class TimestampLessThanEqualToPrevious : public InvalidTimestamp {
public:
    TimestampLessThanEqualToPrevious(const std::string& file,
                                     size_t line,
                                     const std::string& func,
                                     size_t rowIndex,
                                     double new_timestamp,
                                     double prev_timestamp) :
        InvalidTimestamp(file, line, func) {
        std::string msg = "Timestamp at row " + std::to_string(rowIndex);
        msg += " with value " + std::to_string(new_timestamp);
        msg += " is less-than/equal to timestamp at row ";
        msg += std::to_string(rowIndex - 1) + " with value ";
        msg += std::to_string(prev_timestamp);

        addMessage(msg);
    }
};

class TimestampGreaterThanEqualToNext : public InvalidTimestamp {
public:
    TimestampGreaterThanEqualToNext(const std::string& file,
                                    size_t line,
                                    const std::string& func,
                                    size_t rowIndex,
                                    double new_timestamp,
                                    double next_timestamp) :
        InvalidTimestamp(file, line, func) {
        std::string msg = "Timestamp at row " + std::to_string(rowIndex);
        msg += " with value " + std::to_string(new_timestamp);
        msg += " is greater-than/equal to timestamp at row "; 
        msg += std::to_string(rowIndex + 1) + " with value "; 
        msg += std::to_string(next_timestamp);

        addMessage(msg);
    }
};

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
    
    /** Construct a TimeSeriesTable_ from a DataTable_.                       

    \throws InvalidTable If the input table's independent column is not strictly
                         increasing.                                          */
    TimeSeriesTable_(const DataTable_<double, ETY>& datatable) : 
        DataTable_<double, ETY>(datatable) {
        using DT = DataTable_<double, ETY>;

        if(!std::is_sorted(DT::_indData.cbegin(), DT::_indData.cend()) ||
           std::adjacent_find(DT::_indData.cbegin(), DT::_indData.cend()) != 
           DT::_indData.cend()) {
            throw TimeColumnNotIncreasing{__FILE__, __LINE__, __func__};
        }
    }

protected:
    /** Validate the given row. 

    \throws InvalidRow If the timestamp for the row breaks strictly increasing
                       property of the indpedent column.                      */
    void validateRow(size_t rowIndex,
                     const double& time, 
                     const RowVector& row) const override {
        using DT = DataTable_<double, ETY>;

        if(DT::_indData.empty())
            return;

        if(rowIndex > 0) {
            if(DT::_indData[rowIndex - 1] >= time)
                throw TimestampLessThanEqualToPrevious{__FILE__, __LINE__, 
                        __func__, rowIndex, time, DT::_indData[rowIndex - 1]};
        }

        if(rowIndex < DT::_indData.size() - 1) {
            if(DT::_indData[rowIndex + 1] <= time)
                throw TimestampGreaterThanEqualToNext{__FILE__, __LINE__, 
                        __func__, rowIndex, time, DT::_indData[rowIndex + 1]};
        }
    }
}; // TimeSeriesTable_

/** See TimeSeriesTable_ for details on the interface.                        */
using TimeSeriesTable = TimeSeriesTable_<SimTK::Real>;

} // namespace OpenSim

#endif // OPENSIM_TIME_SERIES_DATA_TABLE_H_
