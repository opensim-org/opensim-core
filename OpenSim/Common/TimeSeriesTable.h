/* -------------------------------------------------------------------------- *
 *                            OpenSim:  TimeSeriesTable.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <SimTKsimbody.h>
#include "Logger.h"

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
#ifndef SWIG
    using InvalidRow::InvalidRow;
#endif
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

class TimeOutOfRange : public Exception {
public:
    TimeOutOfRange(const std::string& file,
                   size_t line,
                   const std::string& func,
                   const double time,
                   const double min,
                   const double max) :
        Exception(file, line, func) {
        std::string msg = "Time " + std::to_string(time) + 
            " is out of time range [" + std::to_string(min) +
            ", " + std::to_string(max) + "]";

        addMessage(msg);
    }
};

class InvalidTimeRange : public Exception {
public:
    InvalidTimeRange(const std::string& file,
                     size_t line,
                     const std::string& func,
                     const double begTime,
                     const double endTime) :
        Exception(file, line, func) {
        std::string msg = " Invalid time range: initial time " + 
            std::to_string(begTime)  + " >= final time = " +
            std::to_string(endTime);

        addMessage(msg);
    }
};

/** TimeSeriesTable_ is a DataTable_ where the independent column is time of 
type double. The time column is enforced to be strictly increasing.           */
template<typename ETY = SimTK::Real>
class TimeSeriesTable_ : public DataTable_<double, ETY> {
public:
    typedef SimTK::RowVector_<ETY>     RowVector;
    typedef SimTK::RowVectorView_<ETY> RowVectorView;

    TimeSeriesTable_()                                   = default;
    TimeSeriesTable_(const TimeSeriesTable_&)            = default;
    TimeSeriesTable_(TimeSeriesTable_&&)                 = default;
    TimeSeriesTable_& operator=(const TimeSeriesTable_&) = default;
    TimeSeriesTable_& operator=(TimeSeriesTable_&&)      = default;
    ~TimeSeriesTable_()                                  = default;

    /** Convenience constructor to efficiently populate a time series table
    from available data. This is primarily useful for constructing with large
    data read in from file without having to reallocate and copy memory.*/
    TimeSeriesTable_(const std::vector<double>& indVec,
        const SimTK::Matrix_<ETY>& depData,
        const std::vector<std::string>& labels) : 
            DataTable_<double, ETY>(indVec, depData, labels) {
        try {
            // Perform the validation of the data of this TimeSeriesTable.
            // validateDependentsMetaData() invoked by the DataTable_
            // constructor via setColumnLabels(), but we invoke it again
            // because base classes cannot properly invoke virtual functions.
            this->validateDependentsMetaData();
            for (size_t i = 0; i < indVec.size(); ++i) {
                this->validateRow(i, indVec[i], depData.row(int(i)));
            }
        }
        catch (std::exception&) {
            // wipe out the data loaded if any
            this->_indData.clear();
            this->_depData.clear();
            this->removeDependentsMetaDataForKey("labels");
            throw;
        }
    }

    /** Construct a table with only the independent (time) column and 0
    dependent columns. This constructor is useful if you want to populate the
    table by appending columns rather than by appending rows.                 */
    TimeSeriesTable_(const std::vector<double>& indVec) :
            DataTable_<double, ETY>(indVec) {
        try {
            // Perform the validation of the data of this TimeSeriesTable.
            // validateDependentsMetaData() invoked by the DataTable_
            // constructor via setColumnLabels(), but we invoke it again
            // because base classes cannot properly invoke virtual functions.
            this->validateDependentsMetaData();
            for (size_t i = 0; i < indVec.size(); ++i) {
                this->validateRow(i, indVec[i], this->_depData.row(int(i)));
            }
        }
        catch (std::exception&) {
            // wipe out the data loaded if any
            this->_indData.clear();
            this->_depData.clear(); // should be empty
            this->removeDependentsMetaDataForKey("labels"); // should be empty
            throw;
        }

    }

#ifndef SWIG
    using DataTable_<double, ETY>::DataTable_;
    using DataTable_<double, ETY>::operator=;
    /** Flatten the columns of this table to create a TimeSeriesTable_<double>.
    See documentation of DataTable_::flatten() for details.                   */
    using DataTable_<double, ETY>::flatten;
    /** Pack the columns of this table (which should be TimeSeriesTable_<double>
    ) to create a TimeSeriesTable_<SimTK::Vec3>, TimeSeriesTable_<SimTK::Vec6>,
    TimeSeriesTable_<SimTK::UnitVec3> and so on. See documentation for 
    DataTable_::pack().                                                       */
    using DataTable_<double, ETY>::pack;
#endif    
    
    /** Construct a TimeSeriesTable_ from a DataTable_.                       

    \throws InvalidTable If the input table's independent column is not strictly
                         increasing.                                          */
    TimeSeriesTable_(const DataTable_<double, ETY>& datatable) : 
        DataTable_<double, ETY>(datatable) {
        using DT = DataTable_<double, ETY>;

        OPENSIM_THROW_IF(!std::is_sorted(DT::_indData.cbegin(), 
                                         DT::_indData.cend()) ||
                         std::adjacent_find(DT::_indData.cbegin(), 
                                            DT::_indData.cend()) != 
                         DT::_indData.cend(),
                         TimeColumnNotIncreasing);
    }

    /** Construct TimeSeriesTable_ from a file.

    \param filename Name of the file.

    \throws InvalidArgument If the input file contains more than one table.
    \throws InvalidArgument If the input file contains a table that is not of
                            this TimeSeriesTable_ type.                       */
    TimeSeriesTable_(const std::string& filename) : 
        TimeSeriesTable_{filename, ""} {}

    /** Construct TimeSeriesTable_ from a file.

    \param filename Name of the file.
    \param tablename Name of the table in the file to construct this 
                     TimeSeriesTable_ from. For example, a c3d file contains 
                     tables named 'markers' and 'forces'.

    \throws InvalidArgument If the input file contains more than one table and 
                            tablename was not specified.
    \throws InvalidArgument If the input file contains a table that is not of
                            this TimeSeriesTable_ type.                       */
    TimeSeriesTable_(const std::string& filename, 
                     const std::string& tablename) {
        auto absTables = FileAdapter::createAdapterFromExtension(filename)->read(filename);

        OPENSIM_THROW_IF(absTables.size() > 1 && tablename.empty(),
                         InvalidArgument,
                         "File '" + filename + 
                         "' contains more than one table and tablename not "
                         "specified.");

        AbstractDataTable* absTable{};
        if(tablename.empty()) {
            absTable = (absTables.cbegin()->second).get();
        } else {
            try {
                absTable = absTables.at(tablename).get();
            } catch (const std::out_of_range&) {
                OPENSIM_THROW(InvalidArgument,
                              "File '" + filename + "' contains no table named "
                              "'"+ tablename + "'.");
            }
        }
        auto table = dynamic_cast<TimeSeriesTable_*>(absTable);
        OPENSIM_THROW_IF(table == nullptr,
                         InvalidArgument,
                         "DataTable cannot be created from file '" + filename +
                         "'. Type mismatch.");

        *this = std::move(*table);
    }

    /** Get index of row whose time is nearest/closest to the given value.

    \param time Value to search for.
    \param restrictToTimeRange  When true -- Exception is thrown if the given
                                value is out-of-range of the time column. A value
                                within SimTK::SignifcantReal of a time column
                                bound is considered to be equal to the bound.
                                When false -- If the given value is less than or
                                equal to the first value in the time column, the
                                index returned is of the first row. If the given
                                value is greater than or equal to the last value
                                in the time column, the index of the last row is
                                returned. Defaults to 'true'.

    \throws TimeOutOfRange If the given value is out-of-range of time column.
    \throws EmptyTable If the table is empty.                                 */
    size_t getNearestRowIndexForTime(const double time,
                                     const bool restrictToTimeRange = true) const {
        using DT = DataTable_<double, ETY>;
        const auto& timeCol = DT::getIndependentColumn();
        OPENSIM_THROW_IF(timeCol.size() == 0,
            EmptyTable);
        const SimTK::Real eps = SimTK::SignificantReal;
        OPENSIM_THROW_IF(restrictToTimeRange &&
            ((time < timeCol.front() - eps) ||
            (time > timeCol.back() + eps)),
            TimeOutOfRange,
            time, timeCol.front(), timeCol.back());

        auto iter = std::lower_bound(timeCol.begin(), timeCol.end(), time);
        if (iter == timeCol.end())
            return timeCol.size() - 1;
        if (iter == timeCol.begin())
            return 0;
        if ((*iter - time) <= (time - *std::prev(iter)))
            return std::distance(timeCol.begin(), iter);
        else
            return std::distance(timeCol.begin(), std::prev(iter));
    }
    /** Get index of row whose time is first to be higher than the given value.

     \param time Value to search for.
     */
    size_t getRowIndexAfterTime(
            const double& time) const {
        size_t candidate = getNearestRowIndexForTime(time, false);
        using DT = DataTable_<double, ETY>;
        const auto& timeCol = DT::getIndependentColumn();
        const SimTK::Real eps = SimTK::SignificantReal;
        // increment if less than time passed in
        if (timeCol[candidate] < time-eps) 
            candidate++;
        OPENSIM_THROW_IF(candidate > timeCol.size() - 1,
                    TimeOutOfRange, time, timeCol.front(), timeCol.back());
        return candidate;
    }
    /** Get index of row whose time is the largest time less than the given value.

     \param time Value to search for.
     */
    size_t getRowIndexBeforeTime(const double& time) const {
        size_t candidate = getNearestRowIndexForTime(time, false);
        using DT = DataTable_<double, ETY>;
        const auto& timeCol = DT::getIndependentColumn();
        const SimTK::Real eps = SimTK::SignificantReal;
        // increment if less than time passed in
        if (timeCol[candidate] > time+eps) candidate--;
        OPENSIM_THROW_IF(candidate < 0, TimeOutOfRange, time,
                timeCol.front(), timeCol.back());
        return candidate;
    }

    /** Get row whose time column is nearest/closest to the given value. 

    \param time Value to search for. 
    \param restrictToTimeRange When true -- Exception is thrown if the given 
                               value is out-of-range of the time column. 
                               When false -- If the given value is less than or 
                               equal to the first value in the time column, the
                               row returned is the first row. If the given value
                               is greater than or equal to the last value in the
                               time column, the row returned is the last row. 
                               This operation only returns existing rows and 
                               does not perform any interpolation. Defaults to
                               'true'.

    \throws TimeOutOfRange If the given value is out-of-range of time column.
    \throws EmptyTable If the table is empty.                                 */
    RowVectorView
    getNearestRow(const double& time,
                  const bool restrictToTimeRange = true) const {
        using DT = DataTable_<double, ETY>;
        return DT::getRowAtIndex( 
            getNearestRowIndexForTime(time, restrictToTimeRange) );
    }

    /** Get writable reference to row whose time column is nearest/closest to 
    the given value. 

    \param time Value to search for. 
    \param restrictToTimeRange When true -- Exception is thrown if the given 
                               value is out-of-range of the time column. 
                               When false -- If the given value is less than or 
                               equal to the first value in the time column, the
                               row returned is the first row. If the given value
                               is greater than or equal to the last value in the
                               time column, the row returned is the last row. 
                               This operation only returns existing rows and 
                               does not perform any interpolation. Defaults to
                               'true'.

    \throws TimeOutOfRange If the given value is out-of-range of time column.
    \throws EmptyTable If the table is empty.                                 */
    RowVectorView
    updNearestRow(const double& time,
                  const bool restrictToTimeRange = true) {
        using DT = DataTable_<double, ETY>;
        return DT::updRowAtIndex(
            getNearestRowIndexForTime(time, restrictToTimeRange));
    }

    /** Compute the average row in the time range (inclusive) given. This
    operation does not modify the table. It just computes and returns an average
    row. 

    \throws InvalidTimeRange If beginTime is greater than or equal to endTime.
    \throws TimeOutOfRange If beginTime or endTime is out of range of time 
                           column.                                            */
    RowVector averageRow(const double& beginTime, const double& endTime) const {
        using DT = DataTable_<double, ETY>;
        OPENSIM_THROW_IF(endTime <= beginTime,
                         InvalidTimeRange,
                         beginTime, endTime);
        const auto& timeCol = DT::getIndependentColumn();
        OPENSIM_THROW_IF(beginTime < timeCol.front() ||
                         beginTime > timeCol.back(),
                         TimeOutOfRange,
                         beginTime, timeCol.front(), timeCol.back(),);
        OPENSIM_THROW_IF(endTime < timeCol.front() ||
                         endTime > timeCol.back(),
                         TimeOutOfRange,
                         endTime, timeCol.front(), timeCol.back());

        std::vector<double> comps(DT::numComponentsPerElement(), 0);
        RowVector row{static_cast<int>(DT::getNumColumns()),
                      DT::makeElement(comps.begin(), comps.end())};
        unsigned numRowsInRange{};
        for(unsigned r = 0; r < DT::getNumRows(); ++r) {
            if(timeCol[r] >= beginTime && timeCol[r] <= endTime) {
                row += DT::getRowAtIndex(r);
                ++numRowsInRange;
            }
        }
        row /= numRowsInRange;

        return row;
    }
    /**
     * Trim TimeSeriesTable to rows that have times that lies between 
     * newStartTime, newFinalTime. The trimming is done in place, no copy is made. 
     * Uses getRowIndexAfterTime to locate first row and
     * getNearestRowIndexForTime method to locate last row.
     */
    void trim(const double& newStartTime, const double& newFinalTime) {
        OPENSIM_THROW_IF(newFinalTime < newStartTime, EmptyTable);
        const auto& timeCol = this->getIndependentColumn();
        size_t start_index = 0;
        size_t last_index = this->getNumRows() - 1;
        // Avoid throwing exception if newStartTime is less than first time
        // or newFinalTime is greater than last value in table
        start_index = this->getRowIndexAfterTime(newStartTime);
        last_index = this->getRowIndexBeforeTime(newFinalTime);
        // Make sure last_index >= start_index before attempting to trim
        OPENSIM_THROW_IF(last_index < start_index, EmptyTable);
        // do the actual trimming based on index instead of time.
        trimToIndices(start_index, last_index);
        // If resulting table is empty, throw
        if (this->getNumRows()==0)
            log_warn("Trimming resulted in an empty table.");
    }
    /**
     * trim TimeSeriesTable, keeping rows at newStartTime to the end.
     */
    void trimFrom(const double& newStartTime) { 
        this->trim(newStartTime, this->getIndependentColumn().back());
    }
    /**
     * trim TimeSeriesTable, keeping rows up to newFinalTime
     */
    void trimTo(const double& newFinalTime) {
        this->trim(this->getIndependentColumn().front(), newFinalTime);
    }
protected:
    /** Validate the given row. 

    \throws InvalidRow If the timestamp for the row breaks strictly increasing
                       property of the independent column.                    */
    void validateRow(size_t rowIndex,
                     const double& time, 
                     const RowVector& row) const override {
        using DT = DataTable_<double, ETY>;

        if(DT::_indData.empty())
            return;

        if(rowIndex > 0) {
            OPENSIM_THROW_IF(DT::_indData[rowIndex - 1] >= time,
                             TimestampLessThanEqualToPrevious, rowIndex, time, 
                             DT::_indData[rowIndex - 1]);
        }

        if(rowIndex < DT::_indData.size() - 1) {
            OPENSIM_THROW_IF(DT::_indData[rowIndex + 1] <= time,
                             TimestampGreaterThanEqualToNext, rowIndex, time, 
                             DT::_indData[rowIndex + 1]);
        }
    }
    /** trim table to rows between start_index and last_index inclusively
     */
    void trimToIndices(const size_t& start_index, const size_t& last_index) {
        // This uses the rather invasive but efficient mechanism to copy a
        // block of the underlying Matrix.
        // Side effect may include that headers/metaData may be left stale.
        // Alternatively we can create a new TimeSeriesTable and copy contents
        // one row at a time but that's rather overkill
        SimTK::Matrix_<ETY> matrixBlock = this->updMatrix()((int)start_index, 0,
                (int)(last_index - start_index + 1),
                (int)this->getNumColumns());
        this->updMatrix() = matrixBlock;
        std::vector<double> newIndependentVector = std::vector<double>(
                this->getIndependentColumn().begin() + start_index,
                this->getIndependentColumn().begin() + last_index + 1);
        this->_indData = newIndependentVector;
    }

    friend class TableUtilities;

}; // TimeSeriesTable_

/** See TimeSeriesTable_ for details on the interface.                        */
typedef TimeSeriesTable_<SimTK::Real> TimeSeriesTable;

/** See TimeSeriesTable_ for details on the interface.                        */
typedef TimeSeriesTable_<SimTK::Vec3> TimeSeriesTableVec3;

/** See TimeSeriesTable_ for details on the interface.                        */
typedef TimeSeriesTable_<SimTK::Quaternion> TimeSeriesTableQuaternion;

/** See TimeSeriesTable_ for details on the interface.                        */
typedef TimeSeriesTable_<SimTK::Rotation> TimeSeriesTableRotation;

} // namespace OpenSim

#endif // OPENSIM_TIME_SERIES_DATA_TABLE_H_
