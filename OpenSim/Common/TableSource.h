/* -------------------------------------------------------------------------- *
 *                            OpenSim:  TableSource.h                         *
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

#ifndef OPENSIM_TABLE_SOURCE_H_
#define OPENSIM_TABLE_SOURCE_H_

#include "TimeSeriesTable.h"
#include "Component.h"

namespace OpenSim {

/** Component representing a source of data from a TimeSeriesTable_.

This Component has two outputs:
- A list output with one channel per column of the TimeSeriesTable_.
- A non-list output for a row of the TimeSeriesTable_.

Construct this Component by giving it a TimeSeriesTable_. Then use it by 
connecting its output to the Input of another Component that accepts compatible
type of Input. Make sure to populate the column-labels of the TimeSeriesTable_ 
before connecting this Component to the input of another Component.    

\tparam ET Type of each element of the TimeSeriesTable_ this Component 
           represents.                                                        */
template<typename ET>
class TableSource_ : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(TableSource_, ET, Component);

public:
    OpenSim_DECLARE_PROPERTY(filename, std::string,
                             "Path to the file to populate the TableSource_ "
                             "with. The path is relative to the working "
                             "directory, not relative to the directory "
                             "containing the model file.");
    OpenSim_DECLARE_PROPERTY(tablename, std::string,
                             "Name of the table in the file to populate the "
                             "TableSource with. Ex. 'markers', 'forces'.");

    /** Type of the TimeSeriesTable_ this Component will hold.                */
    typedef TimeSeriesTable_<ET> Table;
    /** Type of the 'row' Output of this Component.                           */
    typedef SimTK::Vector_<ET>   Vector;

    OpenSim_DECLARE_OUTPUT(all_columns, Vector, getRowAtTime, 
                           SimTK::Stage::Time);
    OpenSim_DECLARE_LIST_OUTPUT(column, ET, getColumnAtTime, 
                                SimTK::Stage::Time);

    TableSource_() {
        constructProperties();
    }

    TableSource_(const TableSource_&)            = default;
    TableSource_(TableSource_&&)                 = default;
    TableSource_& operator=(const TableSource_&) = default;
    TableSource_& operator=(TableSource_&&)      = default;
    
    /** Construct the TableSource_ by giving it a TimeSeriesTable_ to hold.   */
    TableSource_(const Table& table) :
        _table{table} {
        constructProperties();
    }

    /** Construct the TableSource_ from a file.

    \param filename Name of the file.

    \throws KeyNotFound If table provided does not have column-labels.        */
    TableSource_(const std::string& filename) :
        TableSource_{filename, ""}{}

    /** Construct the TableSource_ from a file.

    \param filename Name of the file.
    \param tablename Name of the table in the file to populate the TableSource
                     with. Ex. 'markers', 'forces'.

    \throws KeyNotFound If table provided does not have column-labels.        */
    TableSource_(const std::string& filename,
                 const std::string& tablename) {
        constructProperties();
        setTable(filename, tablename);
    }

    /// \name Get/Set underlying TimeSeriesTable_
    /// @{

    /** Get a read-only reference to the TimeSeriesTable_ this TableSource_ 
    currently holds.                                                          */
    const Table& getTable() const {
        return _table;
    }

    /** Replace the existing TimeSeriesTable_ that this TableSource_ currently 
    holds. The properties 'filename' and 'tablename' are reset to empty strings
    as a result of this operation.

    \throws KeyNotFound If table provided does not have column labels.        */
    void setTable(const Table& table) {
        setTable_impl(table);
        set_filename("");
        set_tablename("");
    }

    /** Replace the TimeSeriesTable_ that this TableSource_ currently holds. 
    Property 'filename' is reset to the value provided. Property 'tablename' is
    reset to the empty string as a result of this operation.

    \throws InvalidCall If property `filename` is set. This call is not allowed
                        if `filename` property is set.                        
    \throws KeyNotFound If table provided does not have column labels.        */
    void setTable(const std::string& filename) {
        setTable_impl(TimeSeriesTable_<ET>{filename});
        set_filename(filename);
        set_tablename("");
    }

    /** Replace the TimeSeriesTable_ that this TableSource_ currently holds. 
    Properties 'filename' and 'tablename' are reset to the values provided.

    \param filename Name of the file.
    \param tablename Name of the table in the file to construct the 
                     TimeSeriesTable_ from. For example, a c3d file contains 
                     tables named 'markers' and 'forces'.

    \throws InvalidCall If property `filename` is set. This call is not allowed
                        if `filename` property is set.                        
    \throws KeyNotFound If table provided does not have column labels.        */
    void setTable(const std::string& filename,
                  const std::string& tablename) {
        setTable_impl(TimeSeriesTable_<ET>{filename, tablename});
        set_filename(filename);
        set_tablename(tablename);
    }

    /// @}

protected:
    void setTable_impl(const Table& table) {
        _table = table;
        auto& columnOutput = updOutput("column");
        columnOutput.clearChannels();
        for(const auto& columnLabel : _table.getColumnLabels())
            columnOutput.addChannel(columnLabel);
    }

    /** Retrieve value of a column at a given time(implicit in the State 
    provided). Linear interpolation is performed if the TimeSeriesTable_ does
    not contain an entry for the time mentioned by the state.

    \throws EmptyTable If the TimeSeriesTable_ this TableSource_ holds is 
                       currently empty.
    \throws TimeOutOfRange If the time specified by the State is either less 
                           than the smallest timestamp or greater than the 
                           largest timestamp in the TimeSeriesTable_.
    \throws KeyNotFound If TimeSeriesTable_ does not have column-labels.      */
    ET getColumnAtTime(const SimTK::State& state, 
                       const std::string& columnLabel) const {
        OPENSIM_THROW_IF(_table.getNumRows() == 0, EmptyTable);
        const auto& timeCol = _table.getIndependentColumn();
        const auto time = state.getTime();
        OPENSIM_THROW_IF(time < timeCol.front() ||
                         time > timeCol.back(),
                         TimeOutOfRange, 
                         time, timeCol.front(), timeCol.back());

        const auto colInd = 
            static_cast<int>(_table.getColumnIndex(columnLabel));
        auto lb = std::lower_bound(timeCol.begin(), timeCol.end(), time);
        if(lb == timeCol.begin())
            return _table.getMatrix().getElt(0, colInd);
        else if(lb == timeCol.end())
            return _table.
                   getMatrix().
                   getElt(static_cast<int>(timeCol.size() - 1), colInd);
        else if(*lb == time)
            return _table.
                   getMatrix().
                   getElt(static_cast<int>(lb - timeCol.begin()), colInd);
        else {
            auto prevTime = *(lb - 1);
            auto nextTime = *lb;
            auto prevElt = _table.
                           getMatrix().
                           getElt(static_cast<int>(lb - 1 - timeCol.begin()),
                                  colInd);
            auto nextElt = _table.
                           getMatrix().
                           getElt(static_cast<int>(lb - timeCol.begin()), 
                                  colInd);
            auto elt = ((time - prevTime) / (nextTime - prevTime)) * 
                       (nextElt - prevElt) + prevElt;
            return elt;
        }
    }

    /** Retrieve a row of the TimeSeriesTable_ at a given time (specified by the
    state). Linear interpolation is performed if the TimeSeriesTable_ does not
    have an entry for the time mentioned by the state.

    \throws EmptyTable If the TimeSeriesTable_ this TableSource_ holds is 
                       currently empty.
    \throws TimeOutOfRange If the time specified by the State is either less 
                           than the smallest timestamp or greater than the 
                           largest timestamp in the TimeSeriesTable_.         */
    Vector getRowAtTime(const SimTK::State& state) const {
        OPENSIM_THROW_IF(_table.getNumRows() == 0, EmptyTable);
        const auto& timeCol = _table.getIndependentColumn();
        const auto time = state.getTime();
        OPENSIM_THROW_IF(time < timeCol.front() ||
                         time > timeCol.back(),
                         TimeOutOfRange, 
                         time, timeCol.front(), timeCol.back());

        auto lb = std::lower_bound(timeCol.begin(), timeCol.end(), time);
        if(lb == timeCol.begin())
            return _table.getRowAtIndex(0).getAsVector();
        else if(lb == timeCol.end())
            return _table.getRowAtIndex(timeCol.size() - 1).getAsVector();
        else if(*lb == time)
            return _table.getRowAtIndex(lb - timeCol.begin()).getAsVector();
        else {
            auto prevTime = *(lb - 1);
            auto nextTime = *lb;
            auto prevRow = _table.getRowAtIndex(lb - 1 - timeCol.begin());
            auto nextRow = _table.getRowAtIndex(lb - timeCol.begin());
            auto row = ((time - prevTime) / (nextTime - prevTime)) * 
                       (nextRow - prevRow) + prevRow;
            return row.getAsVector();
        }
    }

private:
    void constructProperties() {
        constructProperty_filename("");
        constructProperty_tablename("");
    }

    void extendFinalizeFromProperties() override {
        Super::extendFinalizeFromProperties();

        if(!get_filename().empty())
            setTable_impl(TimeSeriesTable_<ET>{get_filename(),
                                               get_tablename()});

        auto& columnOutput = updOutput("column");
        for(const auto& columnLabel : _table.getColumnLabels())
            columnOutput.addChannel(columnLabel);
    }

    Table _table;
}; // class TableSource_


/** This TableSource_ can hold a TimeSeriesTable_<SimTK::Real> and so its 
list-output 'column' will have channels of type SimTK::Real (double). Its other
output 'row' will be of type SimTK::Vector_<SimTK::Real>.                     */
typedef TableSource_<SimTK::Real> TableSource;

/** This TableSource_ can hold a TimeSeriesTable_<SimTK::Vec3> and so its 
list-output 'column' will have channels of type SimTK::Vec3. Its other output
'row' will be of type SimTK::Vector_<SimTK::Vec3>.                            */
typedef TableSource_<SimTK::Vec3> TableSourceVec3;

} // namespace Opensim

#endif // OPENSIM_TABLE_SOURCE_H_
