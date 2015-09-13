/* -------------------------------------------------------------------------- *
 *                            OpenSim:  DataTable.h                           *
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

/** \file
This file defines the  DataTable_ class, which is used by OpenSim to provide an 
in-memory container for data access and manipulation.                         */

#ifndef OPENSIM_COMMON_DATATABLE_H
#define OPENSIM_COMMON_DATATABLE_H

// Non-standard headers.
#include "SimTKcommon.h"
#include "OpenSim/Common/Exception.h"
#include "OpenSim/Common/ValueArrayDictionary.h"

namespace OpenSim {

/** AbstractDataTable is the base-class of all DataTable_(templated) allowing 
storage of DataTable_ templated on different types to be stored in a container 
like std::vector. AbstractDataTable_ offers:
- Interface to access columns of DataTable_ through column labels. 
- A heterogeneous container to store metadata associated with the DataTable_ in
  the form of key-value pairs where key is of type std::string and value can be
  of any type.

This class is abstract and cannot be used directly. Create instances of 
DataTable_ instead. See DataTable_ for details on ussage.                     */
class AbstractDataTable {
protected:
    class ValueDictionary {
    public:
        using AbstractValue = SimTK::AbstractValue;

        const AbstractValue& getValueForKey(const std::string& key) const {
            return _dictionary.getValueArrayForKey(key)[0];
        }

        template<typename ValueType>
        void setValueForKey(const std::string& key,
                            const ValueType& value) {
            ValueArray<ValueType> value_array{};
            value_array.upd().push_back(value);
            _dictionary.setValueArrayForKey(key, value_array);
        }

        void removeValueForKey(const std::string& key) {
            _dictionary.removeValueArrayForKey(key);
        }

        std::vector<std::string> getKeys() const {
            return _dictionary.getKeys();
        }

    private:
        ValueArrayDictionary _dictionary;
    }; // ValueDictionary

public:
    using TableMetaData       = ValueDictionary;
    using DependentsMetaData  = ValueArrayDictionary;
    using IndependentMetaData = ValueDictionary;

    AbstractDataTable()                                      = default;
    AbstractDataTable(const AbstractDataTable&)              = default;
    AbstractDataTable(AbstractDataTable&&)                   = default;
    AbstractDataTable& operator=(const AbstractDataTable&)   = default;
    AbstractDataTable& operator=(AbstractDataTable&&)        = default;
    virtual std::unique_ptr<AbstractDataTable> clone() const = 0;
    virtual ~AbstractDataTable() {}

    const TableMetaData& getTableMetaData() const {
        return _tableMetaData;
    }

    TableMetaData& updTableMetaData() {
        return _tableMetaData;
    }

    size_t getNumRows() const {
        return computeNumRows();
    }

    size_t getNumColumns() const {
        return computeNumColumns();
    }

    const IndependentMetaData& getIndependentMetaData() const {
        return _independentMetaData;
    }
    
    void 
    setIndependentMetaData(const IndependentMetaData& independentMetaData) {
        _independentMetaData = independentMetaData;
        validateIndependentMetaData();
    }

    const DependentsMetaData& getDependentsMetaData() const {
        return _dependentsMetaData;
    }
    
    void 
    setDependentsMetaData(const DependentsMetaData& dependentsMetaData) {
        _dependentsMetaData = dependentsMetaData;
        validateDependentsMetaData();
    }

protected:
    virtual size_t computeNumRows() const = 0;
    virtual size_t computeNumColumns() const = 0;
    virtual void validateIndependentMetaData() const = 0;
    virtual void validateDependentsMetaData() const = 0;

    TableMetaData       _tableMetaData;
    DependentsMetaData  _dependentsMetaData;
    IndependentMetaData _independentMetaData;
}; // AbstractDataTable




/** DataTable_ is a in-memory storage container for data (in the form of a 
matrix with column names) with support for holding metadata.                
                                                                              
- Underlying matrix will have entries of configurable type ET (template 
  param).
- Random-access (constant-time) to specific entries, entire columns and entire 
  rows using their index.
- Average constant-time access to columns through column-labels.
- Add rows and columns to existing DataTable_. 
- Add/concatenate two DataTable_(s) by row and by column. 
- %Set column labels for a subset of columns, update them, remove them etc. 
- Construct DataTable_ emtpy OR with a given shape and default value OR using 
  and iterator pair one entry at a time.
- Heterogeneous metadata container through base class AbstractDataTable. 
  Metadata in the form of key-value pairs where key is a std::string and value 
  is is of any type.
                                                                              
\tparam ET Type of the entries in the underlying matrix. Defaults to         
           SimTK::Real (alias for double).                                    */
template<typename ETX = double, typename ETY = SimTK::Real>
class DataTable_ : public AbstractDataTable {
public:
    using RowVector    = SimTK::RowVector_<ETY>;
    using ColumnVector = SimTK::Vector_<ETY>;

    void appendRow(const ETX& indRow, const RowVector& depRow) {
        _indData.push_back(indRow);
        _depData.push_back(depRow);
        validateAppendRow();
    }

    RowVector getRowAtIndex(size_t index) const {
        return _depData.getRow(index);
    }

    RowVector getRow(const ETX& ind) const {
        auto row_ind = std::distance(_indData.begin(), 
                                     std::find(_indData.cbegin(), 
                                               _indData.cend(), 
                                               ind));
        return _depData.getRow(row_ind);
    }

    RowVector updRowAtIndex(size_t index) {
        return _depData.updRow(index);
    }

    RowVector updRow(const ETX& ind) {
        auto row_ind = std::distance(_indData.begin(), 
                                     std::find(_indData.cbegin(), 
                                               _indData.cend(), 
                                               ind));
        return _depData.updRow(row_ind);
    }

    const std::vector<ETX>& getIndependentColumn() const {
        return _indData;
    }

    ColumnVector getDependentColumnAtIndex(size_t index) const {
        return _depData.getCol(index);
    }

protected:
    size_t computeNumRows() const override {
        return _depData.nrow();
    }

    size_t computeNumColumns() const override {
        return _depData.ncol();
    }

    void validateIndependentMetaData() const override {
    }

    void validateDependentsMetaData() const override {
    }

    virtual void validateAppendRow() const = 0;

    std::vector<ETX>    _indData;
    SimTK::Matrix_<ETY> _depData;
};  // DataTable_

/** See DataTable_ for details on the interface.                              */
using DataTable = DataTable_<SimTK::Real>;

} // namespace OpenSim

#endif //OPENSIM_COMMON_DATATABLE_H
