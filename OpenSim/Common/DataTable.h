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

#ifndef OPENSIM_DATA_TABLE_H_
#define OPENSIM_DATA_TABLE_H_

/** \file
This file defines the  DataTable_ class, which is used by OpenSim to provide an 
in-memory container for data access and manipulation.                         */

// Non-standard headers.
#include "SimTKcommon.h"
#include "OpenSim/Common/Exception.h"
#include "OpenSim/Common/ValueArrayDictionary.h"

namespace OpenSim {

/** AbstractDataTable is the base-class of all DataTable_(templated) allowing 
storage of DataTable_ templated on different types to be stored in a container 
like std::vector. AbstractDataTable offers:
- Interface to access column metadata of the DataTable_.
- A heterogeneous container to store metadata associated with the DataTable_ in
  the form of key-value pairs where key is of type std::string and value can be
  of any type.

This class is abstract and cannot be used directly. Create instances of 
DataTable_ instead. See DataTable_ for details on usage.                     */
class AbstractDataTable {
public:
    using TableMetaData       = ValueArrayDictionary;
    using DependentsMetaData  = ValueArrayDictionary;
    using IndependentMetaData = ValueArrayDictionary;

    AbstractDataTable()                                      = default;
    AbstractDataTable(const AbstractDataTable&)              = default;
    AbstractDataTable(AbstractDataTable&&)                   = default;
    AbstractDataTable& operator=(const AbstractDataTable&)   = default;
    AbstractDataTable& operator=(AbstractDataTable&&)        = default;
    virtual std::unique_ptr<AbstractDataTable> clone() const = 0;
    virtual ~AbstractDataTable()                             = default;

    /** Get metadata associated with the table.                               */
    const TableMetaData& getTableMetaData() const {
        return _tableMetaData;
    }

    /** Update metadata associated with the table.                            */
    TableMetaData& updTableMetaData() {
        return _tableMetaData;
    }

    /** Get number of rows.                                                   */
    size_t getNumRows() const {
        return implementGetNumRows();
    }

    /** Get number of dependent columns.                                      */
    size_t getNumColumns() const {
        return implementGetNumColumns();
    }
    
    /** Get metadata associated with the independent column.                  */
    const IndependentMetaData& getIndependentMetaData() const {
        return _independentMetaData;
    }
    
    /** Set metadata associated with the independent column.                  */
    void 
    setIndependentMetaData(const IndependentMetaData& independentMetaData) {
        _independentMetaData = independentMetaData;
        validateIndependentMetaData();
    }

    /** Get metadata associated with the dependent columns.                   */
    const DependentsMetaData& getDependentsMetaData() const {
        return _dependentsMetaData;
    }
    
    /** Set metadata associated with the dependent columns.                   */
    void 
    setDependentsMetaData(const DependentsMetaData& dependentsMetaData) {
        _dependentsMetaData = dependentsMetaData;
        validateDependentsMetaData();
    }

protected:
    /** Get number of rows. Implemented by derived classes.                   */
    virtual size_t implementGetNumRows() const       = 0;
    
    /** Get number of columns. Implemented by derived classes.                */
    virtual size_t implementGetNumColumns() const    = 0;
    
    /** Check if metadata for independent column is valid . Implemented by 
    derived classes.                                                          */
    virtual void validateIndependentMetaData() const = 0;

    /** Check if metadata for dependent column is valid. Implemented by derived
    classes.                                                                  */
    virtual void validateDependentsMetaData() const  = 0;

    TableMetaData       _tableMetaData;
    DependentsMetaData  _dependentsMetaData;
    IndependentMetaData _independentMetaData;
}; // AbstractDataTable


/** DataTable_ is a in-memory storage container for data with support for 
holding metadata (using the base class AbstractDataTable). Data contains an 
independent column and a set of dependent columns. The type of the independent 
column can be configured using ETX (template param). The type of the dependent 
columns, which together form a matrix, can be configured using ETY (template 
param). Independent and Dependent columns can contain metadata. DataTable_ as a 
whole can contain metadata.                                                   */
template<typename ETX = double, typename ETY = SimTK::Real>
class DataTable_ : public AbstractDataTable {
public:
    using RowVector     = SimTK::RowVector_<ETY>;
    using RowVectorView = SimTK::RowVectorView_<ETY>;
    using VectorView    = SimTK::VectorView_<ETY>;

    DataTable_()                             = default;
    DataTable_(const DataTable_&)            = default;
    DataTable_(DataTable_&&)                 = default;
    DataTable_& operator=(const DataTable_&) = default;
    DataTable_& operator=(DataTable_&&)      = default;
    ~DataTable_()                            = default;

    std::unique_ptr<AbstractDataTable> clone() const override {
        return std::unique_ptr<AbstractDataTable>{new DataTable_{*this}};
    }

    /** Append row to the DataTable_.                                         */
    void appendRow(const ETX& indRow, const RowVector& depRow) {
        validateRow(_indData.size(), indRow, depRow);

        _indData.push_back(indRow);

        if(_depData.nrow() == 0 || _depData.ncol() == 0) {
            try {
                auto& labels = 
                    _dependentsMetaData.getValueArrayForKey("labels");
                if(depRow.size() != labels.size())
                    throw Exception{"Number of columns in the input row does "
                            "not match the number of labels."};
            } catch(std::out_of_range&) {
                // No operation.
            }
            _depData.resize(1, depRow.size());
        } else
            _depData.resizeKeep(_depData.nrow() + 1, _depData.ncol());
            
        _depData.updRow(_depData.nrow() - 1) = depRow;
    }

    /** Get row at index.                                                     */
    RowVectorView getRowAtIndex(size_t index) const {
        return _depData.row(index);
    }

    /** Get row corresponding to the given entry in the independent column.   */
    RowVectorView getRow(const ETX& ind) const {
        auto iter = std::find(_indData.cbegin(), _indData.cend(), ind);

        if(iter == _indData.cend())
            throw Exception{"Independent column has no entry with value " +
                    std::to_string(ind)};

        return _depData.row(std::distance(_indData.cbegin(), iter));
    }

    /** Update row at index.                                                  */
    RowVectorView updRowAtIndex(size_t index) {
        return _depData.updRow(index);
    }

    /** Update row corresponding to the given entry in the independent column.*/
    RowVectorView updRow(const ETX& ind) {
        auto iter = std::find(_indData.cbegin(), _indData.cend(), ind);

        if(iter == _indData.cend())
            throw Exception{"Independent column has no entry with value " +
                    std::to_string(ind)};

        return _depData.updRow(std::distance(_indData.cbegin(), iter));
    }

    /** Get independent column.                                               */
    const std::vector<ETX>& getIndependentColumn() const {
        return _indData;
    }

    /** Get dependent column.                                                 */
    VectorView getDependentColumnAtIndex(size_t index) const {
        return _depData.col(index);
    }

    /** Set independent column at index.                                      */
    void setIndependentColumnAtIndex(size_t index, const ETX& value) {
        if(index >= _indData.size())
            throw Exception{"Invalid index."};
        
        validateRow(index, value, _depData.row(index));
        _indData[index] = value;
    }

protected:
    size_t implementGetNumRows() const override {
        return _depData.nrow();
    }

    size_t implementGetNumColumns() const override {
        return _depData.ncol();
    }

    void validateIndependentMetaData() const override {
        try {
            _independentMetaData.getValueForKey("labels");
        } catch(std::out_of_range&) {
            throw Exception{"Independent metadata does not contain 'labels'."};
        }
    }

    void validateDependentsMetaData() const override {
        unsigned num_cols{};
        try {
            num_cols = _dependentsMetaData.getValueArrayForKey("labels").size();
        } catch (std::out_of_range&) {
            throw Exception{"Dependent metadata does not contain 'labels'."};
        }

        if(num_cols == 0)
            throw Exception{"Dependent metadata for 'labels' has length zero."};

        if(_depData.ncol() != 0 && num_cols != _depData.ncol())
            throw Exception{"Dependent metadata for 'labels' has incorrect "
                    "length."};

        for(const std::string& key : _dependentsMetaData.getKeys()) {
            if(num_cols != _dependentsMetaData.getValueArrayForKey(key).size())
                throw Exception{"All entries in dependent metadata must have "
                        "same length."};
        }
    }

    /** Derived classes optionally can implement this function to validate
    append/update operations.                                                 */
    virtual void validateRow(size_t rowIndex, 
                             const ETX&, 
                             const RowVector&) const {
        // No operation.
    }

    std::vector<ETX>    _indData;
    SimTK::Matrix_<ETY> _depData;
};  // DataTable_

/** See DataTable_ for details on the interface.                              */
using DataTable = DataTable_<SimTK::Real>;

} // namespace OpenSim

#endif //OPENSIM_DATA_TABLE_H_
