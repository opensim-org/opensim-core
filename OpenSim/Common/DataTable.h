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

class InvalidRow : public Exception {
public:
    using Exception::Exception;
};

class IncorrectNumColumns : public InvalidRow {
public:
    IncorrectNumColumns(const std::string& file,
                        size_t line,
                        const std::string& func,
                        size_t expected,
                        size_t received) :
        InvalidRow(file, line, func) {
        std::string msg = "expected = " + std::to_string(expected);
        msg += " received = " + std::to_string(received);

        addMessage(msg);
    }
};

class RowIndexOutOfRange : public IndexOutOfRange {
public:
    using IndexOutOfRange::IndexOutOfRange;
};

class ColumnIndexOutOfRange : public IndexOutOfRange {
public:
    using IndexOutOfRange::IndexOutOfRange;
};

class MissingMetaData : public Exception {
public:
    MissingMetaData(const std::string& file,
                    size_t line,
                    const std::string& func,
                    const std::string& key) :
        Exception(file, line, func) {
        std::string msg = "Missing key '" + key + "'.";

        addMessage(msg);
    }
};

class IncorrectMetaDataLength : public Exception {
public:
    IncorrectMetaDataLength(const std::string& file,
                            size_t line,
                            const std::string& func,
                            const std::string& key,
                            size_t expected,
                            size_t received) :
        Exception(file, line, func) {
        std::string msg = "Key = " + key ;
        msg += " expected = " + std::to_string(expected);
        msg += " received = " + std::to_string(received);

        addMessage(msg);
    }
};

class MetaDataLengthZero : public Exception {
public:
    MetaDataLengthZero(const std::string& file,
                       size_t line,
                       const std::string& func,
                       const std::string& key) :
        Exception(file, line, func) {
        std::string msg = "Key = " + key;

        addMessage(msg);
    }
};

/** AbstractDataTable is the base-class of all DataTable_(templated) allowing 
storage of DataTable_ templated on different types to be stored in a container 
like std::vector. DataTable_ represents a matrix and an additional column. The 
columns of the matrix are dependent columns. The additional column is the
independent column. All dependent columns and the independent column can have
metadata. AbstractDataTable offers:
- Interface to access metadata of independent column and dependent columns.
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

    /** Append row to the DataTable_.                                         

    \throws IncorrectNumCoilumns If the row added is invalid. Validity of the 
    row added is decided by the derived class.                                */
    void appendRow(const ETX& indRow, const RowVector& depRow) {
        validateRow(_indData.size(), indRow, depRow);

        _indData.push_back(indRow);

        if(_depData.nrow() == 0 || _depData.ncol() == 0) {
            try {
                auto& labels = 
                    _dependentsMetaData.getValueArrayForKey("labels");
                OPENSIM_THROW_IF(depRow.ncol() != labels.size(),
                                 IncorrectNumColumns, 
                                 labels.size(), 
                                 static_cast<size_t>(depRow.ncol()));
            } catch(KeyNotFound&) {
                // No "labels". So no operation.
            }
            _depData.resize(1, depRow.size());
        } else
            _depData.resizeKeep(_depData.nrow() + 1, _depData.ncol());
            
        _depData.updRow(_depData.nrow() - 1) = depRow;
    }

    /** Get row at index.                                                     

    \throws RowIndexOutOfRange If index is out of range.                      */
    RowVectorView getRowAtIndex(size_t index) const {
        OPENSIM_THROW_IF(isRowIndexOutOfRange(index),
                         RowIndexOutOfRange, index, 0, _indData.size())
        return _depData.row((int)index);
    }

    /** Get row corresponding to the given entry in the independent column.   

    \throws KeyNotFound If the independent column has no entry with given
                        value.                                                */
    RowVectorView getRow(const ETX& ind) const {
        auto iter = std::find(_indData.cbegin(), _indData.cend(), ind);

        OPENSIM_THROW_IF(iter == _indData.cend(),
                         KeyNotFound, std::to_string(ind));

        return _depData.row((int)std::distance(_indData.cbegin(), iter));
    }

    /** Update row at index.                                                  

    \throws RowIndexOutOfRange If the index is out of range.                  */
    RowVectorView updRowAtIndex(size_t index) {
        OPENSIM_THROW_IF(isRowIndexOutOfRange(index),
                         RowIndexOutOfRange, index, 0, _indData.size());
        return _depData.updRow((int)index);
    }

    /** Update row corresponding to the given entry in the independent column.

    \throws KeyNotFound If the independent column has no entry with given
                        value.                                                */
    RowVectorView updRow(const ETX& ind) {
        auto iter = std::find(_indData.cbegin(), _indData.cend(), ind);

        OPENSIM_THROW_IF(iter == _indData.cend(),
                         KeyNotFound, std::to_string(ind));

        return _depData.updRow((int)std::distance(_indData.cbegin(), iter));
    }

    /** Get independent column.                                               */
    const std::vector<ETX>& getIndependentColumn() const {
        return _indData;
    }

    /** Get dependent column.                                                 

    \throws ColumnIndexOutOfRange If index is out of range.                   */
    VectorView getDependentColumnAtIndex(size_t index) const {
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(index),
                         ColumnIndexOutOfRange, index, 0,
                         static_cast<size_t>(_depData.ncol()));
        return _depData.col((int)index);
    }

    /** Set independent column at index.                                      

    \throws RowIndexOutOfRange If index is out of range.                        
    \throws InvalidRow If this operation invalidates the row. Validation is
                       performed by derived classes.                          */
    void setIndependentColumnAtIndex(size_t index, const ETX& value) {
        OPENSIM_THROW_IF(isRowIndexOutOfRange(index),
                         RowIndexOutOfRange, index, 0, _indData.size());
        validateRow(index, value, _depData.row((int)index));
        _indData[index] = value;
    }

protected:
    bool isRowIndexOutOfRange(size_t index) const {
        return index >= _indData.size();
    }

    bool isColumnIndexOutOfRange(size_t index) const {
        return index >= static_cast<size_t>(_depData.ncol());
    }

    /** Get number of rows.                                                   */
    size_t implementGetNumRows() const override {
        return _depData.nrow();
    }

    /** Get number of columns.                                                */
    size_t implementGetNumColumns() const override {
        return _depData.ncol();
    }

    /** Validate metadata for independent column.                             
    
    \throws InvalidMetaData If independent column's metadata does not contain
                            a key named "labels".                             */
    void validateIndependentMetaData() const override {
        try {
            _independentMetaData.getValueForKey("labels");
        } catch(KeyNotFound&) {
            OPENSIM_THROW(MissingMetaData, "labels");
        }
    }

    /** Validate metadata for dependent columns.

    \throws InvalidMetaData (1) If metadata for dependent columns does not 
                            contain a key named "labels". (2) If ValueArray
                            for key "labels" does not have length equal to the
                            number of columns in the table. (3) If not all
                            entries in the metadata for dependent columns have
                            the correct length (equal to nubmer of columns).  */
    void validateDependentsMetaData() const override {
        unsigned numCols{};
        try {
            numCols = (unsigned)_dependentsMetaData
                                        .getValueArrayForKey("labels").size();
        } catch (KeyNotFound&) {
            OPENSIM_THROW(MissingMetaData, "labels");
        }

        OPENSIM_THROW_IF(numCols == 0,
                         MetaDataLengthZero,"labels");

        OPENSIM_THROW_IF(_depData.ncol() != 0 && numCols != _depData.ncol(),
                         IncorrectMetaDataLength, "labels", 
                         static_cast<size_t>(_depData.ncol()), numCols);

        for(const std::string& key : _dependentsMetaData.getKeys()) {
            OPENSIM_THROW_IF(numCols != 
                             _dependentsMetaData.
                             getValueArrayForKey(key).size(),
                             IncorrectMetaDataLength, key, numCols,
                             _dependentsMetaData.
                             getValueArrayForKey(key).size());
        }
    }

    /** Derived classes optionally can implement this function to validate
    append/update operations.                                                 

    \throws InvalidRow If the given row considered invalid by the derived
                       class.                                                 */
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
