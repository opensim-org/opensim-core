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

#include "AbstractDataTable.h"
#include "FileAdapter.h"

namespace OpenSim {

/** DataTable_ is a in-memory storage container for data with support for 
holding metadata (using the base class AbstractDataTable). Data contains an 
independent column and a set of dependent columns. The type of the independent 
column can be configured using ETX (template param). The type of the dependent 
columns, which together form a matrix, can be configured using ETY (template 
param). Independent and Dependent columns can contain metadata. DataTable_ as a 
whole can contain metadata.                                                   

\tparam ETX Type of each element of the underlying matrix holding dependent 
            data.
\tparam ETY Type of each element of the column holding independent data.      */
template<typename ETX = double, typename ETY = SimTK::Real>
class DataTable_ : public AbstractDataTable {
    static_assert(!std::is_reference<ETY>::value,
                  "Template argument ETY cannot be a 'reference'.");
    static_assert(!std::is_pointer<ETY>::value,
                  "Template argument ETY cannot be a 'pointer'.");
    static_assert(!std::is_const<ETY>::value && !std::is_volatile<ETY>::value,
                  "Template argument ETY cannot be 'const' or 'volatile'.");

public:
    /** Type of each row of matrix holding dependent data.                    */
    typedef SimTK::RowVector_<ETY>     RowVector;
    /** (Read only view) Type of each row of matrix.                          */
    typedef SimTK::RowVectorView_<ETY> RowVectorView;
    /** Type of each column of matrix holding dependent data.                 */
    typedef SimTK::VectorView_<ETY>    VectorView;
    /** Type of the matrix holding the dependent data.                        */
    typedef SimTK::Matrix_<ETY>        Matrix;
    /** (Read only view) Type of the matrix  holding the dependent data.      */
    typedef SimTK::MatrixView_<ETY>    MatrixView;

    DataTable_()                             = default;
    DataTable_(const DataTable_&)            = default;
    DataTable_(DataTable_&&)                 = default;
    DataTable_& operator=(const DataTable_&) = default;
    DataTable_& operator=(DataTable_&&)      = default;
    ~DataTable_()                            = default;

    std::shared_ptr<AbstractDataTable> clone() const override {
        return std::shared_ptr<AbstractDataTable>{new DataTable_{*this}};
    }

    /** Construct DataTable_ from a file.                                     

    \param filename Name of the file. File should contain only one table. For
                    example, trc, csv & sto files contain one table whereas a 
                    c3d file can contain more than.                           

    \throws InvalidArgument If the input file contains more than one table.   
    \throws InvalidArgument If the input file contains a table that is not of
                            this DataTable_ type.                             */
    DataTable_(const std::string& filename) {
        auto absTables = FileAdapter::readFile(filename);

        OPENSIM_THROW_IF(absTables.size() > 1,
                         InvalidArgument,
                         "File '" + filename + 
                         "' contains more than one table.");

        auto* absTable = (absTables.cbegin()->second).get();
        DataTable_* table{};

        table = dynamic_cast<DataTable_*>(absTable);
        OPENSIM_THROW_IF(table == nullptr,
                         InvalidArgument,
                         "DataTable cannot be created from file '" + filename +
                         "'. Type mismatch.");

        *this = std::move(*table);
    }

    /// @name Row accessors/mutators.
    /// Following get/upd functions operate on matrix and not the independent
    /// column.
    /// The function appendRow() is pretty flexible and it is possible to 
    /// append a row with any sequence of elements. Following are some examples:
    /// \code
    /// // Easiest way to append a row is to provide the list of elements 
    /// // directly to appendRow.
    /// // For a table with elements of type double, this could look like below.
    /// table.appendRow(0.1, // Independent column.
    ///                 {0.3, 0.4, 0.5, 0.6}); // 4 elements of type double.
    /// // For a table with elements of type SimTK::Vec3, this could like below.
    /// table.appendRow(0.1, // Independent column.
    ///                 {{0.31, 0.32, 0.33},
    ///                  {0.41, 0.42, 0.43},
    ///                  {0.51, 0.52, 0.53},
    ///                  {0.61, 0.62, 0.63}}); // 4 elements of SimTK::Vec3.
    /// \endcode
    /// \code
    /// // It is possible to append a sequence container like std::vector or 
    /// // std::list by providing it directly to appendRow.
    /// // For a table with elements of type double, this could look like below.
    /// std::vector<double> row{0.3, 0.4, 0.5, 0.6};
    /// table.appendRow(0.1, row);
    /// // For a table with elements of type SimTK::Vec3, this could look like
    /// // below.
    /// std::vector<SimTK::Vec3> row{{0.31, 0.32, 0.33},
    ///                              {0.41, 0.42, 0.43},
    ///                              {0.51, 0.52, 0.53},   // 4 elements of
    ///                              {0.61, 0.62, 0.63}}); //  SimTK::Vec3.
    /// table.appendRow(0.1, row);
    /// \endcode
    /// \code
    /// // A SimTK::RowVector can be provided to appendRow as well.
    /// // For a table with elements of type double, this could look like below.
    /// SimTK::RowVector row{0.3, 0.4, 0.5, 0.6};
    /// table.appendRow(0.1, row);
    /// // For a table with elements of type SimTK::Vec3, this could look like
    /// // below.
    /// SimTK::RowVector_<SimTK::Vec3> row{{0.31, 0.32, 0.33},
    ///                                    {0.41, 0.42, 0.43},
    ///                                    {0.51, 0.52, 0.53},  // 4 elements of
    ///                                    {0.61, 0.62, 0.63}}); // SimTK::Vec3.
    /// table.appendRow(0.1, row);
    /// \endcode
    /// \code
    /// // It is possible to be use a pair of iterators to append a row as well.
    /// // This could arise in situations where you might want to append a row
    /// // using a subset of elements in a sequence.
    /// // For a table with elements of type double, this could look like below.
    /// std::vector<double> row{0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
    /// table.appendRow(0.1, // Independent column.
    ///                 row.begin() + 1, // Start from second element (0.4).
    ///                 row.end() - 1);  // End at last but one (0.7).
    /// // For a table with elements of type SimTK::Vec3, this could look like
    /// // below.
    /// std::vector<SimTK::Vec3> row{{0.31, 0.32, 0.33},
    ///                              {0.41, 0.42, 0.43},
    ///                              {0.51, 0.52, 0.53},   
    ///                              {0.61, 0.62, 0.63},
    ///                              {0.71, 0.72, 0.73},   // 6 elements of
    ///                              {0.81, 0.82, 0.83}}); //  SimTK::Vec3.
    /// table.appendRow(0.1, // Independent column.
    ///                 row.begin() + 1, // Start from second element.
    ///                 row.end() - 1); // End at last but one.
    /// \endcode
    /// @{

    /** Append row to the DataTable_.

    \param indRow Entry for the independent column corresponding to the row to
                  be appended.
    \param container Sequence container holding the elements of the row to be
                     appended.

    \throws IncorrectNumColumns If the row added is invalid. Validity of the 
    row added is decided by the derived class.                                */
    template<typename Container>
    void appendRow(const ETX& indRow, const Container& container) {
        using Value = decltype(*(container.begin()));
        using RmrefValue = typename std::remove_reference<Value>::type;
        using RmcvRmrefValue = typename std::remove_cv<RmrefValue>::type;
        static_assert(std::is_same<ETY, RmcvRmrefValue>::value,
                      "The 'container' specified does not provide an iterator "
                      "which when dereferenced provides elements that "
                      "are of same type as elements of this table.");

        appendRow(indRow, container.begin(), container.end());
    }

    /** Append row to the DataTable_.

    \param indRow Entry for the independent column corresponding to the row to
                  be appended.
    \param container std::initializer_list containing elements of the row to be
                     appended.

    \throws IncorrectNumColumns If the row added is invalid. Validity of the 
    row added is decided by the derived class.                                */
    void appendRow(const ETX& indRow, 
                   const std::initializer_list<ETY>& container) {
        appendRow(indRow, container.begin(), container.end());
    }

    /** Append row to the DataTable_.

    \param indRow Entry for the independent column corresponding to the row to
                  be appended.
    \param begin Iterator representing the beginning of the row to be appended.
    \param end Iterator representing one past the end of the row to be appended.

    \throws IncorrectNumColumns If the row added is invalid. Validity of the 
    row added is decided by the derived class.                                */
    template<typename RowIter>
    void appendRow(const ETX& indRow, RowIter begin, RowIter end) {
        using Value = decltype(*begin);
        using RmrefValue = typename std::remove_reference<Value>::type;
        using RmcvRmrefValue = typename std::remove_cv<RmrefValue>::type;
        static_assert(std::is_same<ETY, RmcvRmrefValue>::value,
                      "The iterator 'begin' provided does not provide elements"
                      " that are of same type as elements of this table.");

        RowVector row{static_cast<int>(std::distance(begin, end))};
        int ind{0};
        for(auto it = begin; it != end; ++it)
            row[ind++] = *it;

        appendRow(indRow, row);
    }

    /** Append row to the DataTable_.                                         

    \throws IncorrectNumColumns If the row added is invalid. Validity of the 
    row added is decided by the derived class.                                */
    void appendRow(const ETX& indRow, const RowVector& depRow) {
        validateRow(_indData.size(), indRow, depRow);

        _indData.push_back(indRow);

        if(_depData.nrow() == 0 || _depData.ncol() == 0) {
            try {
                auto& labels = 
                    _dependentsMetaData.getValueArrayForKey("labels");
                OPENSIM_THROW_IF(static_cast<unsigned>(depRow.ncol()) != 
                                 labels.size(),
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
                         RowIndexOutOfRange, 
                         index, 0, static_cast<unsigned>(_indData.size() - 1));

        return _depData.row(static_cast<int>(index));
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
                         RowIndexOutOfRange, 
                         index, 0, static_cast<unsigned>(_indData.size() - 1));

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

    /// @} End of Row accessors/mutators.

    /// @name Dependent and Independent column accessors/mutators.
    /// @{

    /** Get independent column.                                               */
    const std::vector<ETX>& getIndependentColumn() const {
        return _indData;
    }

    /** Get dependent column at index.

    \throws ColumnIndexOutOfRange If index is out of range for number of columns
                                  in the table.                               */
    VectorView getDependentColumnAtIndex(size_t index) const {
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(index),
                         ColumnIndexOutOfRange, index, 0,
                         static_cast<size_t>(_depData.ncol() - 1));

        return _depData.col(static_cast<int>(index));
    }

    /** Get dependent Column which has the given column label.                

    \throws KeyNotFound If columnLabel is not found to be label of any existing
                        column.                                               */
    VectorView getDependentColumn(const std::string& columnLabel) const {
        return _depData.col(static_cast<int>(getColumnIndex(columnLabel)));
    }

    /** Update dependent column at index.

    \throws ColumnIndexOutOfRange If index is out of range for number of columns
                                  in the table.                               */
    VectorView updDependentColumnAtIndex(size_t index) {
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(index),
                         ColumnIndexOutOfRange, index, 0,
                         static_cast<size_t>(_depData.ncol() - 1));

        return _depData.updCol(static_cast<int>(index));
    }

    /** Update dependent Column which has the given column label.

    \throws KeyNotFound If columnLabel is not found to be label of any existing
                        column.                                               */
    VectorView updDependentColumn(const std::string& columnLabel) {
        return _depData.updCol(static_cast<int>(getColumnIndex(columnLabel)));
    }

    /** %Set value of the independent column at index.

    \throws RowIndexOutOfRange If rowIndex is out of range.
    \throws InvalidRow If this operation invalidates the row. Validation is
                       performed by derived classes.                          */
    void setIndependentValueAtIndex(size_t rowIndex, const ETX& value) {
        OPENSIM_THROW_IF(isRowIndexOutOfRange(rowIndex),
                         RowIndexOutOfRange, 
                         rowIndex, 0, 
                         static_cast<unsigned>(_indData.size() - 1));

        validateRow(rowIndex, value, _depData.row((int)rowIndex));
        _indData[rowIndex] = value;
    }

    /// @}

    /// @name Matrix accessors/mutators.
    /// Following functions operate on the matrix not including the independent
    /// column.
    /// @{

    /** Get a read-only view to the underlying matrix.                        */
    const MatrixView& getMatrix() const {
        return _depData.getAsMatrixView();
    }

    /** Get a read-only view of a block of the underlying matrix.             

    \throws InvalidArgument If numRows or numColumns is zero.
    \throws RowIndexOutOfRange If one or more rows of the desired block is out
                               of range of the matrix.
    \throws ColumnIndexOutOfRange If one or more columns of the desired block is
                                  out of range of the matrix.                 */
    MatrixView getMatrixBlock(size_t rowStart,
                              size_t columnStart,
                              size_t numRows,
                              size_t numColumns) const {
        OPENSIM_THROW_IF(numRows == 0 || numColumns == 0,
                         InvalidArgument,
                         "Either numRows or numColumns is zero.");
        OPENSIM_THROW_IF(isRowIndexOutOfRange(rowStart),
                         RowIndexOutOfRange,
                         rowStart, 0, 
                         static_cast<unsigned>(_depData.nrow() - 1));
        OPENSIM_THROW_IF(isRowIndexOutOfRange(rowStart + numRows - 1),
                         RowIndexOutOfRange,
                         rowStart + numRows - 1, 0, 
                         static_cast<unsigned>(_depData.nrow() - 1));
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(columnStart),
                         ColumnIndexOutOfRange,
                         columnStart, 0, 
                         static_cast<unsigned>(_depData.ncol() - 1));
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(columnStart + numColumns - 1),
                         ColumnIndexOutOfRange,
                         columnStart + numColumns - 1, 0, 
                         static_cast<unsigned>(_depData.ncol() - 1));

        return _depData.block(static_cast<int>(rowStart),
                              static_cast<int>(columnStart),
                              static_cast<int>(numRows),
                              static_cast<int>(numColumns));
    }

    /** Get a writable view to the underlying matrix.                         */
    MatrixView& updMatrix() {
        return _depData.updAsMatrixView();
    }

    /** Get a writable view of a block of the underlying matrix.

    \throws InvalidArgument If numRows or numColumns is zero.
    \throws RowIndexOutOfRange If one or more rows of the desired block is out
                               of range of the matrix.
    \throws ColumnIndexOutOfRange If one or more columns of the desired block is
                                  out of range of the matrix.                 */
    MatrixView updMatrixBlock(size_t rowStart,
                              size_t columnStart,
                              size_t numRows,
                              size_t numColumns) {
        OPENSIM_THROW_IF(numRows == 0 || numColumns == 0,
                         InvalidArgument,
                         "Either numRows or numColumns is zero.");
        OPENSIM_THROW_IF(isRowIndexOutOfRange(rowStart),
                         RowIndexOutOfRange,
                         rowStart, 0, 
                         static_cast<unsigned>(_depData.nrow() - 1));
        OPENSIM_THROW_IF(isRowIndexOutOfRange(rowStart + numRows - 1),
                         RowIndexOutOfRange,
                         rowStart + numRows - 1, 0, 
                         static_cast<unsigned>(_depData.nrow() - 1));
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(columnStart),
                         ColumnIndexOutOfRange,
                         columnStart, 0, 
                         static_cast<unsigned>(_depData.ncol() - 1));
        OPENSIM_THROW_IF(isColumnIndexOutOfRange(columnStart + numColumns - 1),
                         ColumnIndexOutOfRange,
                         columnStart + numColumns - 1, 0, 
                         static_cast<unsigned>(_depData.ncol() - 1));

        return _depData.updBlock(static_cast<int>(rowStart),
                                 static_cast<int>(columnStart),
                                 static_cast<int>(numRows),
                                 static_cast<int>(numColumns));
    }

    /// @}

protected:
    /** Check if row index is out of range.                                   */
    bool isRowIndexOutOfRange(size_t index) const {
        return index >= _indData.size();
    }

    /** Check if column index is out of range.                                */
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
        size_t numCols{};
        try {
            numCols = (unsigned)_dependentsMetaData
                                        .getValueArrayForKey("labels").size();
        } catch (KeyNotFound&) {
            OPENSIM_THROW(MissingMetaData, "labels");
        }

        OPENSIM_THROW_IF(numCols == 0,
                         MetaDataLengthZero,"labels");

        OPENSIM_THROW_IF(_depData.ncol() != 0 && 
                         numCols != static_cast<unsigned>(_depData.ncol()),
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

/** Print DataTable out to a stream. Metadata is not printed to the stream as it
is currently allowed to contain objects that do not support this operation.   
Meant to be used for Debugging only.                                          */
template<typename ETX, typename ETY>
std::ostream& operator<<(std::ostream& outStream,
                         const DataTable_<ETX, ETY>& table) {
    outStream << "----------------------------------------------------------\n";
    outStream << "NumRows: " << table.getNumRows()    << std::endl;
    outStream << "NumCols: " << table.getNumColumns() << std::endl;
    outStream << "Column-Labels: ";
    const auto& labels = table.getColumnLabels();
    if(!labels.empty()) {
        outStream << "['" << labels[0] << "'";
        if(labels.size() > 1)
            for(size_t l = 1; l < labels.size(); ++l)
                outStream << " '" << labels[l] << "'";
        outStream << "]" << std::endl;
    }
    for(size_t r = 0; r < table.getNumRows(); ++r) {
        outStream << table.getIndependentColumn().at(r) << " ";
        outStream << table.getRowAtIndex(r) << std::endl;
    }

    outStream << "----------------------------------------------------------\n";
    return outStream;
}

/** See DataTable_ for details on the interface.                              */
typedef DataTable_<double, double> DataTable;
/** See DataTable_ for details on the interface.                              */
typedef DataTable_<double, SimTK::Vec3> DataTableVec3;

} // namespace OpenSim

#endif //OPENSIM_DATA_TABLE_H_
