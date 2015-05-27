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
 * Author(s): Ajay Seth                                                       *
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
* This file defines the  DataTable class, which is used by OpenSim to provide  *
* an in-memory container for data access and manipulation.                     *
*/

#ifndef OPENSIM_DATA_TABLE_H_
#define OPENSIM_DATA_TABLE_H_

#include <memory>

namespace OpenSim {

/**----------------------------------------------------------------------------*
* DataTableBase defines an interface for an in-memory numerical table data     *
* structure -- DataTable. A DataTable is independent of the data source used   *
* to populate it. A concrete DataTable provides a random access to data        *
* elements by row and/or column indices as well as column by column-name.      *
* This class allows storing different DataTables in containers.                *
*                                                                              *
* @author Ajay Seth                                                            *
*-----------------------------------------------------------------------------*/
class DataTableBase {
 public:
  using size_t       = std::size_t;
  using string       = std::string;
  using MetaDataType = std::unordered_map<string, double>;

  virtual ~DataTableBase() = default;

  virtual DataTableBase clone() const = 0;

  /// Get meta-data.
  virtual MetaDataType getMetaData() const = 0;
  /// Get meta-data value for a given key.
  virtual double getMetaData(const string& key) const = 0;
  /// Set meta-data. Existing meta-data is replaced with a *copy* of
  /// new_metadata.
  virtual void setMetaData(const MetaDataType& new_metadata) = 0;
  /// Set meta-data. Existing meta-data is replaced with a *move* of
  /// new_metadata.c 
  virtual void setMetaData(MetaDataType&& new_meta_data) = 0;
  /// Set individual key-value pairs in meta-data.
  virtual void setMetaData(const string& key, const double value) = 0;

  /// Number of rows.
  virtual size_t getNumRows() const = 0;
  /// Number of columns.
  virtual size_t getNumCols() const = 0;

  /// Does column(by the given name) exist ?
  virtual bool hasCol(const string& name) const = 0;
  /// Does column(by the given index) exist ?
  virtual bool hasCol(size_t ind) const = 0;

  /// Get column index corresponding to given name. If column name does
  /// not exist, throws ColumnDoesNotExist exception.
  virtual size_t getColInd(const string& name) const = 0;
  /// Get column name corresponding to given index. If column index does
  /// not exist, throws ColumnDoesNotExist exception.
  virtual string getColName(const size_t ind) const = 0;
  /// Set column name corresponding to given index. If column index does
  /// not exist, throws ColumnDoesNotExist exception.
  virtual void setColName(const size_t ind, const string& new_colname) = 0;
  /// Change name of a column from old to new. If old column name does not
  /// exist, throws ColumnDoesNotExist exception.
  virtual void setColName(const string& old, const string& new) = 0;

  /// Get column names.
  virtual std::vector<string> getColNames() const = 0;
  /// Set column names with a *copy* of colnames.
  virtual void setColNames(const std::vector<string>& colnames) = 0;
  /// Set column names with a *move* of colnames.
  virtual void setColNames(std::vector<string>&& colnames) = 0;

  /// Write to an output stream.
  virtual void writeToStream(std::ostream &out) const = 0;

};  // DataTableBase


class ColumnDoesNotExist : public std::runtime_error {
 public:
 ColumnDoesNotExist(const std::string& expl) : runtime_error(expl) {}
};

/** Concrete Implementation of the <tt>DataTable<DataType></tt>
@tparam DataType  Any numerical type: (SimTK::) Real, Vec<M>, Mat<M,N>, ...*/
template<typename DataType = SimTK::Real>
class DataTable_ : public DataTableBase {
//=============================================================================
// METHODS
//=============================================================================
public:
    virtual ~DataTable_(){};

    /** Construct an empty DataTable */
    DataTable_() : _type(typeid(DataType)) {}

    /** Construct a pre-sized DataTable to be filled in later. If values
        are accessed before they are assigned they will be NaN. */
    DataTable_(size_t nrows, size_t ncols, 
        const DataType& val = DataType(SimTK::NaN)) :
        _data(int(nrows), int(ncols), val),
        _type(typeid(DataType)) {}

    /** Convenience construction of a DataTable from a file name with type
        identified by the file extension. E.g. `results.csv`, `stats.sto`, ...
    @param[in] filename         the name of the data file to be accessed
    */
    DataTable_(const std::string& filename) :
        _type(typeid(DataType)) {
        std::string daId = FileAdapter::findExtension(filename);
        auto* reader = FileAdapter::createAdapter(daId);
        if (!reader){
            throw Exception("DataTable() no adapter to read file '"
                + filename + "' could be found.");
        }
        reader->setFilename(filename);
        reader->prepareForReading(*this);
        reader->read();
        delete reader;
    }

    DataTable_& operator=(const DataTable_& dt) {
        _metaData = dt.getMetaData();
        _data = dt.getAsMatrix() ;
        _columnLabels = dt.getColumnLabels();
        return *this;
    }

    virtual DataTableBase* clone() const override {
        return new DataTable_(*this);
    }


    const XMLDocument& getMetaData() const override final { return _metaData; }
    XMLDocument& updMetaData() override final { return _metaData; }

    size_t  getNumRows() const override { return _data.nrow(); }
    size_t  getNumCols() const override { return _data.ncol(); }

    const Array<std::string>& getColumnLabels() const override {
        return _columnLabels;
    }

    Array<std::string>& updColumnLabels() override {
        return _columnLabels;
    }
    
    /** Return true if the DataTable contains a column for the label. */
    bool hasColumn(const std::string& label) const override{
        return (_columnLabels.findIndex(label) >= 0);
    }

    size_t getColumnIndex(const std::string& label) const override {
        return _columnLabels.findIndex(label);
    }

    /** Read-only access to a row of data in the table as a RowVector.*/
    const SimTK::RowVectorView_<DataType> getRow(size_t row) const {
        return _data[int(row)];
    }

    /** Write access to a column of data as a RowVector */
    SimTK::RowVectorView_<DataType> updRow(size_t row) {
        return _data[int(row)];
    }

    /** Read-only access to a column of data in the table as a Vector.*/
    const SimTK::VectorView_<DataType> getColumn(size_t col) const {
        return _data(int(col));
    }
    /** Write access to a column of data as a Vector */
    SimTK::VectorView_<DataType> updColumn(size_t col) {
        return _data(int(col));
    }

    /** Read-only access to an DataType element in the table by row and
        column indices.*/
    const DataType& getElement(size_t row, size_t col) const {
        return _data(int(row), int(col));
    }

    /** Read-only access to the table as a Matrix */
    const SimTK::Matrix_<DataType>& getAsMatrix() const {
        return _data;
    }

    /** Append a row of data as a RowVector to the table. */
    void appendRow(const SimTK::RowVector_<DataType>& row) {
        // ncols specified by the table unless it is zero, in which
        // case allow the first row appended to dictate its size
        int ncols = _data.ncol() == 0 ? row.size() : _data.ncol();
        SimTK_ASSERT_ALWAYS(row.size() == ncols, "DataTable::appendRow() "
            "row length does match number of columns.");
        SimTK_ASSERT_ALWAYS(row.size() > 0, "DataTable::appendRow() "
            "row is empty.");
        _data.resizeKeep(_data.nrow() + 1, ncols);
        _data[_data.nrow() - 1].updAsRowVector() = row;
    }

    /** Append another data table's rows to this table. If the number of
        columns are in compatible it will throw and exception. */
    void appendDataTable(const DataTable_& table) {
        size_t nrows = table.getNumRows();
        if (getNumCols() != table.getNumCols()) {
            throw Exception("DataTable::appendDataTable() cannot append a "
                " DataTable with a different number of columns.");
        }
        // resize once to tack on the new table
        size_t offset = _data.nrow();
        _data.resizeKeep(int( offset + nrows), int(getNumCols()) );
        for (size_t i = 0; i < nrows; ++i){
            _data[int(i + offset)].updAsRowVector() = table.getRow(i);
        }
    }

    /** Dump the contents of table and strings to an output stream.
        Primarily used for debugging purposes.*/
    void dumpToStream(std::ostream &out) const override {
        out << "DataTable of type: " << getDataTypeInfo().name() << std::endl;
        out << getMetaData() << std::endl;
        out << getColumnLabels() << std::endl;
        for (int i=0; i < getNumRows(); ++i){
            out << getRow(i) << std::endl;
        }
    }

    SimTK_DOWNCAST(DataTable_, DataTableBase);

protected:
    /** Satisfy DataTableBase to determine its numerical data type */
    const std::type_index& getDataTypeInfo() const override {
        return _type;
    }

    /** Enable derived types to have write access to the data matrix */
    SimTK::Matrix_<DataType>& updAsMatrix() {
        return _data;
    }

private:
    // Amorphous data that cannot be put into a matrix.
    XMLDocument _metaData;
    // The internal data model for fast tabular access
    SimTK::Matrix_<DataType> _data;
    // The column labels
    Array<std::string> _columnLabels;
    // Numerical data type contained in the table
    const std::type_index _type;
//=============================================================================
};  // END of class DataTable_<T>
//=============================================================================

inline std::ostream& operator<<(std::ostream &out, const DataTableBase& dt)
{
    out << dt.getColumnLabels() << std::endl;
    dt.dumpToStream(out);
    out << std::endl;
    return out;
}

typedef DataTable_<SimTK::Real> DataTable;

} //namespace OpenSim
//=============================================================================
//=============================================================================

#endif //OPENSIM_DATA_TABLE_H_
