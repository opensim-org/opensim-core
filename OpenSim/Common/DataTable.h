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

// Standard headers.
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
#include <string>
#include <type_traits>
// Non-standard headers.
#include <SimTKcommon.h>


namespace OpenSim {
  class DataTableBase;
  template<typename ET> class DataTable_;

  using DataTable = DataTable_<SimTK::Real>;
} // namespace OpenSim


/**----------------------------------------------------------------------------*
* DataTableBase defines an interface for an in-memory numerical table data     *
* structure -- DataTable. A DataTable is independent of the data source used   *
* to populate it. A concrete DataTable provides a random access to data        *
* elements by row and/or column indices as well as column by column-name.      *
* This class allows storing different DataTables in containers.                *
*                                                                              *
* @author Ajay Seth                                                            *
*-----------------------------------------------------------------------------*/
class OpenSim::DataTableBase {

public:
  using size_t                    = std::size_t;
  using string                    = std::string;
  using MetaDataType              = std::unordered_map<string, string>;
  using MetaDataIterType          = MetaDataType::iterator;
  using MetaDataConstIterType     = MetaDataType::const_iterator;
  using ColNamesType              = std::unordered_map<string, size_t>;
  using ColNamesIterType          = ColNamesType::iterator;
  using ColNamesConstIterType     = ColNamesType::const_iterator;
  using MetaDataKVType            = std::pair<string, string>;
  using MetaDataIterPairType      = std::pair<MetaDataIterType, 
					      MetaDataIterType>;
  using MetaDataConstIterPairType = std::pair<MetaDataConstIterType, 
					      MetaDataConstIterType>;
  using ColNamesIterPairType      = std::pair<ColNamesIterType,
					      ColNamesIterType>;
  using ColNamesConstIterPairType = std::pair<ColNamesConstIterType,
					      ColNamesConstIterType>;

  virtual ~DataTableBase() = default;

  virtual DataTableBase& clone() const = 0;

  /// Get meta-data.
  virtual MetaDataConstIterPairType getMetaData() const = 0;
  
  /// Set meta-data. Existing meta-data is replaced with a *copy* of
  /// new_metadata.
  virtual MetaDataIterPairType updMetaData() = 0;

  /// Number of rows.
  virtual size_t getNumRows() const = 0;

  /// Number of columns.
  virtual size_t getNumCols() const = 0;

  /// Does column(by the given name) exist ?
  virtual bool hasCol(const string& name) const = 0;

  /// Does column(by the given index) exist ?
  virtual bool hasCol(const size_t ind) const = 0;

  /// Get column index corresponding to given name. If column name does
  /// not exist, throws ColumnDoesNotExist exception.
  virtual size_t getColInd(const string& colname) const = 0;

  /// Get column name corresponding to given index. If column index does
  /// not exist, throws ColumnDoesNotExist exception.
  virtual string getColName(const size_t colind) const = 0;

  /// Set column name corresponding to given index. If column index does
  /// not exist, throws ColumnDoesNotExist exception.
  virtual void setColName(const size_t ind, const string& new_colname) = 0;

  /// Change name of a column from old to new. If old column name does not
  /// exist, throws ColumnDoesNotExist exception.
  virtual void setColName(const string& old_colname, 
			  const string& new_colname) = 0;

  /// Get column names.
  virtual ColNamesConstIterPairType getColNames() const = 0;

  /// Set column names with a *copy* of colnames.
  virtual ColNamesIterPairType updColNames() = 0;

  /// Set column names with a *move* of colnames.
  virtual void repColNames(const std::vector<string>& colnames) = 0;
}; // DataTableBase


class ColumnDoesNotExist : public std::runtime_error {
public:
  ColumnDoesNotExist(const std::string& expl) : runtime_error(expl) {}
};

class ZeroElements : public std::runtime_error {
public:
  ZeroElements(const std::string& expl) : runtime_error(expl) {}
};

class InvalidEntry : public std::runtime_error {
public:
  InvalidEntry(const std::string& expl) : runtime_error(expl) {}
};


/**----------------------------------------------------------------------------*
* DataTable_ is a concrete implementation of DataTableBase interface. Concrete *
* Implementation of DataTableBase interface. The element type is specified     *
* through the template argument.                                               *
*-----------------------------------------------------------------------------*/
// * Derive DataTable for timeseries with an additional private member.
// * Allow for move constructors.
// * Allow setting the type of meta-data.
// * Provide a global function to create DataTable that infers the element
//   datatype.
// * Wrap DataTable in a smart pointer or make DataTable itself a smart
//   pointer ?
// * Are all the functions efficient ? Any better ways of using SimTK::Matrix_ ?
// * Are all the operations correct ?
// * Do the mutating operations allow users to invalidate the DataTable ?
// * Include time-complexity in documentation.
// * Are the function names and signatures consistent with rest of the code ?
// * How to enforce constness in the functions getRow and getCol ?
// * For the function getAsMatrix, is it better to return reference to
//   underlying matrix for efficiency ?
template<typename ET = SimTK::Real> // Element datatype.
class OpenSim::DataTable_ : public OpenSim::DataTableBase {
public:
  /// Enum to specify if the the input iterator is traversing data row-wise or
  /// col-wise.
  enum InputItDir {
    ROWWISE,
    COLWISE
  };

  /// Construct an empty DataTable.
  DataTable_() : 
    data{}, metadata{}, col_ind{} {}

  /// Construct a pre-sized DataTable populated with given value val. Default
  /// value for val is NaN.
  DataTable_(size_t nrows,
	     size_t ncols,
	     const ET& val = ET(SimTK::NaN)) :
    data{int(nrows), int(ncols), val}, metadata{}, col_ind{} {}

  /// Construct DataTable using an "input iterator". 
  template<typename InputIt>
  DataTable_(InputIt first,
	     InputIt last,
	     size_t ndir,
	     InputItDir dir = ROWWISE) :
    data{dir == ROWWISE ? 1 : ndir, dir == COLWISE ? ndir : 1},
    metadata{},
    col_ind{} {
    if(!(first != last))
      throw ZeroElements{""};

    size_t row{0};
    size_t col{0};
    while(first != last) {
      data.set(row, col, *first);
      ++first;
      if(dir == ROWWISE && ++col == ndir && first != last) {
	col = 0;
	++row;
	data.resizekeep(data.nrow() + 1, data.ncol());
      } else if(++row == ndir && first != last) {
	row = 0;
	++col;
	data.resizekeep(data.nrow(), data.ncol() + 1);
      }
    }
  }

  DataTable_(const DataTable_& dt) :
    data{dt.data}, metadata{dt.metadata}, col_ind{dt.col_ind} {}

  DataTable_& clone() const override {
    return *(new DataTable_{*this});
  }

  DataTable_(DataTable_&& dt) :
    data{std::move(dt.data)},
    metadata{std::move(dt.metadata)},
    col_ind{std::move(dt.col_ind)} {}

  virtual ~DataTable_() {};

  DataTable_& operator=(const DataTable_& dt) {
    metadata = dt.metadata;
    data = dt.data ;
    col_ind = dt.col_ind;
    return *this;
  }

  MetaDataConstIterPairType getMetaData() const override {
    return std::make_pair(metadata.cbegin(), metadata.cend());
  }

  string getMetaDataAsString(string key) const {
    return metadata.at(key);
  }
  int getMetaDataAsInt(string key) const {
    return std::stoi(metadata.at(key));
  }
  long getMetaDataAsLong(string key) const {
    return std::stol(metadata.at(key));
  }
  long long getMetaDataAsLongLong(string key) const {
    return std::stoll(metadata.at(key));
  }
  unsigned long getMetaDataAsULong(string key) const {
    return std::stoul(metadata.at(key));
  }
  unsigned long long getMetaDataAsULongLong(string key) const {
    return std::stoull(metadata.at(key));
  }
  float getMetaDataAsFloat(string key) const {
    return std::stof(metadata.at(key));
  }
  double getMetaDataAsDouble(string key) const {
    return std::stod(metadata.at(key));
  }
  long double getMetaDataAsLongDouble(string key) const {
    return std::stold(metadata.at(key));
  }

  MetaDataIterPairType updMetaData() override {
    return std::make_pair(metadata.begin(), metadata.end());
  }

  /// Set individual key-value pairs in meta-data.
  template<typename VT, 
	   typename = typename std::enable_if<
	     std::is_constructible<string, VT>::value>::type>
  void insertMetaData(const string& key, VT&& value) {
    metadata.insert(std::make_pair(key, std::forward<VT>(value)));
  }

  template<typename VT, 
  	   typename = typename std::enable_if<
	     std::is_arithmetic<VT>::value>::type>
  void insertMetaData(const string& key, const VT value) { 
    metadata.insert(std::make_pair(key, std::to_string(value)));
  }

  /// Replace meta-data. Existing meta-data is replaced with a *copy* of
  /// new_metadata, which must be an associative container.
  template<typename AC>
  void repMetaData(const AC& new_metadata) {
    metadata.clear();
    metadata.insert(new_metadata.begin(), new_metadata.end());
  }

  /// Replace meta-data. Existing meta-data is replaced with a *move* of
  /// new_metadata.
  void repMetaData(MetaDataType&& new_metadata) {
    metadata = std::move(new_metadata);
  }

  template<typename InputIter>
  void repMetaData(InputIter begin, InputIter end) {
    metadata.clear();
    metadata.insert(begin, end);
  }

  size_t getNumRows() const override {
    return data.nrow(); 
  }

  size_t getNumCols() const override {
    return data.ncol(); 
  }

  bool hasCol(const std::string& colname) const override {
    return col_ind.find(colname) != col_ind.end();
  }

  bool hasCol(const size_t ind) const override {
    return ind >= 0 && ind < data.ncol();
  }

  size_t getColInd(const string& colname) const override {
    return col_ind.at(colname);
  }

  string getColName(const size_t colind) const override {
    using ColNamesValueType = typename ColNamesType::value_type;
    auto res = std::find(col_ind.cbegin(), 
			 col_ind.cend(), 
			 [colind] (const ColNamesValueType& kv) {
			   return kv.second == colind;
			 });
    if(res == col_ind.end())
      throw std::out_of_range{""};

    return res->first;
  }

  void setColName(const size_t colind, const string& new_colname) override {
    using ColNamesValueType = typename ColNamesType::value_type;
    auto res = std::find(col_ind.begin(), 
			 col_ind.end(), 
			 [colind] (const ColNamesValueType& kv) {
			   return kv.second == colind;
			 });
    if(res == metadata.end())
      throw std::out_of_range{""};
    else {
      col_ind.erase(res);
      col_ind[new_colname] = colind;
    }
  }

  void setColName(const string& old_colname, 
		  const string& new_colname) override {
    size_t colind{col_ind.at(old_colname)};
    col_ind.erase(old_colname);
    col_ind[new_colname] = colind;
  }

  ColNamesConstIterPairType getColNames() const override {
    return std::make_pair(col_ind.cbegin(), col_ind.cend());
  }

  ColNamesIterPairType updColNames() override {
    return std::make_pair(col_ind.begin(), col_ind.end());
  }

  void repColNames(const std::vector<string>& colnames) override {
    col_ind.clear();
    for(size_t i = 0; i < colnames.size(); ++i)
      col_ind[colnames[i]] = i;
  }

  template<typename InputIter>
  void repColNames(InputIter begin, InputIter end) {
    col_ind.clear();
    size_t i = 0;
    while(begin != end)
      col_ind[*begin++] = i++;
  }
    
  SimTK::RowVectorView_<ET> getRow(size_t row) const {
    return data.row(row);
  }

  SimTK::RowVectorView_<ET> updRow(size_t row) {
    return data.updRow(row);
  }

  SimTK::VectorView_<ET> getColumn(size_t col) const {
    return data.col(col);
  }

  SimTK::VectorView_<ET> updColumn(size_t col) {
    return data.updCol(col);
  }

  const ET& getElt(size_t row, size_t col) const {
    return data.getElt(row, col);
  }

  ET& updElt(size_t row, size_t col) {
    return data.updElt(row, col);
  }

  SimTK::Matrix_<ET> getAsMatrix() const {
    return new SimTK::Matrix_<ET>{data};
  }

  /// Append a row of data as a RowVector to the table.
  void appendRow(const SimTK::RowVector_<ET>& row) {
    // ncols specified by the table unless it is zero, in which
    // case allow the first row appended to dictate its size
    int ncols = data.ncol() == 0 ? row.size() : data.ncol();
    SimTK_ASSERT_ALWAYS(row.size() == ncols, "DataTable::appendRow() "
			"row length does match number of columns.");
    SimTK_ASSERT_ALWAYS(row.size() > 0, "DataTable::appendRow() "
			"row is empty.");
    data.resizeKeep(data.nrow() + 1, ncols);
    data[data.nrow() - 1].updAsRowVector() = row;
  }

  /** Append another data table's rows to this table. If the number of
      columns are in compatible it will throw and exception. */
  void appendDataTable(const DataTable_& table) {
    size_t nrows = table.getNumRows();
    if (getNumCols() != table.getNumCols()) {
      throw InvalidEntry("DataTable::appendDataTable() cannot append a "
			 " DataTable with a different number of columns.");
    }
    // resize once to tack on the new table
    size_t offset = data.nrow();
    data.resizeKeep(int( offset + nrows), int(getNumCols()) );
    for (size_t i = 0; i < nrows; ++i){
      data[int(i + offset)].updAsRowVector() = table.getRow(i);
    }
  }

private:
  /// Meta-data.
  MetaDataType       metadata;
  /// Matrix of data. This excludes timestamp column.
  SimTK::Matrix_<ET> data;
  /// Column name to column index.
  ColNamesType       col_ind;
};  // DataTable_

#endif //OPENSIM_DATA_TABLE_H_
