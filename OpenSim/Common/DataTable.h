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
  class AbstractDataTable;
  template<typename ET> class DataTable_;

  using DataTable = DataTable_<SimTK::Real>;
} // namespace OpenSim


/**----------------------------------------------------------------------------*
* AbstractDataTable defines an interface for an in-memory numerical table data *
* structure -- DataTable. A DataTable is independent of the data source used   *
* to populate it. A concrete DataTable provides a random access to data        *
* elements by row and/or column indices as well as column by column-name.      *
* This class allows storing different DataTables in containers.                *
*                                                                              *
* @author Ajay Seth                                                            *
*-----------------------------------------------------------------------------*/
class OpenSim::AbstractDataTable {
public:
  using size_t                     = std::size_t;
  using string                     = std::string;
  using MetaDataType               = std::unordered_map<string, AbstractValue&>;
  using ColLabelsType              = std::unordered_map<string, size_t>;
  using ColLabelsIterType          = ColLabelsType::iterator;
  using ColLabelsConstIterType     = ColLabelsType::const_iterator;
  using MetaDataKVType             = std::pair<string, string>;
  using ColLabelsIterPairType      = std::pair<ColLabelsIterType,
                                               ColLabelsIterType>;
  using ColLabelsConstIterPairType = std::pair<ColLabelsConstIterType,
                                               ColLabelsConstIterType>;

  virtual ~AbstractDataTable() = default;

  virtual AbstractDataTable& clone() const = 0;

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
}; // AbstractDataTable


class ColumnDoesNotExist : public std::runtime_error {
public:
  ColumnDoesNotExist(const std::string& expl) : runtime_error(expl) {}
};

class ColumnHasNoLabel : public std::runtime_error {
public:
  ColumnHasNoLabel(const std::string& expl) : runtime_error(expl) {}
}

class ZeroElements : public std::runtime_error {
public:
  ZeroElements(const std::string& expl) : runtime_error(expl) {}
};

class InvalidEntry : public std::runtime_error {
public:
  InvalidEntry(const std::string& expl) : runtime_error(expl) {}
};


// * Change metadata from unordered map of string to string -->> string to
//   type erased object.
// * Change colNames to colLabels.
// * have an overload for getting columns by name.
// * Add a method to populate a default constructed DataTable.

/**----------------------------------------------------------------------------*
* DataTable_ is a concrete implementation of AbstractDataTable interface.      *
* Concrete implementation of AbstractDataTable interface. The element type is  *
* specified through the template argument.                                     *
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
// * Error checking disabled during non-Debug builds. Is that okay ?
template<typename ET = SimTK::Real> // Element datatype.
class OpenSim::DataTable_ : public OpenSim::AbstractDataTable {
public:
  /// Enum to specify if the the input iterator is traversing data row-wise or
  /// col-wise.
  enum InputItDir {
    Row,
    Col
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
#ifndef NDEBUG
    if(!(first != last))
      throw ZeroElements{"Input iterators produce zero elements."};
#endif

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

  size_t getNumRows() const override {
    return data.nrow(); 
  }

  size_t getNumCols() const override {
    return data.ncol(); 
  }

  SimTK::RowVectorView_<ET> getRow(const size_t row) const {
    return data.row(row);
  }

  SimTK::RowVectorView_<ET> updRow(const size_t row) {
    return data.updRow(row);
  }

  SimTK::VectorView_<ET> getCol(const size_t col) const {
    return data.col(col);
  }

  SimTK::VectorView_<ET> getCol(const string& collabel) const {
    return data.col(col_ind.at(collabel));
  }

  SimTK::VectorView_<ET> updCol(const size_t col) {
    return data.updCol(col);
  }

  SimTK::VectorView_<ET> updCol(const string& collabel) {
    return data.updCol(col_ind.at(collabel));
  }

  const ET& getElt(const size_t row, const size_t col) const {
    return data.getElt(row, col);
  }

  const ET& getElt(const size_t row, const string& collabel) const {
    return data.getElt(row, col_ind.at(collabel));
  }

  ET& updElt(const size_t row, const size_t col) {
    return data.updElt(row, col);
  }

  ET& updElt(const size_t row, const string& collabel) {
    return data.updElt(row, col_ind.at(collabel));
  }

  SimTK::Matrix_<ET> getAsMatrix() const {
    return new SimTK::Matrix_<ET>{data};
  }

  /// Append a row of data as a RowVector to the table.
  void addRow(const SimTK::RowVector_<ET>& row) {
#ifndef NDEBUG
    if(row.size() == 0)
      throw ZeroElements{"Input row has zero length."};
    if(data.ncol() > 0 && row.size() != data.ncol())
      throw InvalidEntry{"Input row has incorrect number of columns."};
#endif
    data.resizeKeep(data.nrow() + 1, data.ncols());
    data.updRow(data.nrow() - 1).updAsRowVector() = row;
  }

  template<InputIt>
  void addRow(InputIt first, InputIt last, size_t ncol_hint = 2) {
#ifndef NDEBUG
    if(!(first != last))
      throw ZeroElements{"Input iterators produce zero elements."};
#endif

    if(data.ncol() > 0) {
      data.resizeKeep(data.nrow() + 1, data.ncol());
      size_t col = 0;
      while(first != last)
        data.set(data.nrow() - 1, col++, *first);
    } else {
      size_t colind{0}
      size_t ncol{ncol_hint};
      data.resizeKeep(1, ncol);
      while(first != last) {
        data.set(1, colind++, *first);
        if(colind == ncol && first != last) {
          // If ncol is a power of 2, double it. Otherwise round it to the next
          // power of 2.
          ncol = (ncol & (ncol - 1)) == 0 ? ncol << 2 : rndToNextPowOf2(ncol); 
          data.resizeKeep(1, ncol);
        }
      }
      if(colind != ncol)
        data.resizeKeep(1, colind);
    }
  }

  void addCol(const SimTK::Vector_<ET>& col) {
#ifndef NDEBUG
    if(col.size() == 0)
      throw ZeroElements{"Input column has zero length."};
    if(data.nrow() > 0 && col.size() != data.nrow())
      throw InvalidEntry{"Input column has incorrect number of rows."};
#endif
    data.resizeKeep(col.size(), data.ncol() + 1);
    data.updCol(data.ncol() - 1).updAsVector() = col;
  }

  template<InputIt>
  void addCol(InputIt first, InputIt last, nrow_hint = 2) {
#ifndef NDEBUG
    if(!(first != last))
      throw ZeroElements{"Input iterators produce zero elements."};
#endif

    if(data.nrow() > 0) {
      data.resizeKeep(data.nrow(), data.ncol() + 1);
      size_t row = 0;
      while(first != last)
        data.set(row++, data.nrow() - 1, *first);
    } else {
      size_t rowind{0}
      size_t nrow{nrow_hint};
      data.resizeKeep(nrow, 1);
      while(first != last) {
        data.set(rowind++, 1, *first);
        if(rowind == nrow && first != last) {
          // If nrow is a power of 2, double it. Otherwise round it to the next
          // power of 2.
          nrow = (nrow & (nrow - 1)) == 0 ? nrow << 2 : rndToNextPowOf2(nrow); 
          data.resizeKeep(1, nrow);
        }
      }
      if(rowind != nrow)
        data.resizeKeep(rowind, 1);
    }
  }

  void rbindDataTable(const DataTable_& table) {
#ifndef NDEBUG
    if(data.ncol() != table.data.ncol()) {
      throw InvalidEntry{"Input DataTable has incorrect number of columns."};
    }
#endif

    size_t old_nrow{data.nrow()};
    data.resizeKeep(data.nrow() + table.data.nrow(), data.ncol());
    data.updBlock(old_nrow, data.nrow() - 1, 
                  0       , data.ncol() - 1).updAsMatrix = table.data;
  }

  void cbindDataTable(const DataTable_& table) {
#ifndef NDEBUG
    if(data.nrow() != table.data.nrow()) {
      throw InvalidEntry{"Input DataTable has incorrect number of rows."};
    }
#endif

    size_t old_ncol{data.ncol()};
    data.resizeKeep(data.nrow(), data.ncol() + table.data.ncol());
    data.updBlock(0       , data.nrow() - 1,
                  old_ncol, data.ncol() - 1).updAsMatrix = table.data;
  }

  /*---------------------------------------------------------------------------*
  * Meta-data accessors & mutators.                                            *
  *---------------------------------------------------------------------------*/

  template<typename ValueType>
  void insertMetaData(const string& key, const ValueType& value) {
    metadata.insert(std::make_pair(key, Value<ValueType>{value}));
  }
  
  template<typename ValueType>
  void insertMetaData(const string& key, ValueType&& value) {
    metadata.insert(std::make_pair(key, Value<ValueType>{std::move(value)}));
  }

  template<typename ValueType>
  void insertMetaData(const std::pair<string, ValueType>& key_value) {
    metadata.insert(std::make_pair(key_value.first, 
                                   Value<ValueType>{key_value.second}));
  }

  template<typename ValueType>
  void insertMetaData(std::pair<string, ValueType>&& key_value) {
    using namespace std;
    metadata.insert(make_pair(move(key_value.first), 
                              move(Value<ValueType>{key_value.second})));
  }

  /// Works for both lvalues and rvalues.
  template<typename InputIt>
  void insertMetaData(InputIt first, InputIt last) {
    while(first != last) {
      insertMetaData(*first);
      ++first;
    }
  }

  template<typename ValueType>
  const ValueType& getMetaData(const string& key) const {
    return metadata.at(key).getValue<ValueType>();
  }

  template<typename ValueType>
  ValueType& updMetaData(const string& key) {
    return metadata.at(key).updValue<ValueType>();
  }

  /// The the value will be moved from the meta-data container to the caller.
  template<typename ValueType>
  ValueType popMetaData(const string& key) {
    ValueType value{std::move(metadata.at(key))};
    metadata.erase(key);
    return value
  }

  void clearMetaData() {
    metadata.clear();
  }

  bool metaDataExists(const string& key) {
    return metadata.find(key) != metadata.end();
  }

  bool isMetaDataEmpty() {
    return metadata.empty();
  }

  size_t metaDataSize() {
    return metadata.size();
  }

  /*---------------------------------------------------------------------------*
  * Column labels accessors & mutators.                                        *
  *---------------------------------------------------------------------------*/

  bool colHasLabel(const size_t colind) override {
    using ColLabelsValueType = typename ColLabelsType::value_type;
    auto res = std::find_if(col_ind.begin(), 
			    col_ind.end(), 
			    [colind] (const ColLabelsValueType& kv) {
			      return kv.second == colind;
			    });
    return res == col_ind.end();
  }

  bool colExists(const size_t colind) override {
    return colind >= 0 && colind < data.ncol();
  }

  bool colExists(const string& key) override {
    return col_ind.find(key) != col_ind.end();
  }

  void insertColLabel(const size_t colind, const string& collabel) override {
    checkColExistsAndHasLabel(colind);
    col_ind.insert(std::make_pair(collabel, colind));
  }

  void insertColLabel(const size_t colind, string&& collabel) override {
    checkColExistsAndHasLabel(colind);
    col_ind.insert(std::make_pair(std::move(collabel), colind));
  }

  /// Input iterators can be either lvalue or rvalue iterators.
  template<typelabel InputIt>
  void insertColLabels(InputIt first, InputIt last) {
    while(first != last) {
      checkColExistsAndHasLabel(colind);
      col_ind.insert(std::make_pair(first->second, first->first));
      ++first;
    }
  }
  
  void setColLabel(const size_t colind, const string& new_collabel) override {
    checkColExistsAndHasLabel(colind);
    col_ind.erase(res);
    col_ind.insert(std::make_pair(new_collabel));
  }

  void setColLabel(const string& old_collabel, 
                   const string& new_collabel) override {
    size_t colind{col_ind.at(old_collabel)};
    col_ind.erase(old_collabel);
    col_ind[new_collabel] = colind;
  }

  void setColLabels(const std::vector<string>& collabels) override {
    col_ind.clear();

    for(size_t i = 0; i < collabels.size(); ++i) {
      if(!colExists(colind))
        throw ColumnDoesNotExist{"Column index out of range."};

      col_ind[collabels[i]] = i;
    }
  }

  template<typelabel InputIter>
  void setColLabels(InputIter begin, InputIter end) {
    col_ind.clear();

    insertColLabels(begin, end);
  }

  string getColLabel(const size_t colind) const override {
    checkColExists(colind);
    using ColLabelsValueType = typename ColLabelsType::value_type;
    auto res = std::find_if(col_ind.begin(),
			    col_ind.end(),
			    [colind] (const ColLabelsValueType& kv) {
			      return kv.second == colind;
			    });
    if(res == col_ind.end())
      throw ColumnHasNoLabel{""};

    return res->first;
  }

  ColLabelsConstIterPairType getColLabels() const override {
    return std::make_pair(col_ind.cbegin(), col_ind.cend());
  }

  ColLabelsIterPairType updColLabels() override {
    return std::make_pair(col_ind.begin(), col_ind.end());
  }

  size_t getColInd(const string& collabel) const override {
    return col_ind.at(collabel);
  }

private:
  /// Exists only in Debug builds.
  void checkColExists(const size_t colind) {
#ifndef NDEBUG
    if(!colExists(colind))
      throw ColumnDoesNotExist{""};
#endif
  }

  /// Exists only in Debug builds.
  void checkColHasLabel(const size_t colind) {
#ifndef NDEBUG
    if(colHasLabel(colind))
      throw InvalidEntry{"Column already has a label."};
#endif
  }

  /// Exists only in Debug builds.
  void checkColExistsAndHasLabel(const size_t colind) {
    checkColExists(colind);
    checkColHasLabel(colind);
  }

  /// Round to next highest power of 2.
  size_t rndToNextPowOf2(size_t num) {
    --num;
    num |= (num >>  1); // Highest  2 bits are set by end of this.
    num |= (num >>  2); // Highest  4 bits are set by end of this.
    num |= (num >>  4); // Highest  8 bits are set by end of this.
    num |= (num >>  8); // Highest 16 bits are set by end of this.
    num |= (num >> 16); // Highest 32 bits are set by end of this.
    return ++num;
  }

  /// Meta-data.
  MetaDataType       metadata;
  /// Matrix of data. This excludes timestamp column.
  SimTK::Matrix_<ET> data;
  /// Column label to column index.
  ColLabelsType      col_ind;
};  // DataTable_

#endif //OPENSIM_DATA_TABLE_H_
