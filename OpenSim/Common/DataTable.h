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

  /// Enum to specify if the the input iterator is traversing data row-wise or
  /// col-wise.
  /// clang3.6 crashes if this is turned to a "enum class"
  enum InputItDir{
    ROWWISE, 
    COLWISE
  };

  // Bind two DataTables by row.
  template<typename ET>
  DataTable_<ET> rbindDataTables(const DataTable_<ET>& dt1, 
                                 const DataTable_<ET>& dt2);

  // Bind two DataTables by col.
  template<typename ET>
  DataTable_<ET> cbindDataTables(const DataTable_<ET>& dt1, 
                                 const DataTable_<ET>& dt2);

  // Exceptions.
  class ColumnDoesNotExist;
  class ColumnHasNoLabel;
  class ZeroElements;
  class InvalidEntry;
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
  virtual ~AbstractDataTable() = default;

  virtual std::unique_ptr<AbstractDataTable> clone() const = 0;
}; // AbstractDataTable


class OpenSim::ColumnDoesNotExist : public std::runtime_error {
public:
  ColumnDoesNotExist(const std::string& expl) : runtime_error(expl) {}
};

class OpenSim::ColumnHasNoLabel : public std::runtime_error {
public:
  ColumnHasNoLabel(const std::string& expl) : runtime_error(expl) {}
};

class OpenSim::ZeroElements : public std::runtime_error {
public:
  ZeroElements(const std::string& expl) : runtime_error(expl) {}
};

class OpenSim::InvalidEntry : public std::runtime_error {
public:
  InvalidEntry(const std::string& expl) : runtime_error(expl) {}
};


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
// * All throw statements are limited to Debug builds. 
// * Why not use C++14 ?
// * Have all the exceptions thrown from DataTable to be in the namespace
//   OpenSim.
// * Let me know if I have missed any test.
// * Are the virtual constructors useful here ? The base class does not have
//   any other functions.
// * We should probably add initializer list constructors, move constructors and
//   iterator constructors to SimTK::Matrix and its derivatives.
// * Is it possible to make getRow give a compile time error if the target is
//   not a constant variable.
// * Default constructing a RowVector gives 1x1 matrix but default constructing
//   a Vector gives 0x1 matrix ?
// * Any useful methods I have missed ?
template<typename ET = SimTK::Real> // Element datatype.
class OpenSim::DataTable_ : public OpenSim::AbstractDataTable {
public:
  using size_t                     = std::size_t;
  using string                     = std::string;
  using MetaDataType               = std::unordered_map<string, 
                                                        SimTK::AbstractValue&>;
  using ColLabelsType              = std::unordered_map<string, size_t>;
  using ColLabelsIterType          = ColLabelsType::iterator;
  using ColLabelsConstIterType     = ColLabelsType::const_iterator;
  using ColLabelsIterPairType      = std::pair<ColLabelsIterType,
                                               ColLabelsIterType>;
  using ColLabelsConstIterPairType = std::pair<ColLabelsConstIterType,
                                               ColLabelsConstIterType>;

  /// Construct an empty DataTable.
  DataTable_() :
    data{}, metadata{}, col_ind{} {}

  /// Construct a pre-sized DataTable populated with given value val. Default
  /// value for val is NaN.
  DataTable_(size_t nrows,
	     size_t ncols,
	     const ET& val = ET{SimTK::NaN}) :
    data{int(nrows), int(ncols), val}, metadata{}, col_ind{} {}

  /// Construct DataTable using an `std::input_iterator`(or its derivative). 
  template<typename InputIt>
  DataTable_(InputIt first,
	     typename std::enable_if<!std::is_integral<InputIt>::value, 
                                     InputIt>::type last,
	     const size_t ndir,
	     const InputItDir dir = ROWWISE) :
    data{static_cast<int>(dir == ROWWISE ? 1    : ndir), 
         static_cast<int>(dir == ROWWISE ? ndir :    1)},
    metadata{},
    col_ind{} {
    if(!(first != last))
      throw OpenSim::ZeroElements{"Input iterators produce zero elements."};

    int row{0};
    int col{0};
    while(first != last) {
      data.set(row, col, *first);
      ++first;
      if(dir == ROWWISE) {
        ++col;
        if(col == static_cast<int>(ndir)  && first != last) {
          col = 0;
          ++row;
          data.resizeKeep(data.nrow() + 1, data.ncol());
        }
      } else {
        ++row;
        if(row == static_cast<int>(ndir) && first != last) {
          row = 0;
          ++col;
          data.resizeKeep(data.nrow(), data.ncol() + 1);
        }
      }
    }
  }

  DataTable_(const DataTable_& dt) :
    data{dt.data}, metadata{dt.metadata}, col_ind{dt.col_ind} {}

  std::unique_ptr<AbstractDataTable> clone() const override {
    return std::unique_ptr<AbstractDataTable>(new DataTable_{*this});
  }

  DataTable_(DataTable_&& dt) :
    data{std::move(dt.data)},
    metadata{std::move(dt.metadata)},
    col_ind{std::move(dt.col_ind)} {}

  virtual ~DataTable_() {}

  DataTable_& operator=(const DataTable_& dt) {
    metadata = dt.metadata;
    data = dt.data ;
    col_ind = dt.col_ind;
    return *this;
  }

  size_t getNumRows() const {
    return static_cast<size_t>(data.nrow()); 
  }

  size_t getNumCols() const {
    return static_cast<size_t>(data.ncol()); 
  }

  SimTK::RowVectorView_<ET> getRow(const size_t row) const {
    return data.row(static_cast<int>(row));
  }

  SimTK::RowVectorView_<ET> updRow(const size_t row) {
    return data.updRow(static_cast<int>(row));
  }

  SimTK::VectorView_<ET> getCol(const size_t col) const {
    return data.col(static_cast<int>(col));
  }

  SimTK::VectorView_<ET> getCol(const string& collabel) const {
    try {
      return data.col(static_cast<int>(col_ind.at(collabel)));
    } catch (std::out_of_range exc) {
      std::string expl{"Column label '" + collabel + "' does not exist."};
      throw OpenSim::ColumnDoesNotExist{expl};
    }
  }

  SimTK::VectorView_<ET> updCol(const size_t col) {
    return data.updCol(static_cast<int>(col));
  }

  SimTK::VectorView_<ET> updCol(const string& collabel) {
    try {
      return data.updCol(static_cast<int>(col_ind.at(collabel)));
    } catch (std::out_of_range exc) {
      std::string expl{"Column label '" + collabel + "' does not exist."};
      throw OpenSim::ColumnDoesNotExist{expl};
    }
  }

  const ET& getElt(const size_t row, const size_t col) const {
    return data.getElt(static_cast<int>(row), static_cast<int>(col));
  }

  const ET& getElt(const size_t row, const string& collabel) const {
    try {
      return data.getElt(static_cast<int>(row), 
                         static_cast<int>(col_ind.at(collabel)));
    } catch (std::out_of_range exc) {
      std::string expl{"Column label '" + collabel + "' does not exist."};
      throw OpenSim::ColumnDoesNotExist{expl};
    }
  }

  ET& updElt(const size_t row, const size_t col) {
    return data.updElt(static_cast<int>(row), static_cast<int>(col));
  }

  ET& updElt(const size_t row, const string& collabel) {
    try {
      return data.updElt(static_cast<int>(row), 
                         static_cast<int>(col_ind.at(collabel)));
    } catch (std::out_of_range exc) {
      std::string expl{"Column label '" + collabel + "' does not exist."};
      throw OpenSim::ColumnDoesNotExist{expl};
    }
  }

  SimTK::Matrix_<ET> getAsMatrix() const {
    return *(new SimTK::Matrix_<ET>{data});
  }

  /// Append a row of data as a RowVector to the table.
  void addRow(const SimTK::RowVector_<ET>& row) {
    if(row.nrow() == 0 || row.ncol() == 0)
      throw OpenSim::ZeroElements{"Input row has zero length."};
    if(data.ncol() > 0 && row.size() != data.ncol())
      throw InvalidEntry{"Input row has incorrect number of columns."};

    data.resizeKeep(data.nrow() + 1, row.ncol());
    data.updRow(data.nrow() - 1).updAsRowVector() = row;
  }

  template<typename InputIt>
  void addRow(InputIt first, InputIt last, size_t ncol_hint = 2) {
    if(!(first != last))
      throw OpenSim::ZeroElements{"Input iterators produce zero elements."};

    if(data.ncol() > 0) {
      data.resizeKeep(data.nrow() + 1, data.ncol());
      int colind = 0;
      while(first != last) {
        data.set(data.nrow() - 1, colind++, *first);
        ++first;
      }
    } else {
      int colind{0};
      size_t ncol{ncol_hint};
      data.resizeKeep(1, static_cast<int>(ncol));
      while(first != last) {
        data.set(0, colind++, *first);
        ++first;
        if(colind == static_cast<int>(ncol) && first != last) {
          // If ncol is a power of 2, double it. Otherwise round it to the next
          // power of 2.
          ncol = (ncol & (ncol - 1)) == 0 ? ncol << 2 : rndToNextPowOf2(ncol); 
          data.resizeKeep(1, static_cast<int>(ncol));
        }
      }
      if(colind != static_cast<int>(ncol))
        data.resizeKeep(1, colind);
    }
  }

  void addCol(const SimTK::Vector_<ET>& col) {
    if(col.nrow() == 0 || col.ncol() == 0)
      throw OpenSim::ZeroElements{"Input column has zero length."};
    if(data.nrow() > 0 && col.size() != data.nrow())
      throw InvalidEntry{"Input column has incorrect number of rows."};

    data.resizeKeep(col.size(), data.ncol() + 1);
    data.updCol(data.ncol() - 1).updAsVector() = col;
  }

  template<typename InputIt>
  void addCol(InputIt first, InputIt last, size_t nrow_hint = 2) {
    if(!(first != last))
      throw OpenSim::ZeroElements{"Input iterators produce zero elements."};

    if(data.nrow() > 0) {
      data.resizeKeep(data.nrow(), data.ncol() + 1);
      int rowind = 0;
      while(first != last) {
        data.set(rowind++, data.ncol() - 1, *first);
        ++first;
      }
    } else {
      int rowind{0};
      size_t nrow{nrow_hint};
      data.resizeKeep(static_cast<int>(nrow), 1);
      while(first != last) {
        data.set(rowind++, 0, *first);
        ++first;
        if(rowind == static_cast<int>(nrow) && first != last) {
          // If nrow is a power of 2, double it. Otherwise round it to the next
          // power of 2.
          nrow = (nrow & (nrow - 1)) == 0 ? nrow << 2 : rndToNextPowOf2(nrow); 
          data.resizeKeep(static_cast<int>(nrow), 1);
        }
      }
      if(rowind != static_cast<int>(nrow))
        data.resizeKeep(rowind, 1);
    }
  }

  void rbindDataTable(const DataTable_& table) {
    if(data.ncol() != table.data.ncol()) 
      throw InvalidEntry{"Input DataTable has incorrect number of columns."};
    if(&data == &table.data)
      throw InvalidEntry{"Cannot rbind a DataTable to itself."};

    int old_nrow{data.nrow()};
    data.resizeKeep(data.nrow() + table.data.nrow(), data.ncol());
    data.updBlock(old_nrow         ,           0, 
                  table.data.nrow(), data.ncol()).updAsMatrix() = table.data;
  }

  void cbindDataTable(const DataTable_& table) {
    if(data.nrow() != table.data.nrow())
      throw InvalidEntry{"Input DataTable has incorrect number of rows."};
    if(&data == &table.data)
      throw InvalidEntry{"Cannot cbind a DataTable to itself."};

    int old_ncol{data.ncol()};
    data.resizeKeep(data.nrow(), data.ncol() + table.data.ncol());
    data.updBlock(0          ,          old_ncol,
                  data.nrow(), table.data.ncol()).updAsMatrix() = table.data;
  }

  void clearData() {
    data.clear();
  }

  /*---------------------------------------------------------------------------*
  * Meta-data accessors & mutators.                                            *
  *---------------------------------------------------------------------------*/

  template<typename ValueType>
  void insertMetaData(const string& key, const ValueType& value) {
    metadata.insert(std::make_pair(key, SimTK::Value<ValueType>{value}));
  }
  
  template<typename ValueType>
  void insertMetaData(const string& key, ValueType&& value) {
    metadata.insert(std::make_pair(key, 
                                   SimTK::Value<ValueType>{std::move(value)}));
  }

  template<typename ValueType>
  void insertMetaData(const std::pair<string, ValueType>& key_value) {
    metadata.insert(std::make_pair(key_value.first, 
                                   SimTK::Value<ValueType>{key_value.second}));
  }

  template<typename ValueType>
  void insertMetaData(std::pair<string, ValueType>&& key_value) {
    using namespace std;
    metadata.insert(make_pair(move(key_value.first), 
                              move(SimTK::Value<ValueType>{key_value.second})));
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
    const SimTK::AbstractValue& abs_value{metadata.at(key)};
    return abs_value.getValue<ValueType>();
  }

  template<typename ValueType>
  ValueType& updMetaData(const string& key) {
    const SimTK::AbstractValue& abs_value{metadata.at(key)};
    return abs_value.updValue<ValueType>();
  }

  /// The the value will be moved from the meta-data container to the caller.
  template<typename ValueType>
  ValueType popMetaData(const string& key) {
    ValueType value{std::move(metadata.at(key))};
    metadata.erase(key);
    return value;
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

  bool colHasLabel(const size_t colind) {
    using ColLabelsValueType = typename ColLabelsType::value_type;
    auto res = std::find_if(col_ind.begin(), 
			    col_ind.end(), 
			    [colind] (const ColLabelsValueType& kv) {
			      return kv.second == colind;
			    });
    return res == col_ind.end();
  }

  bool colExists(const size_t colind) {
    return colind >= 0 && colind < data.ncol();
  }

  bool colExists(const string& key) {
    return col_ind.find(key) != col_ind.end();
  }

  void insertColLabel(const size_t colind, const string& collabel) {
    checkColExistsAndHasLabel(colind);
    col_ind.insert(std::make_pair(collabel, colind));
  }

  void insertColLabel(const size_t colind, string&& collabel) {
    checkColExistsAndHasLabel(colind);
    col_ind.insert(std::make_pair(std::move(collabel), colind));
  }

  /// Input iterators can be either lvalue or rvalue iterators.
  template<typename InputIt>
  void insertColLabels(InputIt first, InputIt last) {
    size_t colind{0};
    while(first != last) {
      checkColExistsAndHasLabel(colind++);
      col_ind.insert(std::make_pair(first->second, first->first));
      ++first;
    }
  }
  
  void setColLabel(const size_t colind, const string& new_collabel) {
    string old_collabel{getColLabel(colind)};
    col_ind.erase(old_collabel);
    col_ind.insert(std::make_pair(new_collabel, colind));
  }

  void setColLabel(const string& old_collabel, 
                   const string& new_collabel) {
    size_t colind{col_ind.at(old_collabel)};
    col_ind.erase(old_collabel);
    col_ind[new_collabel] = colind;
  }

  void setColLabels(const std::vector<string>& collabels) {
    col_ind.clear();

    for(size_t colind = 0; colind < collabels.size(); ++colind) {
      checkColExists(colind);
      col_ind[collabels[colind]] = colind;
    }
  }

  template<typename InputIter>
  void setColLabels(InputIter begin, InputIter end) {
    col_ind.clear();
    insertColLabels(begin, end);
  }

  string getColLabel(const size_t colind) const {
    checkColExists(colind);
    using ColLabelsValueType = typename ColLabelsType::value_type;
    auto res = std::find_if(col_ind.begin(),
			    col_ind.end(),
			    [colind] (const ColLabelsValueType& kv) {
			      return kv.second == colind;
			    });
    if(res == col_ind.end()) {
      throw OpenSim::ColumnHasNoLabel{"Column " + std::to_string(colind) + 
                                      " has no label."};
    }

    return res->first;
  }

  ColLabelsConstIterPairType getColLabels() const {
    return std::make_pair(col_ind.cbegin(), col_ind.cend());
  }

  ColLabelsIterPairType updColLabels() {
    return std::make_pair(col_ind.begin(), col_ind.end());
  }

  size_t getColInd(const string& collabel) const {
    return col_ind.at(collabel);
  }

private:
  void checkColExists(const size_t colind) {
    if(!colExists(colind)) {
      throw OpenSim::ColumnDoesNotExist{"Column " + std::to_string(colind) + 
                                        " does not exist."};
    }
  }

  void checkColHasLabel(const size_t colind) {
    if(colHasLabel(colind)) {
      throw InvalidEntry{"Column " + std::to_string(colind) + 
                         " already has a label."};
    }
  }

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

  /// Matrix of data. This excludes timestamp column.
  SimTK::Matrix_<ET> data;
  /// Meta-data.
  MetaDataType       metadata;
  /// Column label to column index.
  ColLabelsType      col_ind;
};  // DataTable_


template<typename ET>
OpenSim::DataTable_<ET> 
OpenSim::rbindDataTables(const OpenSim::DataTable_<ET>& dt1, 
                         const OpenSim::DataTable_<ET>& dt2) {
  OpenSim::DataTable_<ET> dt{dt1};
  dt.rbindDataTable(dt2);
  return dt;
}


template<typename ET>
OpenSim::DataTable_<ET> 
OpenSim::cbindDataTables(const OpenSim::DataTable_<ET>& dt1, 
                         const OpenSim::DataTable_<ET>& dt2) {
  OpenSim::DataTable_<ET> dt{dt1};
  dt.cbindDataTable(dt2);
  return dt;
}

#endif //OPENSIM_DATA_TABLE_H_
