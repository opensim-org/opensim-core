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
This file defines the  DataTable class, which is used by OpenSim to provide an 
in-memory container for data access and manipulation.                         */

#ifndef OPENSIM_DATA_TABLE_H_
#define OPENSIM_DATA_TABLE_H_

// Standard headers.
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
#include <string>
#include <type_traits>
#include <limits>
// Non-standard headers.
#include <SimTKcommon.h>
#include <OpenSim/Common/Exception.h>


namespace OpenSim {
  class AbstractDataTable;
  template<typename ET> class DataTable_;

  using DataTable = DataTable_<SimTK::Real>;

  /** Enum to specify if the the input iterator is traversing data row-wise or
  col-wise. clang3.6 crashes if this is turned to a "enum class". Until it is 
  fixed, use a pre c++11 enum.                                                */
  enum InputItDir{
    RowWise, 
    ColWise
  };

  /** Bind two DataTables by row.*/
  template<typename ET>
  DataTable_<ET> rbindDataTables(const DataTable_<ET>& dt1, 
                                 const DataTable_<ET>& dt2);

  /** Bind two DataTables by col.*/
  template<typename ET>
  DataTable_<ET> cbindDataTables(const DataTable_<ET>& dt1, 
                                 const DataTable_<ET>& dt2);

  // Exceptions.
  class NotEnoughElements;
  class NumberOfColsMismatch;
  class NumberOfRowsMismatch;
  class ColumnDoesNotExist;
  class ColumnHasLabel;
  class ColumnHasNoLabel;
  class ZeroElements;
  class InvalidEntry;
  class MetaDataKeyExists;
  class MetaDataKeyDoesNotExist;
  class MetaDataTypeMismatch;
} // namespace OpenSim


class OpenSim::NotEnoughElements : public OpenSim::Exception {
public:
  NotEnoughElements(const std::string& expl) : Exception(expl) {}
};

class OpenSim::NumberOfColsMismatch : public OpenSim::Exception {
public:
  NumberOfColsMismatch(const std::string& expl) : Exception(expl) {}
};

class OpenSim::NumberOfRowsMismatch : public OpenSim::Exception {
public:
  NumberOfRowsMismatch(const std::string& expl) : Exception(expl) {}
};

class OpenSim::ColumnDoesNotExist : public OpenSim::Exception {
public:
  ColumnDoesNotExist(const std::string& expl) : Exception(expl) {}
};

class OpenSim::ColumnHasLabel : public OpenSim::Exception {
public:
  ColumnHasLabel(const std::string& expl) : Exception(expl) {}
};

class OpenSim::ColumnHasNoLabel : public OpenSim::Exception {
public:
  ColumnHasNoLabel(const std::string& expl) : Exception(expl) {}
};

class OpenSim::ZeroElements : public OpenSim::Exception {
public:
  ZeroElements(const std::string& expl) : Exception(expl) {}
};

class OpenSim::InvalidEntry : public OpenSim::Exception {
public:
  InvalidEntry(const std::string& expl) : Exception(expl) {}
};

class OpenSim::MetaDataKeyExists : public OpenSim::Exception {
public:
  MetaDataKeyExists(const std::string& expl) : Exception(expl) {}
};

class OpenSim::MetaDataKeyDoesNotExist : public OpenSim::Exception {
public:
  MetaDataKeyDoesNotExist(const std::string& expl) : Exception(expl) {}
};

class OpenSim::MetaDataTypeMismatch : public OpenSim::Exception {
public:
  MetaDataTypeMismatch(const std::string& expl) : Exception(expl) {}
};


/**-----------------------------------------------------------------------------
AbstractDataTable is the base-class of all DataTable_ allowing storage of
DataTable_ of different types to be stored in containers. AbstractDataTable_ 
offers interface to access column labels of DataTable_. See DataTable_ interface
for details on using DataTable_.
------------------------------------------------------------------------------*/
class OpenSim::AbstractDataTable {
public:
  using size_t                     = std::size_t;
  using string                     = std::string;
  using ColLabelsType              = std::unordered_map<string, size_t>;
  using ColLabelsIterType          = ColLabelsType::iterator;
  using ColLabelsConstIterType     = ColLabelsType::const_iterator;
  using ColLabelsIterPairType      = std::pair<ColLabelsIterType,
                                               ColLabelsIterType>;
  using ColLabelsConstIterPairType = std::pair<ColLabelsConstIterType,
                                               ColLabelsConstIterType>;


  AbstractDataTable() = default;
  AbstractDataTable(const AbstractDataTable&) = delete;
  AbstractDataTable(AbstractDataTable&&) = delete;
  virtual std::unique_ptr<AbstractDataTable> clone() const = 0;
  virtual ~AbstractDataTable() {}

  /**---------------------------------------------------------------------------
     Column label acessors & mutators.
  ----------------------------------------------------------------------------*/
  
  /** Check if a column index has label. Time complexity is linear in number of 
  column labels. All columns will have an index. All columns need not have a 
  label.

  \throws OpenSim::ColumnDoesNotExist If col index specified does not exist.  */
  virtual bool colHasLabel(const size_t colind) const = 0;

  /** Check if a column exists by its index.                                  */
  virtual bool colExists(const size_t colind) const = 0;

  /** Check if a column exists under given label. All columns have an index but
  not all columns may have labels.                                            */
  virtual bool colExists(const string& collabel) const = 0;

  /** Label a column. The column should not have a label already. To update the
  label of a column that already has a label, use updColLabel().

  \throws OpenSim::ColumnDoesNotExist If the col index specified does not exist.
  \throws OpenSim::ColumnHasLabel If the column index specified already has a 
                                  label.                                      */
  virtual void insertColLabel(const size_t colind, const string& collabel) = 0;

  /** Label a column. The column should not have a label already. To update the
  label of a column that already has a label, use updColLabel().

  \throws OpenSim::ColumnDoesNotExist If the col index specified does not exist.
  \throws OpenSim::ColumnHasLabel If the column index specified already has a 
                                  label.                                      */
  virtual void insertColLabel(const size_t colind, string&& collabel) = 0;

  /** Get the label of a column. Time complexity is linear in the number of
  column labels. The returned value is a copy of the label. To update the label 
  of a column, use updColLabel(). 

  \throws OpenSim::ColumnHasNoLabel If the column does not have a label.
  \throws OpenSim::ColumnDoesNotExist If the column does not exist.           */
  virtual string getColLabel(const size_t colind) const = 0;

  /** Get all the column labels. Returns an iterator pair(std::pair) where first
  element of pair is the beginning and second element of the pair is the end 
  (sentinel) of the labels. Dereferencing the iterator will produce a pair
  (std::pair) where the first element of the pair is the label and the second 
  element is the column index. To update the column abels, use updColLabels().*/
  virtual ColLabelsConstIterPairType getColLabels() const = 0;

  /** Update the label of a column with a new label. Time complexity is linear
  in the number of column labels. The column specified must already have a
  label. To label a column that does not yet have a label, use insertLabel().

  \throws OpenSim::ColumnHasNoLabel If the column specified does not already
                                    have a label.
  \throws OpenSim::ColumnDoesNotExist If the column specified does not 
                                      exist.                                  */
  virtual void updColLabel(const size_t colind, const string& new_collabel) = 0;

  /** Update the label of a column with a new label. Time complexity is constant
  on average and linear in number of column labels in the worst case.

  \throws OpenSim::ColumnHasNoLabel If the column specified does not already
                                    have a label.
  \throws OpenSim::ColumnDoesNotExist If the column specified does not exist. */
  virtual void updColLabel(const string& old_collabel, 
                           const string& new_collabel) = 0;

  /** Update all the column labels. Returns an iterator pair(std::pair) where
  first element of pair is the beginning and second element of the pair is the 
  end(sentinel) of the labels. Dereferencing the iterator will produce a pair 
  (std::pair) where the first element of the pair is the label and the second 
  element is the column index.                                                */
  virtual ColLabelsIterPairType updColLabels() = 0;

  /** Get the index of a column from its label. Time complexity is constant on
  average and linear in number of column labels on worst case.

  \throws OpenSim::ColumnDoesNotExist If the column label does not exist.     */
  virtual size_t getColInd(const string& collabel) const = 0;

  /** Clear all the column labels.                                            */
  virtual void clearColLabels() = 0;
}; // AbstractDataTable


/**-----------------------------------------------------------------------------
\brief DataTable_ is a in-memory storage container for data(in the form of a
matrix with column names) with support for holding metatdata.                
                                                                              
<ul> <li>Underlying matrix will have entries of configurable type ET(template 
param).</li>  <li>Random-access(constant-time) to specific entries, entire 
columns and entire rows using their index.</li> <li>Average constant-time 
access to columns through column-labels.</li> <li>Add rows and columns to 
existing DataTable_.</li> <li>Bind two DataTable_(s) by row and by column. </li>
 <li>Set column labels for a subset of columns, update them, remove them etc.
</li> <li>Construct DataTable_ emtpy OR with a given shape and default value OR 
using and iterator pair one entry at a time.</li> <li>Heterogeneous metadata 
container. Metadata in the form of key-value pairs where key is a std::string 
and value is is of any type.</li> </ul>
                                                                              
\tparam ET Type of the entries in the underlying matrix. Defaults to         
           SimTK::Real(alias for double).
------------------------------------------------------------------------------*/
template<typename ET = SimTK::Real> // Element datatype.
class OpenSim::DataTable_ : public OpenSim::AbstractDataTable {
public:
  using MetaDataType               = std::unordered_map<string, 
                                                        SimTK::AbstractValue*>;

  /** Construct empty DataTable.*/
  DataTable_() :
    m_data{}, m_metadata{}, m_col_ind{} {}

  /** Construct DataTable of given shape populated with given value val. Default
  value for val is NaN.
  
  \param nrows Number of rows.
  \param ncols Number of columns.
  \param val   Value to initialize all the entries to.                        */
  DataTable_(size_t nrows,
             size_t ncols,
             const ET& val = ET{SimTK::NaN}) :
    m_data{int(nrows), int(ncols), val}, m_metadata{}, m_col_ind{} {}

  /** Construct DataTable using an iterator(satisfying requirement of an 
  input_iterator) which produces one entry at a time. The entries of DataTable_
  are copy initialized using the values produced by the iterator. For example, 
  specifying RowWise for parameter dir and 10 for parameter ndir will populate 
  the DataTable one row at a time with each row's length taken to be 10.
      
  \param first Beginning of range covered by the iterator.
  \param last End of the range covered by the iterator.
  \param ndir Extent of the dimention specified by parameter dir. 
  \param dir Dimension to populate the DataTable. Populate row-wise or column 
             wise. See OpenSim::InputItDir for possible values.
  \param allow_missing Allow for missing values. When set to false, this
                       function will throw if the input iterator fills up the 
                       last row/col partially. When set to true, missing 
                       elements are set to SimTK::NaN.
  
  \throws ZeroElements When input-iterator does not produce any elements.
                       That is first == last.                                 
  \throws InvalidEntry When the required input argument 'ndir' is zero.
  \throws NotEnoughElements If dir == RowWise, this is thrown when the input 
                            iterator does not produce enough elements to fill 
                            up the last row completely. If dir == ColWise, this
                            is thrown when the input iterator does not produce 
                            enough elements to fill up the last col 
                            completely.                                       */
  template<typename InputIt>
  DataTable_(InputIt first,
             typename std::enable_if<!std::is_integral<InputIt>::value,
                                     InputIt>::type last,
             const size_t ndir,
             const InputItDir dir = RowWise,
             const bool allow_missing = false) :
    m_data{static_cast<int>(dir == RowWise ? 1    : ndir), 
           static_cast<int>(dir == RowWise ? ndir :    1)},
    m_metadata{},
    m_col_ind{} {
    if(!(first != last))
      throw ZeroElements{"Input iterators produce zero elements."};
    if(ndir == 0)
      throw InvalidEntry{"Input argument 'ndir' cannot be zero."};

    int row{0};
    int col{0};
    while(first != last) {
      m_data.set(row, col, *first);
      ++first;
      if(dir == RowWise) {
        ++col;
        if(col == static_cast<int>(ndir)  && first != last) {
          col = 0;
          ++row;
          m_data.resizeKeep(m_data.nrow() + 1, m_data.ncol());
        }
      } else {
        ++row;
        if(row == static_cast<int>(ndir) && first != last) {
          row = 0;
          ++col;
          m_data.resizeKeep(m_data.nrow(), m_data.ncol() + 1);
        }
      }
    }
    if(!allow_missing) {
      if(dir == RowWise && col != m_data.ncol()) {
        throw NotEnoughElements{"Input iterator did not produce enough elements"
                                " to fill the last row. Expected = " + 
                                std::to_string(m_data.ncol()) + 
                                " Received = " + std::to_string(col)};
      } else if(dir == ColWise && row != m_data.nrow()) {
        throw NotEnoughElements{"Input iterator did not produce enough elements"
                                " to fill the last column. Expected = " +
                                std::to_string(m_data.nrow()) + 
                                " Received = " + std::to_string(row)};
      }
    }
  }

  /** Copy constructor. Perform a deep copy of the DataTable_.*/
  DataTable_(const DataTable_& dt) :
    m_data{dt.m_data}, m_metadata{dt.m_metadata}, m_col_ind{dt.m_col_ind} {
      // Deep copy of metadata.
      for(auto& elem : m_metadata)
        elem.second = elem.second->clone();
    }

  /** Virtual copy constructor. Perform a deep copy of the DataTable_.*/
  std::unique_ptr<AbstractDataTable> clone() const override {
    return std::unique_ptr<AbstractDataTable>(new DataTable_{*this});
  }

  /** Copy assignment operator. Perform a deep copy of the DataTable_.*/
  DataTable_& operator=(const DataTable_& dt) {
    // Release metadata memory.
    for(auto& elem : m_metadata)
      delete elem.second;

    m_data     = dt.m_data;
    m_metadata = dt.m_metadata;
    m_col_ind  = dt.m_col_ind;

    // Deep copy of metadata.
    for(auto& elem : m_metadata)
      elem.second = elem.second->clone();

    return *this;
  }

  /** Move constructor.*/
  DataTable_(DataTable_&& dt) = default;

  DataTable_& operator=(DataTable_&& dt) {
    // Release metadata memory.
    for(auto& elem : m_metadata)
      delete elem.second;

    m_data     = std::move(dt.m_data);
    m_metadata = std::move(dt.m_metadata);
    m_col_ind  = std::move(dt.m_col_ind);

    return *this;
  }

  /** Destructor.*/
  virtual ~DataTable_() {
    // Release metadata memory.
    for(auto& elem : m_metadata)
      delete elem.second;
  }

  /** Get number of rows in the DataTable_.*/
  size_t getNumRows() const {
    return static_cast<size_t>(m_data.nrow()); 
  }

  /** Get number of columns in the DataTable_.*/
  size_t getNumCols() const {
    return static_cast<size_t>(m_data.ncol()); 
  }

  /** Get a row of the DataTable_ by index. Returned row is read-only. Use 
  updRow to obtain a writable reference. See SimTK::RowVectorView_ for more
  details.
  
  \throws SimTK::Exception::IndexOutOfRange If the row index specified is not 
                                            in the DataTable_.                */
  SimTK::RowVectorView_<ET> getRow(const size_t row) const {
    return m_data.row(static_cast<int>(row));
  }

  /** Get a row of the DataTable_ by index. Returned row is editable. See 
  SimTK::RowVectorView_ for more details.
  
  \throws SimTK::Exception::IndexOutOfRange If the row index specified is not 
                                            in the DataTable_.                */
  SimTK::RowVectorView_<ET> updRow(const size_t row) {
    return m_data.updRow(static_cast<int>(row));
  }

  /** Get a col of the DataTable_ by index. Returned col is read-only. Use 
  updCol to obtain a writable reference. See SimTK::VectorView_ for more 
  details.
  
  \throws SimTK::Exception::IndexOutOfRange If the col index specified is not 
                                            in the DataTable_.                */
  SimTK::VectorView_<ET> getCol(const size_t col) const {
    return m_data.col(static_cast<int>(col));
  }

  /** Get a col of the DataTable_ by label. Returned col is read-only. Use 
  updCol to obtain a writable reference. See SimTK::VectorView_ for more 
  details.
      
  \throws ColumnDoesNotExist If the col label specified is not in the 
                             DataTable_.                                      */
  SimTK::VectorView_<ET> getCol(const string& collabel) const {
    try {
      return m_data.col(static_cast<int>(m_col_ind.at(collabel)));
    } catch (std::out_of_range exc) {
      throw ColumnDoesNotExist{"Column label '" + collabel + 
                               "' does not exist."};
    }
  }

  /** Get a col of the DataTable_ by index. Returned col is editable. See 
  SimTK::VectorView_ for more details.
  
  \throws SimTK::Exception::IndexOutOfRange If the col index specified is not 
                                            in the DataTable_.                */
  SimTK::VectorView_<ET> updCol(const size_t col) {
    return m_data.updCol(static_cast<int>(col));
  }

  /** Get a col of the DataTable_ by label. Returned col is editable. See 
  SimTK::VectorView_ for more details.
  
  \throws ColumnDoesNotExist If the col label specified is not in the 
                             DataTable_.                                      */
  SimTK::VectorView_<ET> updCol(const string& collabel) {
    try {
      return m_data.updCol(static_cast<int>(m_col_ind.at(collabel)));
    } catch (std::out_of_range exc) {
      throw ColumnDoesNotExist{"Column label '" + collabel + 
                               "' does not exist."};
    }
  }

  /** Get an element of the DataTable_ by its index-pair(row, col). The returned
  element is read-only. use updElt to get a writable reference.
  
  \throws SimTK::Exception::IndexOutOfRange If the row/col index specified is 
                                            not in the DataTable_.            */
  const ET& getElt(const size_t row, const size_t col) const {
    return m_data.getElt(static_cast<int>(row), static_cast<int>(col));
  }

  /** Get an element of the DataTable_ by (row, col-label) pair. The returned
  element is read-only. use updElt to get a writable reference.
   
  \throws SimTK::Exception::IndexOutOfRange If the row index specified is not 
                                            in the DataTable_.
  \throws ColumnDoesNotExist If the col label specified is not in the 
                             DataTable_.                                      */
  const ET& getElt(const size_t row, const string& collabel) const {
    try {
      return m_data.getElt(static_cast<int>(row), 
                           static_cast<int>(m_col_ind.at(collabel)));
    } catch (std::out_of_range exc) {
      throw ColumnDoesNotExist{"Column label '" + collabel + 
                               "' does not exist."};
    }
  }

  /** Get an element of the DataTable_ by its index-pair(row, col). The returned
  element is editable.
  
  \throws SimTK::Exception::IndexOutOfRange If the row/col index specified is 
                                            not in the DataTable_.            */
  ET& updElt(const size_t row, const size_t col) {
    return m_data.updElt(static_cast<int>(row), static_cast<int>(col));
  }

  /** Get an element of the DataTable_ by (row, col-label) pair. The returned
  element is editable.

  \throws SimTK::Exception::IndexOutOfRange If the row index specified is not 
                                            in the DataTable_.
  \throws ColumnDoesNotExist If the col label specified is not in the 
                             DataTable_.                                      */
  ET& updElt(const size_t row, const string& collabel) {
    try {
      return m_data.updElt(static_cast<int>(row), 
                           static_cast<int>(m_col_ind.at(collabel)));
    } catch (std::out_of_range exc) {
      throw ColumnDoesNotExist{"Column label '" + collabel + 
                               "' does not exist."};
    }
  }

  /** Get a *copy* of the underlying matrix of the DataTable_.                */
  std::unique_ptr<SimTK::Matrix_<ET>> getAsMatrix() const {
    return std::unique_ptr<SimTK::Matrix_<ET>>{new SimTK::Matrix_<ET>{m_data}};
  }

  /** Add(append) a row to the DataTable_ using a SimTK::RowVector_. If the 
  DataTable is empty, input row will be the first row. This function can be
  used to populate an empty DataTable_.
  
  \param row The row to be added as a SimTK::RowVector_.
  
  \throws ZeroElements If number of elements in the input row is zero.
  \throws NumberOfColsMismatch If number of columns in the input row does not 
                               match the number of columns of the DataTable_. */
  void addRow(const SimTK::RowVector_<ET>& row) {
    if(row.nrow() == 0 || row.ncol() == 0)
      throw ZeroElements{"Input row has zero length."};
    if(m_data.ncol() > 0 && row.size() != m_data.ncol())
      throw NumberOfColsMismatch{"Input row has incorrect number of columns."
                                 " Expected = " + 
                                 std::to_string(m_data.ncol()) + 
                                 " Received = " + std::to_string(row.size())};

    m_data.resizeKeep(m_data.nrow() + 1, row.ncol());
    m_data.updRow(m_data.nrow() - 1).updAsRowVector() = row;
  }

  /** Add(append) a row to the DataTable_ using an iterator(satisfying 
  requirements of an input_iterator) producing one entry at a time. If this
  function is called on an empty DataTable_ without providing ncol_hint, it
  performs <i>allocation + relocation</i> for [log2(ncol) + 1] times where ncol 
  is the actual number of elements produced by the input iterator. To add 
  multiple rows at once using an input-iterator, use addRows().
  
  \param first Beginning of range covered by the iterator.
  \param last End of the range covered by the iterator.
  \param ncol_hint Hint for the number of columns in the input iterator. Can be 
                   approximate above or below the actual number. This is only 
                   used when this function is called on an empty DataTable_. 
                   Ignored otherwise. Providing a hint reduces the number of 
                   resize operations which involves memory allocation and 
                   relocation.
  \param allow_missing Allow for missing values. When set to false, this
                       function will throw if the input iterator fills up the 
                       row only partially. When set to true, missing elements 
                       are set to SimTK::NaN.
  
  \throws ZeroElements If the number of elements produced by the input iterator
                       is zero.
  \throws InvalidEntry The DataTable is empty and the input argument 
                       'ncol_hint' is zero.
  \throws NotEnoughElements If allow_missing is false and the input iterator 
                            does not produce enough elements to fill up the 
                            row completely.                                   */
  template<typename InputIt>
  void addRow(InputIt first, 
              InputIt last, 
              const size_t ncol_hint = 2,
              const bool allow_missing = false) {
    if(!(first != last))
      throw ZeroElements{"Input iterators produce zero elements."};
    if((m_data.nrow() == 0 || m_data.ncol() == 0) && ncol_hint == 0)
      throw InvalidEntry{"Input argument 'ncol_hint' cannot be zero when "
                         "DataTable is empty."};

    if(m_data.ncol() > 0) {
      m_data.resizeKeep(m_data.nrow() + 1, m_data.ncol());
      int col{0};
      while(first != last) {
        m_data.set(m_data.nrow() - 1, col++, *first);
        ++first;
      }
      if(!allow_missing && col != m_data.ncol())
        throw NotEnoughElements{"Input iterator did not produce enough elements"
                                " to fill the row. Expected = " + 
                                std::to_string(m_data.ncol()) + 
                                " Received = " + std::to_string(col)};
    } else {
      int col{0};
      size_t ncol{ncol_hint};
      m_data.resizeKeep(1, static_cast<int>(ncol));
      while(first != last) {
        m_data.set(0, col++, *first);
        ++first;
        if(col == static_cast<int>(ncol) && first != last) {
          // If ncol is a power of 2, double it. Otherwise round it to the next
          // power of 2.
          ncol = (ncol & (ncol - 1)) == 0 ? ncol << 2 : rndToNextPowOf2(ncol); 
          m_data.resizeKeep(1, static_cast<int>(ncol));
        }
      }
      if(col != static_cast<int>(ncol))
        m_data.resizeKeep(1, col);
    }
  }

  /** Add(append) multiple rows to the DataTable_ using an iterator(satisfying 
  requirements of an input_iterator) producing one entry at a time. If this
  function is called on an empty DataTable_, ncol must be provided.
  Otherwise, ncol is ignored. To add just one row, use addRow().

  \param first Beginning of range covered by the iterator.
  \param last End of the range covered by the iterator.
  \param ncol Number of columns to create in the DataTable_. This is only
              used(and required) when the function is called on an empty 
              DataTable_. Ignore otherwise.
  \param allow_missing Allow for missing values. When set to false, this
                       function will throw if the input iterator fills up the 
                       last row only partially. When set to true, missing 
                       elements are set to SimTK::NaN.

  \throws ZeroElements If the number of elements produced by the input iterator
                       is zero.
  \throws InvalidEntry If the function is called on an empty DataTable_ 
                       -- without providing the argument ncol or 
                       -- providing a zero for ncol.
  \throws NotEnoughElements If allow_missing is false and the input iterator
                            does not produce enough elements to fill up the 
                            last row completely.                              */
  template<typename InputIt>
  void addRows(InputIt first, 
               InputIt last, 
               const size_t ncol = std::numeric_limits<size_t>::max(),
               const bool allow_missing = false) {
    if(!(first != last))
      throw ZeroElements{"Input iterators produce zero elements."};
    if((m_data.nrow() == 0 || m_data.ncol() == 0) && 
       (ncol == std::numeric_limits<size_t>::max() || ncol == 0))
      throw InvalidEntry{"DataTable is empty. 'ncol' argument must be" 
                         " provided and it cannot be zero."};

    m_data.resizeKeep(m_data.nrow() + 1, 
                      m_data.ncol() == 0 ? 
                      static_cast<int>(ncol) : m_data.ncol());
    int row{m_data.nrow() - 1};
    int col{0};
    while(first != last) {
      m_data.set(row, col, *first);
      ++first; ++col;
      if(col == static_cast<int>(m_data.ncol()) && first != last) {
        col = 0;
        ++row;
        m_data.resizeKeep(m_data.nrow() + 1, m_data.ncol());
      }
    }
    if(!allow_missing && col != m_data.ncol())
      throw NotEnoughElements{"Input iterator did not produce enough elements" 
                              " to fill the last row. Expected = " + 
                              std::to_string(m_data.ncol()) + " Received = " + 
                              std::to_string(col)};
  }

  /** Add(append) a col to the DataTable_ using a SimTK::Vector_. If the 
  DataTable is empty, input col will be the first col. This function can be
  used to populate an empty DataTable_.
  
  \param col The col to be added as a SimTK::Vector_.
  
  \throws ZeroElements If number of elements in the input col is zero.
  \throws NumberOfRowsMismatch If number of columns in the input col does not 
                               match the number of rows of the DataTable_.    */
  void addCol(const SimTK::Vector_<ET>& col) {
    if(col.nrow() == 0 || col.ncol() == 0)
      throw ZeroElements{"Input column has zero length."};
    if(m_data.nrow() > 0 && col.size() != m_data.nrow())
      throw NotEnoughElements{"Input column has incorrect number of rows. " 
                              "Expected = " + std::to_string(m_data.nrow()) + 
                              " Received = " + std::to_string(col.size())};

    m_data.resizeKeep(col.size(), m_data.ncol() + 1);
    m_data.updCol(m_data.ncol() - 1).updAsVector() = col;
  }

  /** Add(append) a col to the DataTable_ using an iterator(satisfying 
  requirements of an input_iterator) producing one entry at a time. If this
  function is called on an empty DataTable_ without providing nrow_hint, it
  performs <i>allocation + relocation</i> for [log2(nrow) + 1] times where nrow 
  is the actual number of elements produced by the input iterator. To add 
  multiple cols at once using input-iterator, use addCols().

  \param first Beginning of range covered by the iterator.
  \param last End of the range covered by the iterator.
  \param nrow_hint Hint for the number of rows in the input iterator. Can be 
                   approximate above or below the actual number. This is only 
                   used when this function is called on an empty DataTable_. 
                   Ignored otherwise. Providing a hint reduces the number of 
                   resize operations which involves memory allocation + r
                   elocation.
  \param allow_missing Allow for missing values. When set to false, this
                       function will throw if the input iterator fills up the 
                       row only partially. When set to true, missing elements 
                       are set to SimTK::NaN.

  \throws ZeroElements If the number of elements produced by the input
                       iterator is zero.                                      
  \throws InvalidEntry DataTable is empty and input argument ncol_hint is zero.
  \throws NotEnoughElements Argument allow_missing is false and the input 
                            iterator does not produce enough elements to fill 
                            up the last col completely                        */
  template<typename InputIt>
  void addCol(InputIt first, 
              InputIt last, 
              const size_t nrow_hint = 2,
              const bool allow_missing = false) {
    if(!(first != last))
      throw ZeroElements{"Input iterators produce zero elements."};
    if((m_data.nrow() == 0 || m_data.ncol() == 0) && nrow_hint == 0)
      throw InvalidEntry{"Input argument 'nrow_hint' cannot be zero when "
                         "DataTable is empty."};

    if(m_data.nrow() > 0) {
      m_data.resizeKeep(m_data.nrow(), m_data.ncol() + 1);
      int row{0};
      while(first != last) {
        m_data.set(row++, m_data.ncol() - 1, *first);
        ++first;
      }
      if(!allow_missing && row != m_data.nrow()) 
        throw NotEnoughElements{"Input iterator did not produce enough elements"
                                " to fill the col. Expected = " + 
                                std::to_string(m_data.nrow()) + 
                                " Received = " + std::to_string(row)};
    } else {
      int row{0};
      size_t nrow{nrow_hint};
      m_data.resizeKeep(static_cast<int>(nrow), 1);
      while(first != last) {
        m_data.set(row++, 0, *first);
        ++first;
        if(row == static_cast<int>(nrow) && first != last) {
          // If nrow is a power of 2, double it. Otherwise round it to the next
          // power of 2.
          nrow = (nrow & (nrow - 1)) == 0 ? nrow << 2 : rndToNextPowOf2(nrow); 
          m_data.resizeKeep(static_cast<int>(nrow), 1);
        }
      }
      if(row != static_cast<int>(nrow))
        m_data.resizeKeep(row, 1);
    }
  }

  /** Add(append) multiple cols to the DataTable_ using an iterator(satisfying 
  requirements of an input_iterator) producing one entry at a time. If this
  function is called on an empty DataTable_, nrow must be provided. Otherwise, 
  nrow is ignored. To add just one col, use addRow().

  \param first Beginning of range covered by the iterator.
  \param last End of the range covered by the iterator.
  \param nrow Number of rows to create in the DataTable_. This is only used 
              (and required) when the function is called on an empty DataTable_.
              Ignore otherwise.
  \param allow_missing Allow for missing values. When set to false, this
                       function will throw if the input iterator fills up the 
                       last col only partially. When set to true, missing 
                       elements are set to SimTK::NaN.

  \throws ZeroElements If the number of elements produced by the input iterator
                       is zero.
  \throws InvalidEntry If the function is called on an empty DataTable_ 
                       -- without providing the argument nrow or 
                       -- providing zero for nrow.
  \throws NotEnoughElements If allow_missing is false and the input iterator
                            does not produce enough elements to fill up the 
                            last col completely.                              */
  template<typename InputIt>
  void addCols(InputIt first, 
               InputIt last, 
               const size_t nrow = std::numeric_limits<size_t>::max(),
               const bool allow_missing = false) {
    if(!(first != last))
      throw ZeroElements{"Input iterators produce zero elements."};
    if((m_data.nrow() == 0 || m_data.ncol() == 0) && 
       (nrow == std::numeric_limits<size_t>::max() || nrow == 0))
      throw InvalidEntry{"DataTable is empty. 'nrow' argument must be" 
                         " provided and it cannot be zero."};

    m_data.resizeKeep(m_data.nrow() == 0 ? 
                      static_cast<int>(nrow) : m_data.nrow(), 
                      m_data.ncol() + 1);
    int row{0};
    int col{m_data.ncol() - 1};
    while(first != last) {
      m_data.set(row, col, *first);
      ++first; ++row;
      if(row == static_cast<int>(m_data.nrow()) && first != last) {
        row = 0;
        ++col;
        m_data.resizeKeep(m_data.nrow(), m_data.ncol() + 1);
      }
    }
    if(!allow_missing && row != m_data.nrow())
      throw NotEnoughElements{"Input iterator did not produce enough elements"
                              " to fill the last col. Expected = " + 
                              std::to_string(m_data.nrow()) + " Received = " + 
                              std::to_string(row)};
  }

  /** Bind another DataTable_ to this DataTable_ by row. The new elements will 
  appear as the last rows of this DataTable_. Only data will be appended.
  Metadata is not added. Columns retain their labels. To create a new 
  DataTable_  that is a bind of two existing DataTable_(s), see 
  rbindDataTables().

  \throws NumberOfColsMismatch If input DataTable_ has incorrect number of
                               columns for bind to work.
  \throws InvalidEntry If trying to bind a DataTable_ to itself.              */
  void rbindDataTable(const DataTable_& table) {
    if(m_data.ncol() != table.m_data.ncol()) 
      throw NumberOfColsMismatch{"Input DataTable has incorrect number of " 
                                 "columns. Expected = " + 
                                 std::to_string(m_data.ncol()) + " Received = " 
                                 + std::to_string(table.m_data.ncol())};
    if(&m_data == &table.m_data)
      throw InvalidEntry{"Cannot rbind a DataTable to itself."};

    int old_nrow{m_data.nrow()};
    m_data.resizeKeep(m_data.nrow() + table.m_data.nrow(), m_data.ncol());
    m_data.updBlock(old_nrow, 
                    0, 
                    table.m_data.nrow(), 
                    m_data.ncol()).updAsMatrix() = table.m_data;
  }

  /** Bind another DataTable_ to this DataTable_ by col. The new elements will 
  appear as the last cols of this DataTable_. Only data will be appended.
  Column labels and metadata are not added. To create a new DataTable_ that is 
  a bind of two existing DataTable_(s), see cbindDataTables().

  \throws NumberOfRowsMismatch If input DataTable_ has incorrect number of
                               rows for bind to work.
  \throws InvalidEntry If trying to bind a DataTable_ to itself.              */
  void cbindDataTable(const DataTable_& table) {
    if(m_data.nrow() != table.m_data.nrow())
      throw NumberOfRowsMismatch{"Input DataTable has incorrect number of " 
                                 "rows. Expected = " + 
                                 std::to_string(m_data.nrow()) + " Received = " 
                                 + std::to_string(table.m_data.nrow())};
    if(&m_data == &table.m_data)
      throw InvalidEntry{"Cannot cbind a DataTable to itself."};

    int old_ncol{m_data.ncol()};
    m_data.resizeKeep(m_data.nrow(), m_data.ncol() + table.m_data.ncol());
    m_data.updBlock(0, 
                    old_ncol,
                    m_data.nrow(), 
                    table.m_data.ncol()).updAsMatrix() = table.m_data;
  }

  /** Clear the data of this DataTable_. After this operation, the DataTabe_
  will be of size 0x0 and all column labels will be cleared as well.          */
  void clearData() {
    m_data.clear();
    m_col_ind.clear();
  }

  /**---------------------------------------------------------------------------
    Meta-data accessors & mutators.
  ----------------------------------------------------------------------------*/

  /** Insert metadata. DataTable_ can hold metadata as an associative array of 
  key-value pairs where is key is always of type std::string and value can be 
  of any type(except an array type[eg char[]]). The metadata inserted can later 
  be retrieved using the functions getMetaData() and updMetaData(). This 
  function throws if the key is already present.

  \param key A std::string that can be used the later to retrieve the inserted 
             metadata.
  \param value An object/value of any type except array types[eg int[]]. The 
               code will fail to compile for array types.

  \throws MetaDataKeyExists If the specified key already exits                */
  template<typename ValueType>
  void insertMetaData(const std::string& key, ValueType&& value) {
    using namespace SimTK;
    using ValueTypeNoRef = typename std::remove_reference<ValueType>::type;

    static_assert(!std::is_array<ValueTypeNoRef>::value,
                  "'value' cannot be of array type. For ex. use std::string"
                  " instead of char[], use std::vector<int> instead of int[].");

    if(metaDataExists(key))
      throw MetaDataKeyExists{"Key '" + std::string{key} + "' already exists. " 
                              "Remove the existing entry before inserting."};

    m_metadata.emplace(key, 
                     new Value<ValueTypeNoRef>{std::forward<ValueType>(value)});
  }

  /** Get previously inserted metadata using its key and type. The template
  argument has to be exactly the non-reference type of the metadata previously 
  stored using insertMetaData(). The return value is a ead-only reference to 
  the metadata. Use updMetaData() to obtain a writable reference. Time 
  complexity is constant on average and linear in number of elements in 
  metadata on worst case.

  \throws MetaDataKeyDoesNotExist If the key specified does not exist in 
                                  metadata.
  \throws MetaDataTypeMismatch If the type specified as template argument does 
                               not match the type of metadata stored under the 
                               key specified.                                 */
  template<typename ValueType>
  const ValueType& getMetaData(const string& key) const {
    static_assert(!std::is_reference<ValueType>::value, 
                  "Template argument 'ValueType' should be exactly the" 
                  " non-reference type of the MetaData value stored.");

    try {
      return m_metadata.at(key)->template getValue<ValueType>();
    } catch(std::out_of_range&) {
      throw MetaDataKeyDoesNotExist{"Key '" + key + "' not found."};
    } catch(std::bad_cast&) {
      throw MetaDataTypeMismatch{"Template argument specified for getMetaData"
                                 " is incorrect."};
    }
  }

  /** Get previously inserted metadata using its key and type. The template
  argument has to be exactly the non-reference type of the metadata previously 
  stored using insertMetaData(). The returned value is editable. Time 
  complexity is constant on average and linear in number of elements in 
  metadata on worst case.

  \throws MetaDataKeyDoesNotExist If the key specified does not exist in 
                                  metadata.
  \throws MetaDataTypeMismatch If the type specified as template argument does 
                               not match the type of metadata stored under the 
                               key specified.                                 */
  template<typename ValueType>
  ValueType& updMetaData(const string& key) {
    static_assert(!std::is_reference<ValueType>::value, "Template argument " 
                  "'ValueType' should be exactly the non-reference type of"
                  "the MetaData value stored.");

    try {
      return m_metadata.at(key)->template updValue<ValueType>();
    } catch(std::out_of_range&) {
      throw MetaDataKeyDoesNotExist{"Key '" + key + "' not found."};
    } catch(std::bad_cast&) {
      throw MetaDataTypeMismatch{"Template argument specified for updMetaData"
                                 " is incorrect"};
    }
  }

  /** Pop previously inserted metadata using its key and type. The template
  argument has to be exactly the non-reference type of the metadata previously 
  inserted using insertMetaData(). The key-value pair is removed from metadata 
  and the value is returned. To simply remove the key-value pair without 
  retrieving the value, use rmvMetaData(). Time complexity is constant on
  average and linear in number of elements in the metadata on worst case.

  \throws MetaDataKeyDoesNotExist If the key specified does not exist in 
                                  metadata.
  \throws MetaDataTypeMismatch If the type specified as template argument does 
                               not match the type of metadata stored under the 
                               key specified.                                 */
  template<typename ValueType>
  ValueType popMetaData(const string& key) {
    static_assert(!std::is_reference<ValueType>::value, "Template argument "
                  "'ValueType' should be exactly the non-reference type of"
                  "the MetaData value stored.");

    try {
      ValueType value = 
        std::move(m_metadata.at(key)->template getValue<ValueType>());
      m_metadata.erase(key);
      return value;
    } catch(std::out_of_range&) {
      throw MetaDataKeyDoesNotExist{"Key '" + key + "' not found."};
    } catch(std::bad_cast&) {
      throw MetaDataTypeMismatch{"Template argument specified for popMetaData"
                                 " is incorrect"};
    }
  }

  /** Remove a metadata key-value pair previously inserted. Return value 
  indicates if there was a removal -- true means a key-value pair was
  removed; false means the key was not found in metadata. Time complexity is
  constant on average and linear in number of elements in the metadata on
  worst case.                                                                 */
  bool rmvMetaData(const string& key) {
    return m_metadata.erase(key);
  }

  /** Clear the metadata. All the metadata will be lost with this operation.  */
  void clearMetaData() {
    m_metadata.clear();
  }

  /** Check if metadata for a given key exists. Time complexity is constant on
  average and linear in the number of elements in the metadata on worst case. */
  bool metaDataExists(const string& key) const {
    return m_metadata.find(key) != m_metadata.end();
  }

  /** Check if metadata is empty -- if the number of elements is zero.        */
  bool isMetaDataEmpty() const {
    return m_metadata.empty();
  }

  /** Get the number of elements in the metadata. Time complexity of other 
  operations on metadata depend on this number.                               */
  size_t metaDataSize() const {
    return m_metadata.size();
  }

  /**---------------------------------------------------------------------------
     Column labels accessors & mutators.                                       
  ----------------------------------------------------------------------------*/

  /** Check if a column index has label. Time complexity is linear in number of 
  column labels. All columns will have an index. All columns need not have a 
  label.

  \throws ColumnDoesNotExist If col index specified does not exist.           */
  bool colHasLabel(const size_t colind) const override {
    using ColLabelsValueType = typename ColLabelsType::value_type;
    checkColExists(colind);
    auto res = std::find_if(m_col_ind.begin(), 
                            m_col_ind.end(), 
                            [colind] (const ColLabelsValueType& kv) {
                              return kv.second == colind;
                            });
    return res != m_col_ind.end();
  }

  /** Check if a column exists by its index.                                  */
  bool colExists(const size_t colind) const override {
    return colind >= 0 && colind < static_cast<size_t>(m_data.ncol());
  }

  /** Check if a column exists under given label. All columns have an index but
  not all columns may have labels.                                            */
  bool colExists(const string& collabel) const override {
    return m_col_ind.find(collabel) != m_col_ind.end();
  }

  /** Label a column. The column should not have a label already. To update the
  label of a column that already has a label, use updColLabel().

  \throws ColumnDoesNotExist If the col index specified does not exist.
  \throws ColumnHasLabel If the column index specified already has a label.   */
  void insertColLabel(const size_t colind, const string& collabel) override {
    checkColExistsAndHasLabel(colind);
    m_col_ind.emplace(collabel, colind);
  }

  /** Label a column. The column should not have a label already. To update the
  label of a column that already has a label, use updColLabel().

  \throws ColumnDoesNotExist If the col index specified does not exist.
  \throws ColumnHasLabel If the column index specified already has a label.   */
  void insertColLabel(const size_t colind, string&& collabel) override {
    checkColExistsAndHasLabel(colind);
    m_col_ind.emplace(std::move(collabel), colind);
  }

  /** Label a set of columns at once using an input iterator that produces one
  index-label pair (std::pair<size_t, std::string>) at a time. The columns
  referred to by the iterator must not already have a label. 

  \throws ColumnDoesNotExist If the col index specified does not exist.
  \throws ColumnHasLabel If the column index specified already has a label.   */
  template<typename InputIt>
  void insertColLabels(InputIt first, InputIt last) {
    while(first != last) {
      checkColExistsAndHasLabel(first->first);
      m_col_ind.emplace(first->second, first->first);
      ++first;
    }
  }
  
  /** Get the label of a column. Time complexity is linear in the number of
  column labels. The returned value is a copy of the label. To update the label 
  of a column, use updColLabel(). 

  \throws ColumnHasNoLabel If the column does not have a label.
  \throws ColumnDoesNotExist If the column does not exist.                    */
  string getColLabel(const size_t colind) const override {
    checkColExists(colind);
    using ColLabelsValueType = typename ColLabelsType::value_type;
    auto res = std::find_if(m_col_ind.begin(),
                            m_col_ind.end(),
                            [colind] (const ColLabelsValueType& kv) {
                              return kv.second == colind;
                            });
    if(res == m_col_ind.end()) {
      throw ColumnHasNoLabel{"Column " + std::to_string(colind) + 
                             " has no label."};
    }

    return res->first;
  }

  /** Get all the column labels. Returns an iterator pair(std::pair) where first
  element of pair is the beginning and second element of the pair is the end 
  (sentinel) of the labels. Dereferencing the iterator will produce a pair
  (std::pair) where the first element of the pair is the label and the second 
  element is the column index. To update the column abels, use updColLabels().*/
  ColLabelsConstIterPairType getColLabels() const override {
    return std::make_pair(m_col_ind.cbegin(), m_col_ind.cend());
  }

  /** Update the label of a column with a new label. Time complexity is linear
  in the number of column labels. The column specified must already have a
  label. To label a column that does not yet have a label, use insertLabel().

  \throws ColumnHasNoLabel If the column specified does not already have a 
                           label.
  \throws ColumnDoesNotExist If the column specified does not exist.          */
  void updColLabel(const size_t colind, const string& new_collabel) override {
    string old_collabel{getColLabel(colind)};
    m_col_ind.erase(old_collabel);
    m_col_ind.emplace(new_collabel, colind);
  }

  /** Update the label of a column with a new label. Time complexity is constant
  on average and linear in number of column labels in the worst case.

  \throws ColumnHasNoLabel If the column specified does not already have a 
                           label.
  \throws ColumnDoesNotExist If the column specified does not exist.          */
  void updColLabel(const string& old_collabel, 
                   const string& new_collabel) override {
    size_t colind{getColInd(old_collabel)};
    m_col_ind.erase(old_collabel);
    m_col_ind[new_collabel] = colind;
  }

  /** Update all the column labels. Returns an iterator pair(std::pair) where
  first element of pair is the beginning and second element of the pair is the 
  end(sentinel) of the labels. Dereferencing the iterator will produce a pair 
  (std::pair) where the first element of the pair is the label and the second 
  element is the column index.                                                */
  ColLabelsIterPairType updColLabels() override {
    return std::make_pair(m_col_ind.begin(), m_col_ind.end());
  }

  /** Get the index of a column from its label. Time complexity is constant on
  average and linear in number of column labels on worst case.

  \throws ColumnDoesNotExist If the column label does not exist.              */
  size_t getColInd(const string& collabel) const override {
    try {
      return m_col_ind.at(collabel);
    } catch(const std::out_of_range&) {
      throw ColumnDoesNotExist{"No column with label '" + collabel + "'."};
    }
  }

  /** Clear all the column labels.                                            */
  void clearColLabels() override {
    m_col_ind.clear();
  }

private:
  // Helper function. Check if a column exists and throw an exception if it
  // does not.
  void checkColExists(const size_t colind) const {
    if(!colExists(colind)) {
      throw ColumnDoesNotExist{"Column " + std::to_string(colind) + 
                               " does not exist."};
    }
  }

  // Helper function. Check if a column has label and throw an exception if it
  // does not.
  void checkColHasLabel(const size_t colind) const {
    if(colHasLabel(colind)) {
      throw ColumnHasLabel{"Column " + std::to_string(colind) + 
                           " already has a label."};
    }
  }

  // Helper function. Does both checkColExists() and checkColHasLabel().
  void checkColExistsAndHasLabel(const size_t colind) const {
    checkColExists(colind);
    checkColHasLabel(colind);
  }

  // Helper function. Round to next highest power of 2. Works only for 32 bits.
  size_t rndToNextPowOf2(size_t num) {
    --num;
    num |= (num >>  1); // Highest  2 bits are set by end of this.
    num |= (num >>  2); // Highest  4 bits are set by end of this.
    num |= (num >>  4); // Highest  8 bits are set by end of this.
    num |= (num >>  8); // Highest 16 bits are set by end of this.
    num |= (num >> 16); // Highest 32 bits are set by end of this.
    return ++num;
  }

  // Matrix of data. This excludes timestamp column.
  SimTK::Matrix_<ET> m_data;
  // Meta-data.
  MetaDataType       m_metadata;
  // Column label to column index.
  ColLabelsType      m_col_ind;
};  // DataTable_


/// Bind two DataTable_(s) by row and produce a new DataTable_.
template<typename ET>
OpenSim::DataTable_<ET> 
OpenSim::rbindDataTables(const OpenSim::DataTable_<ET>& dt1, 
                         const OpenSim::DataTable_<ET>& dt2) {
  OpenSim::DataTable_<ET> dt{dt1};
  dt.rbindDataTable(dt2);
  return dt;
}


/// Bind two DataTable_(s) by col and produce a new DataTable_.
template<typename ET>
OpenSim::DataTable_<ET> 
OpenSim::cbindDataTables(const OpenSim::DataTable_<ET>& dt1, 
                         const OpenSim::DataTable_<ET>& dt2) {
  OpenSim::DataTable_<ET> dt{dt1};
  dt.cbindDataTable(dt2);
  return dt;
}

#endif //OPENSIM_DATA_TABLE_H_
