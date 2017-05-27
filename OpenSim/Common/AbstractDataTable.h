/* -------------------------------------------------------------------------- *
 *                    OpenSim:  AbstractDataTable.h                           *
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

#ifndef OPENSIM_ABSTRACT_DATA_TABLE_H_
#define OPENSIM_ABSTRACT_DATA_TABLE_H_

/** \file
This file defines the AbstractDataTable class, which is used by OpenSim to 
provide an in-memory container for data access and manipulation.              */

// Non-standard headers.
#include "OpenSim/Common/Exception.h"
#include "OpenSim/Common/ValueArrayDictionary.h"

#include <ostream>

namespace OpenSim {

class InvalidRow : public Exception {
public:
    using Exception::Exception;
};

class InvalidColumn : public Exception {
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
        std::string msg = "Incorrect number of columns. ";
        msg += "Expected = " + std::to_string(expected);
        msg += ", Received = " + std::to_string(received);

        addMessage(msg);
    }
};

class IncorrectNumRows : public InvalidColumn {
public:
    IncorrectNumRows(const std::string& file,
                     size_t line,
                     const std::string& func,
                     size_t expected,
                     size_t received) :
        InvalidColumn(file, line, func) {
        std::string msg = "Incorrect number of rows. ";
        msg += "Expected = " + std::to_string(expected);
        msg += ", Received = " + std::to_string(received);

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

class NoColumnLabels : public Exception {
public:
    NoColumnLabels(const std::string& file,
                   size_t line,
                   const std::string& func) :
        Exception(file, line, func) {
        std::string msg = "Table has no column-labels. Use setColumnLabels() to"
                          " add labels.";

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

class EmptyTable : public Exception {
public:
    using Exception::Exception;
};

class KeyExists : public Exception {
public:
    KeyExists(const std::string& file,
              size_t line,
              const std::string& func,
              const std::string& key) :
        Exception(file, line, func) {
        std::string msg = "Key '" + key + "' not found.";

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
class OSIMCOMMON_API AbstractDataTable {
public:
    typedef ValueArrayDictionary TableMetaData;
    typedef ValueArrayDictionary DependentsMetaData;
    typedef ValueArrayDictionary IndependentMetaData;

    AbstractDataTable()                                      = default;
    AbstractDataTable(const AbstractDataTable&)              = default;
    AbstractDataTable(AbstractDataTable&&)                   = default;
    AbstractDataTable& operator=(const AbstractDataTable&)   = default;
    AbstractDataTable& operator=(AbstractDataTable&&)        = default;
    virtual std::shared_ptr<AbstractDataTable> clone() const = 0;
    virtual ~AbstractDataTable()                             = default;

    /** Get number of components per element of the DataTable. See documentation
    for DataTable on possible return values.                                  */
    virtual unsigned numComponentsPerElement() const = 0;

    /** Get number of rows.                                                   */
    size_t getNumRows() const;

    /** Get number of dependent columns.                                      */
    size_t getNumColumns() const;

    /// @name MetaData accessors/mutators.
    /// @{

    /** Add key-value pair to the table metadata.

    If using this function from Python/Java/Matlab, use:
    ```
    addTableMetaDataString(key, value)
    ```
    where both 'key' and 'value' are strings. The above call translates to C++
    as:
    ```
    addTableMetaData<std::string>(key, value)
    ```

    \tparam Value Type of the value. This need not be specified explicitly in
                  most cases. It will be deduced automatically.

    \throws KeyExists If the key provided already exists in table metadata.   */
    template<typename Value>
    void addTableMetaData(const std::string& key, const Value& value) {
        OPENSIM_THROW_IF(!_tableMetaData.setValueForKey(key, value),
                         KeyExists,
                         key);
    }

    /** Whether or not table metadata for the given key exists.               */
    bool hasTableMetaDataKey(const std::string& key) const {
        return _tableMetaData.hasKey(key);
    }

    /** Get table metadata for a given key.

    If using this function from Python/Java/Matlab, use the following table:
    <table>
    <tr>
      <th>C++</th>
      <th>Python / Java / Matlab</th>
      <th>Example</th>
    </tr>
    <tr>
      <td>%getTableMetaData<std::string></td>
      <td>getTableMetaDataString</td>
      <td>%Marker table read from a C3D file could contain quantities 
          <b>DataRate</b> and <b>%Units</b> that can be retrieved with this 
          method.</td>
    </tr>
    <tr>
      <td>getTableMetaData<std::vector<SimTK::Matrix_<double>>></td>
      <td>getTableMetaDataVectorMatrix</td>
      <td>Forces table read from a C3D file could contain quantities 
          <b>Calibration Matrices</b>, <b>%Force Plate Corners</b> and <b>%Force
          Plate Origins</b> that can be retrieved with this method.</td>
    </tr>
    <tr>
      <td>getTableMetaData<std::vector<unsigned>></td>
      <td>getTableMetaDataVectorUnsigned</td>
      <td>Forces table read from a C3D file could contain quantity <b>%Force 
          Plate Types</b> that can be retrieved with this method</td>
    </tr>
    </table>

    \tparam Value Type of the value to be retrieved. For example if the metadata
                  contains key-value pair ("sample-rate", 200), this could be
                  specified as 'unsigned'.

    \throws KeyNotFound If the key provided is not found in table metadata.   */
    template<typename Value>
    Value getTableMetaData(const std::string& key) const {
        const auto& absValue = _tableMetaData.getValueForKey(key);
        try {
            const auto& value = 
                dynamic_cast<const SimTK::Value<Value>&>(absValue);
            return value.get();
        } catch (const std::bad_cast&) {
            OPENSIM_THROW(InvalidTemplateArgument,
                          "The value stored as Table MetaData for key '" + key +
                          "' is not of type that is provided as template "
                          "argument.");
        }
    }

    /** Get table metadata for a given key as a string.

    \throws KeyNotFound If the key provided is not found in table metadata.   */
    std::string getTableMetaDataAsString(const std::string& key) const {
        return _tableMetaData.getValueAsString(key);
    }

    /** Remove key-value pair associated with the given key from table 
    metadata.                                                                 */
    void removeTableMetaDataKey(const std::string& key);

    /** Get table metadata keys.                                              */
    std::vector<std::string> getTableMetaDataKeys() const;

    /** Get metadata associated with the table.                               */
    const TableMetaData& getTableMetaData() const;

    /** Update metadata associated with the table.                            */
    TableMetaData& updTableMetaData();

    /** Get metadata associated with the independent column.                  */
    const IndependentMetaData& getIndependentMetaData() const;
    
    /** %Set metadata associated with the independent column.                 */
    void 
    setIndependentMetaData(const IndependentMetaData& independentMetaData);

    /** Get metadata associated with the dependent columns.                   */
    const DependentsMetaData& getDependentsMetaData() const;

    /** %Set metadata associated with the dependent columns.                  */
    void 
    setDependentsMetaData(const DependentsMetaData& dependentsMetaData);

    /** Remove key-value pair associated with the given key from dependents
    metadata.                                                                 */
    void removeDependentsMetaDataForKey(const std::string& key);

    /// @} End of MetaData accessors/mutators.

    /// @name Column-labels related accessors/mutators.
    /// Following functions operate on column labels of dependent columns only
    /// excluding the independent column.
    /// Following are examples on using setColumnLabels(). If you have a 
    /// sequence of strings, you can pretty much call setColumnLabels() on it.
    /// \code
    /// // Simplest way to set column labels is to provide them directly to
    /// // setColumnLabels.
    /// table.setColumnLabels({"col1", "col2", "col3"});
    /// \endcode
    /// \code
    /// // if you have a sequence container like std::vector or std::list of 
    /// // std::string holding column labels, pass the container directly to
    /// // setColumnLabels.
    /// std::list<std::string> columnLabels{"col1", "col2", "col3"};
    /// table.setColumnLabels(columnLabels);
    /// \endcode
    /// \code
    /// // If you have a sequence container like std::vector or std::list of 
    /// // std::string holding column labels but you want to use only a subset
    /// // of them to set column labels of the table, use iterators like below.
    /// std::vector<std::string> columnLabels{"col-not-used1", 
    ///                                       "col1", "col2", "col3", 
    ///                                       "col-not-used2"};
    /// table.setColumnLabels(columnLabels.begin() + 1, 
    ///                       columnLabels.end() - 1);
    /// \endcode
    /// @{

    /** Does the table have non-zero number of column labels.                 */
    bool hasColumnLabels() const;

    /** Get column labels.                                                    

    \throws NoColumnLabels If column labels have not be set for the table.    */
    std::vector<std::string> getColumnLabels() const;

    /** Get column label of a given column.                                   

    \throws ColumnIndexOutOfRange If columnIndex is out of range of number of
                                  columns.                                    
    \throws NoColumnLabels If column labels have not be set for the table.    */
    const std::string& getColumnLabel(const size_t columnIndex) const;

    /** %Set column labels using a pair of iterators.

    Example:
    \code
    std::vector<std::string> labels{"col1", "col2", "col3", "col4", "col5"};
    // Use subsequence of vector as labels.
    setColumnLabels(labels.begin() + 2, labels.end());
    \endcode

    \param first InputIterator representing the beginning of the sequence of
                 labels.
    \param last InputIterator representing the sentinel or one past the end of
                sequence of labels.                                          

    \throws MetaDataLengthZero If input sequence of labels is zero.
    \throws IncorrectMetaDataLength If length of the input sequence of labels is
                                    incorrect -- does not match the number of
                                    columns in the table.                     */
    template<typename InputIt>
    void setColumnLabels(InputIt first, InputIt last) {
        OPENSIM_THROW_IF(first == last, 
                         MetaDataLengthZero,
                         "labels");

        ValueArray<std::string> labels{};
        for(auto it = first; it != last; ++it)
            labels.upd().push_back(SimTK::Value<std::string>(*it));
        _dependentsMetaData.removeValueArrayForKey("labels");
        _dependentsMetaData.setValueArrayForKey("labels", labels);

        validateDependentsMetaData();
    }

    /** %Set column labels using a sequence container.

    Example:
    \code
    std::vector<std::string> columnLabels{"col1", "col2", "col3"};
    setColumnLabels(columnLabels);
    \endcode

    \tparam Container Any container type (like std::vector, std::list or your 
                      own) that supports begin() and end(). Type of the values
                      produced by iterator should be std::string.

    \throws MetaDataLengthZero If input sequence of labels is zero.
    \throws IncorrectMetaDataLength If length of the input sequence of labels is
                                    incorrect -- does not match the number of
                                    columns in the table.                     */
    template<typename Container>
    void setColumnLabels(const Container& columnLabels) {
        setColumnLabels(columnLabels.begin(), columnLabels.end());
    }

    /** %Set column labels using a std::initializer_list.
    
    Example:
    \code
    setColumnLabels({"col1", "col2", "col3"});
    \endcode                                                                  

    \throws MetaDataLengthZero If input sequence of labels is zero.
    \throws IncorrectMetaDataLength If length of the input sequence of labels is
                                    incorrect -- does not match the number of
                                    columns in the table.                     */
    void 
    setColumnLabels(const std::initializer_list<std::string>& columnLabels);

    /** %Set the label for a column.                                          

    \throws NoColumnLabels If table has no column labels.
    \throws ColumnIndexOutOfRange If columnIndex is out of range for number of
                                  columns in the table.                       */
    void setColumnLabel(const size_t columnIndex,
                        const std::string& columnLabel);

    /** Get index of a column label.                                          

    \throw NoColumnLabels If table has no column labels.
    \throw KeyNotFound If columnLabel is not found to be label for any column.*/
    size_t getColumnIndex(const std::string& columnLabel) const;

    /** Check if the table has a column with the given label.

    \throw NoColumnLabels If table has no column labels.                     */
    bool hasColumn(const std::string& columnLabel) const;

    /// @} End of Column-labels related accessors/mutators.

    /** Check if the table has a column with the given index.                 */
    bool hasColumn(const size_t columnIndex) const;

protected:
    /** Append column-label.                                                  */
    void appendColumnLabel(const std::string& columnLabel);
    
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

} // namespace OpenSim

#endif //OPENSIM_ABSTRACT_DATA_TABLE_H_
