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

#ifndef OPENSIM_COMMON_DATATABLE_H_
#define OPENSIM_COMMON_DATATABLE_H_

// Non-standard headers.
#include "SimTKcommon.h"
#include "OpenSim/Common/Exception.h"

// Standard headers.
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
#include <string>
#include <type_traits>
#include <limits>
#include <algorithm>
#include <cstdlib>


namespace OpenSim {

/** \cond */
class EmptyDataTable : public Exception {
public:
    EmptyDataTable(const std::string& expl) : Exception(expl) {}
};

class NotEnoughElements : public Exception {
public:
    NotEnoughElements(const std::string& expl) : Exception(expl) {}
};

class TooManyElements : public Exception {
public:
    TooManyElements(const std::string& expl) : Exception(expl) {}
};

class NumberOfColumnsMismatch : public Exception {
public:
    NumberOfColumnsMismatch(const std::string& expl) : Exception(expl) {}
};

class NumberOfRowsMismatch : public Exception {
public:
    NumberOfRowsMismatch(const std::string& expl) : Exception(expl) {}
};

class MissingColumnLabels : public Exception {
public:
    MissingColumnLabels(const std::string& expl) : Exception(expl) {}
};

class ColumnLabelsMismatch : public Exception {
public:
    ColumnLabelsMismatch(const std::string& expl) : Exception(expl) {}
};

class DuplicateColumnLabels : public Exception {
public:
    DuplicateColumnLabels(const std::string& expl) : Exception(expl) {}
};

class RowDoesNotExist : public Exception {
public:
    RowDoesNotExist(const std::string& expl) : Exception(expl) {}
};

class ColumnDoesNotExist : public Exception {
public:
    ColumnDoesNotExist(const std::string& expl) : Exception(expl) {}
};

class ColumnHasLabel : public Exception {
public:
    ColumnHasLabel(const std::string& expl) : Exception(expl) {}
};

class ColumnHasNoLabel : public Exception {
public:
    ColumnHasNoLabel(const std::string& expl) : Exception(expl) {}
};

class ColumnLabelExists : public Exception {
public:
    ColumnLabelExists(const std::string& expl) : Exception(expl) {}
};

class ZeroElements : public Exception {
public:
    ZeroElements(const std::string& expl) : Exception(expl) {}
};

class InvalidEntry : public Exception {
public:
    InvalidEntry(const std::string& expl) : Exception(expl) {}
};

class MetaDataKeyExists : public Exception {
public:
    MetaDataKeyExists(const std::string& expl) : Exception(expl) {}
};

class MetaDataKeyDoesNotExist : public Exception {
public:
    MetaDataKeyDoesNotExist(const std::string& expl) : Exception(expl) {}
};

class MetaDataTypeMismatch : public Exception {
public:
    MetaDataTypeMismatch(const std::string& expl) : Exception(expl) {}
};

class IncompatibleIterators : public Exception {
public:
    IncompatibleIterators(const std::string& expl) : Exception(expl) {}
};

namespace internal {
template<typename...>
using void_t = void;

// Compile time check to see if the iterator (template parameter Iter) supports
// dereferencing operator.
template<typename Iter>
using dereference_t = decltype(*std::declval<Iter>());

template<typename Iter, typename = void>
struct is_dereferencable_t 
    : std::false_type {};

template<typename Iter>
struct is_dereferencable_t<Iter, void_t<dereference_t<Iter>>> 
    : std::true_type {};

template<typename Iter>
constexpr bool is_dereferencable = is_dereferencable_t<Iter>::value;


// Compile time checks to see two types are the same after stripping reference 
// and cv qualifiers.
template<typename T>
using rmv_ref_t = typename std::remove_reference<T>::type;

template<typename T>
using rmv_cv_t = typename std::remove_cv<T>::type;

template<typename T>
using rmv_ref_cv_t = rmv_cv_t<rmv_ref_t<T>>;

template<typename A, typename B>
constexpr bool
is_same = std::is_same<rmv_ref_cv_t<A>, rmv_ref_cv_t<B>>::value;

// Compile time check to see if the a type supports equality comparison.
template<typename T>
using eq_expr_t = decltype(std::declval<T>() == std::declval<T>());

template<typename T, typename = void>
struct is_eq_comparable_t
    : std::false_type {};

template<typename T>
struct is_eq_comparable_t<T, void_t<eq_expr_t<T>>>
    : std::true_type {};

template<typename T>
constexpr bool is_eq_comparable = is_eq_comparable_t<T>::value;

// Compile time check to see if the a type supports inequality comparison.
template<typename T>
using neq_expr_t = decltype(std::declval<T>() != std::declval<T>());

template<typename T, typename = void>
struct is_neq_comparable_t
    : std::false_type {};

template<typename T>
struct is_neq_comparable_t<T, void_t<neq_expr_t<T>>>
    : std::true_type {};

template<typename T>
constexpr bool is_neq_comparable = is_neq_comparable_t<T>::value;

// Compile time check to see if a type has member function named begin().
template<typename C>
using mem_begin_t = decltype(std::declval<C>().begin());

template<typename C, typename = void>
struct has_mem_begin_t 
    : std::false_type {};

template<typename C>
struct has_mem_begin_t<C, void_t<mem_begin_t<C>>>
    : std::true_type {};

template<typename C>
constexpr bool
has_mem_begin = has_mem_begin_t<C>::value;

// Compile time check to see if a type has member function named end().
template<typename C>
using mem_end_t = decltype(std::declval<C>().end());

template<typename C, typename = void>
struct has_mem_end_t 
    : std::false_type {};

template<typename C>
struct has_mem_end_t<C, void_t<mem_end_t<C>>>
    : std::true_type {};

template<typename C>
constexpr bool
has_mem_end = has_mem_end_t<C>::value;
}
/** \endcond */


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
    /** \cond */

    using size_t                = std::size_t;
    using string                = std::string;
    using ColumnLabels          = std::unordered_map<string, size_t>;
    using ColumnLabelsConstIter = ColumnLabels::const_iterator;
    using MetaDataValue         = SimTK::ClonePtr<SimTK::AbstractValue>;
    using MetaData              = std::unordered_map<string, MetaDataValue>;

    // Proxy class pretending to be column labels container.
    class ColumnLabelsContainerProxy {
    public:
        ColumnLabelsContainerProxy(const AbstractDataTable* adt) : adt_{adt} {}
        ColumnLabelsContainerProxy()                                  = delete;
        ColumnLabelsContainerProxy(const ColumnLabelsContainerProxy&) = default;
        ColumnLabelsContainerProxy(ColumnLabelsContainerProxy&&)      = default;
        ColumnLabelsContainerProxy& operator=(const ColumnLabelsContainerProxy&)
                                                                      = default;
        ColumnLabelsContainerProxy& operator=(ColumnLabelsContainerProxy&&) 
                                                                      = default;

        ColumnLabelsConstIter cbegin() const {
            return adt_->columnLabelsBegin();
        }

        ColumnLabelsConstIter cend() const {
            return adt_->columnLabelsEnd();
        }

        ColumnLabelsConstIter begin() const {
            return cbegin();
        }

        ColumnLabelsConstIter end() const {
            return cend();
        }

    private:
        const AbstractDataTable* adt_;
    };

    /** \endcond */
public:
    AbstractDataTable()                                      = default;
    AbstractDataTable(const AbstractDataTable&)              = default;
    AbstractDataTable(AbstractDataTable&&)                   = default;
    AbstractDataTable& operator=(const AbstractDataTable&)   = default;
    AbstractDataTable& operator=(AbstractDataTable&&)        = default;
    virtual std::unique_ptr<AbstractDataTable> clone() const = 0;
    virtual ~AbstractDataTable() {}

    /** \name Column-labels.
        Column labels accessors & mutators.                                   */
    /**@{*/

    /** Get total number of columns that have labels.                         */
    size_t getNumColumnLabels() const {
        return col_ind_.size();
    }

    /** Check if the DataTable_ has a column with given string as the label   */
    bool hasColumnLabel(const std::string& columnLabel) const {
        return col_ind_.find(columnLabel) != col_ind_.end();
    }

    /** Check if a column has label. Time complexity is linear in number 
    of column labels. All columns will have an index (starting at 0). All 
    columns need not have a label.

    \throws ColumnDoesNotExist If column index specified does not exist.      */
    bool columnHasLabel(size_t columnIndex) const {
        using ColumnLabelsValue = typename ColumnLabels::value_type;
        throwIfColumnDoesNotExist(columnIndex);

        auto res = std::find_if(col_ind_.begin(), 
                                col_ind_.end(), 
                                [columnIndex] (const ColumnLabelsValue& kv) {
                                    return kv.second == columnIndex;
                                });
        return res != col_ind_.end();
    }

    /** Check if a column exists using column index.                          */
    virtual bool hasColumn(size_t columnIndex) const = 0;

    /** Check if a column exists using column label. All columns have an index 
    but not all columns may have labels.                                      */
    bool hasColumn(const string& columnLabel) const {
        return col_ind_.find(columnLabel) != col_ind_.end();
    }

    /** Label a column. The column should not have a label already. Column 
    labels are unique for entire DataTable_. To update the label of a column 
    that already has a label, use updColumnLabel().

    \throws ColumnLabelExists If a column in the DataTable_ already has the 
                              label specified by 'columnLabel'.
    \throws ColumnDoesNotExist If the column index specified does not exist.
    \throws ColumnHasLabel If the column index specified already has a label. */
    void setColumnLabel(size_t columnIndex, const string& columnLabel) {
        throwIfColumnHasLabel(columnIndex);
        throwIfColumnLabelExists(columnLabel);

        col_ind_.emplace(columnLabel, columnIndex);
    }

    /** Label a set of columns at once using an InputIterator that produces one
    of:
    - index-label pair (std::pair<std::string, std::size_t>).
    - label (std::string). 

    In the first case, the argument \a startAtColumnIndex is ignored. In the 
    second case, the argument \a startAtColumnIndex specifies the column-index 
    of first column that will receive a label. The columns referred to must not 
    already have a label. The column labels have to be unique for the entire 
    DataTable_.

    \param first InputIterator representing beginning of the range of input
                 values. 
    \param last InputIterator representing end of the range of vallues. 
    \param startAtColumnIndex Label the columns starting at this column.

    \throws ZeroElements If the InputIterator produces zero elements.
    \throws ColumnLabelExists If a column in the DataTable_ already has the 
                              label specified by an entry produced by the 
                              iterator.
    \throws ColumnDoesNotExist In the first case above, if the column index 
                               specified by an entry produced by the iterator 
                               does not exist. In the second case above, if the 
                               (1) column index specified by 
                               'startAtColumnIndex' does not exist OR (2) the 
                               iterator produces more entries than expected in 
                               which case it attempts to label a column that 
                               does not exist.
    \throws ColumnHasLabel If the column index specified by an entry produced
                           by the iterator already has a label. */
    template<typename InputIt>
    void setColumnLabels(InputIt first, 
                         InputIt last, 
                         size_t startAtColumnIndex = 0) {
        {
        using namespace internal;
        static_assert(is_dereferencable<InputIt>, "Input iterator (InputIt) is "
                      "not dereferencable. It does not support 'operator*()'.");

        static_assert(is_eq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator==' and so is not comparable for " 
                      "equality.");

        static_assert(is_neq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator!=' and so is not comparable for " 
                      "inequality.");

        static_assert(std::is_constructible<ColumnLabels::value_type, 
                                            decltype(*first)>::value ||
                      std::is_constructible<std::string,
                                            decltype(*first)>::value, 
                      "The type of the value produced by dereferencing the "
                      "input iterator (InputIt) should be one of (1) "
                      "std::pair<std::string, size_t> (2) std::string.");
        }

        throwIfIterHasZeroElements(first, last);

        setColumnLabels_impl(first, last, startAtColumnIndex);
    }

    /** Label a set of columns using one of:
    - sequence container of labels (std::string).
    - associative container of label-index pair (std::pair<std::string, 
      std::size_t>).
 
   In first case above, \a startAtColumnIndex specifies the first column that
    will receive a new label. For the second case above, the argument
    \a startAtColumnIndex is ignored.
    Calling this function is equivalent to:
    \code
    setColumnLabels(container.begin(), container.end(), startAtColumnIndex);
    \endcode
    See overloads of setColumnLabels() taking InputIterator for detais.       */
    template<typename Container>
    void setColumnLabels(const Container& container, 
                         size_t startAtColumnIndex = 0) {
        {
        using namespace internal;
        static_assert(has_mem_begin<Container>, "Input container does not have "
                      "a member function named begin(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(has_mem_end<Container>, "Input container does not have "
                      "a member function named end(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(std::is_same<decltype(container.begin()),
                                   decltype(container.end())>::value,
                      "The member functions begin() and end() of input " 
                      "container do not produce the same type. Input container "
                      "is reuiqred to have members begin() and end() that " 
                      "return an iterator to the container.");
        }

        setColumnLabels(container.begin(), container.end(), startAtColumnIndex);
    }

    /** Label a set of columns at once starting at a given column index. See
    documenation for setColumnLabels() taking an iterator.                    */
    template<typename String>
    void setColumnLabels(const std::initializer_list<String>& columnLabels, 
                         size_t startAtColumnIndex = 0) {
        setColumnLabels(columnLabels.begin(), 
                        columnLabels.end(), 
                        startAtColumnIndex);
    }
  
    /** Get the label of a column. Time complexity is linear in the number of
    column labels. The returned value is a copy of the label. To update the 
    label of a column, use updColumnLabel(). 

    \throws ColumnHasNoLabel If the column does not have a label.
    \throws ColumnDoesNotExist If the column does not exist.                  */
    string getColumnLabel(size_t columnIndex) const {
        using ColumnLabelsValue = typename ColumnLabels::value_type;
        throwIfColumnDoesNotExist(columnIndex);

        auto res = std::find_if(col_ind_.begin(),
                                col_ind_.end(),
                                [columnIndex] (const ColumnLabelsValue& kv) {
                                    return kv.second == columnIndex;
                                });
        if(res == col_ind_.end()) {
            throw ColumnHasNoLabel{"Column " + std::to_string(columnIndex) + 
                                   " has no label."};
        }

        return res->first;
    }

    /** Get all the column labels. Returns an object that can be used in a
    range-for statement. The returned object supports begin() and end() 
    functions to retrieve begin and end iterators respectively. Dereferencing 
    the iterator will produce a pair (std::pair<std::string, std::size_t>) where
    the first element of the pair is the column label and the second element is 
    the column index. Not all columns will have labels. The result is not
    writable. Use updColumnLabel() to update column labels                    */
    ColumnLabelsContainerProxy getColumnLabels() const {
        return ColumnLabelsContainerProxy{this};
    }
    
    /** Change the label of a column. Time complexity is linear in the number of
    column labels. The column specified must already have a label. Column labels
    must be unique for the entire DataTable_. To label a column that does not 
    yet have a label, use setColumnLabel().

    \throws ColumnLabelExists If there is already a column with label specified
                              by 'newColumnLabel'.
    \throws ColumnHasNoLabel If the column specified by the column index does 
                             not already have a label.
    \throws ColumnDoesNotExist If the column specified by the column index does 
                               not exist.                                     */
    void changeColumnLabel(size_t columnIndex, 
                           const string& newColumnLabel) {
        const string old_collabel{getColumnLabel(columnIndex)};
        col_ind_.erase(old_collabel);

        throwIfColumnLabelExists(newColumnLabel);

        col_ind_.emplace(newColumnLabel, columnIndex);
    }

    /** Change the label of a column. Time complexity is constant on average and
    linear in number of column labels in the worst case.

    \throws ColumnLabelExists If there is already column with the label 
                              specified by 'newColumnLabel'.
    \throws ColumnDoesNotExist If there is no column with the specified label.*/
    void changeColumnLabel(const string& oldColumnLabel, 
                           const string& newColumnLabel) {
        const size_t colind{getColumnIndex(oldColumnLabel)};
        col_ind_.erase(oldColumnLabel);

        throwIfColumnLabelExists(newColumnLabel);

        col_ind_[newColumnLabel] = colind;
    }

    /** Change the labels of a set of columns at once using an InputIterator
    that produces one of:
    - A label-index pair (std::pair<std::string, std::size_t>) where label is 
      the new label for the column with given index.
    - A new_label-old_label pair (std::pair<std::string, std::string>).
    - A label (std::string).

    In the first and second case, the argument \a startAtColumnIndex is ignored.
    In the third case, the argument \a startAtColumnIndex specified the column-
    index of the first column that will receive a label. Calling this function
    is equivalent to one of the below calls for each value produced by the 
    iterator:
    \code
    changeColumnLabels(columnIndex, newColumnLabel)
    changeColumnLabels(oldColumnLabel, newColumnLabel)
    \endcode
    See other overloads of changeColumnLabels() for details.                  */
    template<typename InputIt>
    void changeColumnLabels(InputIt first, 
                            InputIt last, 
                            size_t startAtColumnIndex = 0) {
        {
        using namespace internal;
        static_assert(is_dereferencable<InputIt>, "Input iterator (InputIt) is "
                      "not dereferencable. It does not support 'operator*()'.");

        static_assert(is_eq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator==' and so is not comparable for " 
                      "equality.");

        static_assert(is_neq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator!=' and so is not comparable for " 
                      "inequality.");

        static_assert(std::is_constructible<ColumnLabels::value_type, 
                                            decltype(*first)>::value ||
                      std::is_constructible<std::pair<std::string, std::string>,
                                            decltype(*first)>::value ||
                      std::is_constructible<std::string,
                                            decltype(*first)>::value, 
                      "The type of the value produced by dereferencing the "
                      "input iterator (InputIt) should be one of "
                      "(1) std::pair<std::string, size_t> for newColumnLabel-"
                      "columnIndex pair. " 
                      "(2) std::pair<std::string, std::string> for "
                      "newColumnLabel-oldColumnLabel pair. "
                      "(3) std::string for newColumnLabel.");
        }

        throwIfIterHasZeroElements(first, last);

        changeColumnLabels_impl(first, last, startAtColumnIndex);
    }

    /** Change the labels of a set of columns using one of:
    - sequence container of new labels (std::string).
    - associative container of new label - column index pair 
      (std::pair<std::string, size_t>).
    - associative container of new label - old label pair
      (std::pair<std::string, std::string>).

    Calling this function is equivalent to:
    \code
    changeColumnLabels(container.begin(), container.end(), startAtColumnIndex);
    \endcode
    See overload of changeColumnLabels() taking iterators for details.        */
    template<typename Container>
    void changeColumnLabels(const Container& container, 
                            size_t startAtColumnIndex = 0) {
        {
        using namespace internal;
        static_assert(has_mem_begin<Container>, "Input container does not have "
                      "a member function named begin(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(has_mem_end<Container>, "Input container does not have "
                      "a member function named end(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(std::is_same<decltype(container.begin()),
                                   decltype(container.end())>::value,
                      "The member functions begin() and end() of input " 
                      "container do not produce the same type. Input container "
                      "is reuiqred to have members begin() and end() that " 
                      "return an iterator to the container.");
        }

        changeColumnLabels(container.begin(), 
                           container.end(), 
                           startAtColumnIndex);
    }

    /** Change the labels of a set of columns using a braced "{}" list of 
    labels. Calling this function is equivalent to calling:
    \code
    changeColumnLabels(columnLabels.begin(),
                       columnLabels.end(),
                       startAtColumnIndex)
    \endcode
    See overload of changeColumnLabels() taking iterator for details.         */
    template<typename String>
    void changeColumnLabels(const std::initializer_list<String>& columnLabels,
                            size_t startAtColumnIndex = 0) {
        changeColumnLabels(columnLabels.begin(), 
                           columnLabels.end(), 
                           startAtColumnIndex);
    }

    /** Get the index of a column from its label. Time complexity is constant on
    average and linear in number of column labels on worst case.

    \throws ColumnDoesNotExist If the column label does not exist.            */
    size_t getColumnIndex(const string& columnLabel) const {
        throwIfColumnDoesNotExist(columnLabel);

        return col_ind_.at(columnLabel);
    }

    /** Remove label for column specified by column index.

    \retval true If the column index had a label and was removed.
    \retval false If the column index did not have a label to remove.

    \throws ColumnDoesNotExist If the column specified by "columnIndex" does
                               not exist.                                     */
    bool removeColumnLabel(size_t columnIndex) {
        using ColumnLabelsValue = typename ColumnLabels::value_type;
        throwIfColumnDoesNotExist(columnIndex);

        auto res = std::find_if(col_ind_.begin(),
                                col_ind_.end(),
                                [columnIndex] (const ColumnLabelsValue& kv) {
                                    return kv.second == columnIndex;
                                });
        if(res != col_ind_.end()) {
            col_ind_.erase(res);
            return true;
        } else
            return false;
    }

    /** Remove label for column specified by column label.

    \retval true if the column label exists and was removed.
    \retval false If the column label does not exist.                         */
    bool removeColumnLabel(const string& columnLabel) {
        return col_ind_.erase(columnLabel) > 0;
    }

    /** Clear all the column labels. Data is not cleared. Only the column labels
    are cleared                                                               */
    void clearColumnLabels() {
        col_ind_.clear();
    }

    /** Get an iterator (representing the beginning) to iterate over column 
    labels. Get the sentinel iterator using columnLabelsEnd(). Dereferencing the
    iterator produces a pair (std::pair<std::string, std::size_t>) where the 
    first element is the column label and second element is the column index. 
    The result is not writable. Use updColumnLabel() to update column labels. */
    ColumnLabelsConstIter columnLabelsBegin() const {
        return col_ind_.cbegin();
    }

    /** Get an iterator (representing the end) to iterate over column labels. 
    Get the beginning iterator using columnLabelsBegin(). See 
    columnLabelsBegin() on using the iterator.                                */
    ColumnLabelsConstIter columnLabelsEnd() const {
        return col_ind_.cend();
    }

    /**@}*/
    /** \name Meta-data.
        Meta-data accessors and mutators                                      */
    /**@{*/

    /** Insert metadata. DataTable_ can hold metadata as an associative array of
    key-value pairs where is \a key is always of type std::string and \a value 
    can be of any type(except an array type[eg char[]]). The metadata inserted 
    can later be retrieved using the functions getMetaData() and updMetaData(). 
    This function throws if the \a key is already present.

    \param key A std::string that can be used the later to retrieve the inserted
               metadata.
    \param value An object/value of any type except array types(eg int[]). The 
                 code will fail to compile for array types.

    \throws MetaDataKeyExists If the specified key already exits              */
    template<typename ValueType>
    void insertMetaData(const std::string& key, ValueType&& value) {
        using namespace SimTK;
        using ValueTypeNoRef = typename std::remove_reference<ValueType>::type;

        static_assert(!std::is_array<ValueTypeNoRef>::value,
                      "'value' cannot be of array type. For ex. use std::string"
                      " instead of char[], use std::vector<int> instead of " 
                      "int[].");

        if(hasMetaData(key))
            throw MetaDataKeyExists{"Key '" + std::string{key} + 
                                    "' already exists. Remove the existing " 
                                    "entry before inserting."};

        // Type erased value.
        auto tev = new Value<ValueTypeNoRef>{std::forward<ValueType>(value)};
        metadata_.emplace(key, MetaDataValue{tev});
    }

    /** Get previously inserted metadata using its \a key and type. The template
    argument \a ValueType has to be exactly the non-reference type of the 
    metadata previously stored using insertMetaData(). The return value is a 
    read-only reference to the metadata. Use updMetaData() to obtain a writable 
    reference. Time complexity is constant on average and linear in number of 
    elements in metadata on worst case.

    \throws MetaDataKeyDoesNotExist If the key specified does not exist in 
                                    metadata.
    \throws MetaDataTypeMismatch If the type specified as template argument doe 
                                 not match the type of metadata stored under the
                                 key specified.                               */
    template<typename ValueType>
    const ValueType& getMetaData(const string& key) const {
        static_assert(!std::is_reference<ValueType>::value, 
                      "Template argument 'ValueType' should be exactly the" 
                      " non-reference type of the MetaData value stored.");

        try {
            return metadata_.at(key)->template getValue<ValueType>();
        } catch(std::out_of_range&) {
            throw MetaDataKeyDoesNotExist{"Key '" + key + "' not found."};
        } catch(std::bad_cast&) {
            throw MetaDataTypeMismatch{"Template argument specified for " 
                                       "getMetaData is incorrect."};
        }
    }

    /** Update previously inserted metadata using its \a key and type. The 
    template argument \a ValueType has to be exactly the non-reference type of 
    the metadata previously stored using insertMetaData(). The returned value is
    editable. Time complexity is constant on average and linear in number of 
    elements in metadata on worst case.

    \throws MetaDataKeyDoesNotExist If the key specified does not exist in 
                                    metadata.
    \throws MetaDataTypeMismatch If the type specified as template argument does
                                 not match the type of metadata stored under the
                                 key specified.                               */
    template<typename ValueType>
    ValueType& updMetaData(const string& key) {
        return const_cast<ValueType&>(getMetaData<ValueType>(key));
    }

    /** Pop previously inserted metadata using its \a key and type. The template
    argument \a ValueType has to be exactly the non-reference type of the 
    metadata previously inserted using insertMetaData(). The key-value pair is 
    removed from metadata and the value is returned. To simply remove the 
    key-value pair without retrieving the value, use removeMetaData(). Time 
    complexity is constant on average and linear in number of elements in the 
    metadata on worst case.

    \throws MetaDataKeyDoesNotExist If the key specified does not exist in 
                                    metadata.
    \throws MetaDataTypeMismatch If the type specified as template argument does
                                 not match the type of metadata stored under the
                                 key specified.                               */
    template<typename ValueType>
    ValueType popMetaData(const string& key) {
        ValueType value{std::move(updMetaData<ValueType>(key))};
        metadata_.erase(key);
        return value;
    }

    /** Remove a metadata key-value pair previously inserted using the \a key. 

    \retval true If there was a removal. 
    \retval false If the key was not found in metadata. 

    Time complexity is constant on average and linear in number of elements in 
    the metadata on worst case.                                               */
    bool removeMetaData(const string& key) {
        return metadata_.erase(key);
    }

    /** Clear the metadata. All the metadata will be lost with this operation.*/
    void clearMetaData() {
        metadata_.clear();
    }

    /** Check if metadata for a given key exists. Time complexity is constant on
    average and linear in the number of elements in the metadata on worst 
    case.                                                                     */
    bool hasMetaData(const string& key) const {
        return metadata_.find(key) != metadata_.end();
    }

    /** Check if metadata is empty -- if the number of elements is zero.      */
    bool isMetaDataEmpty() const {
        return metadata_.empty();
    }

    /** Get the number of elements in the metadata. Time complexity of other 
    operations on metadata depend on this number.                             */
    size_t getMetaDataSize() const {
        return metadata_.size();
    }

    /**@}*/

protected:
    /** \cond */
    // Helper functions. Only one overload is enabled depending on the type of 
    // the value iterator produces.
    template<typename InputIt>
    void setColumnLabels_impl(InputIt first, 
    typename std::enable_if<std::is_constructible<ColumnLabels::value_type, 
                                                  decltype(*first)>::value, 
                              InputIt>::type last, 
                              size_t) {
        while(first != last) {
            throwIfColumnHasLabel(first->second);
            throwIfColumnLabelExists(first->first);

            col_ind_.emplace(*first);
            ++first;
        }
    }
    template<typename InputIt>
    void setColumnLabels_impl(InputIt first,
    typename std::enable_if<std::is_constructible<std::string,
                                                  decltype(*first)>::value,
                              InputIt>::type last,
                              size_t startAtColumnIndex) {
        size_t columnIndex{startAtColumnIndex};
        while(first != last) {
            throwIfColumnHasLabel(columnIndex);
            throwIfColumnLabelExists(*first);

            col_ind_.emplace(*first, columnIndex);
            ++first; ++columnIndex;
        }
    }

    // Helper functions. Only one overload is enabled depending on the type of 
    // the value iterator produces.
    template<typename InputIt>
    void changeColumnLabels_impl(InputIt first, 
    typename std::enable_if<std::is_constructible<ColumnLabels::value_type, 
                                                  decltype(*first)>::value ||
                            std::is_constructible<std::pair<std::string, 
                                                            std::string>,
                                                  decltype(*first)>::value, 
                                 InputIt>::type last, 
                                 size_t) {
        while(first != last) {
            changeColumnLabel(first->second, first->first);
            ++first;
        }
    }
    template<typename InputIt>
    void changeColumnLabels_impl(InputIt first,
    typename std::enable_if<std::is_constructible<std::string,
                                                  decltype(*first)>::value,
                              InputIt>::type last,
                              size_t startAtColumnIndex) {
        size_t columnIndex{startAtColumnIndex};
        while(first != last) {
            changeColumnLabel(columnIndex, *first);
            ++first; ++columnIndex;
        }
    }

    // Helper function. Check if a column exists and throw an exception if it
    // does not.
    void throwIfColumnDoesNotExist(const size_t columnIndex) const {
        if(!hasColumn(columnIndex)) {
            throw ColumnDoesNotExist{"Column " + std::to_string(columnIndex) + 
                                     " does not exist. Index out of range."};
        }
    }

    // Helper function. Check if a column exists and throw an exception if it
    // does not.
    void throwIfColumnDoesNotExist(const std::string& columnLabel) const {
        if(!hasColumnLabel(columnLabel)) {
            throw ColumnDoesNotExist{"No Column with label '" + columnLabel + 
                                     "'."};
        }
    }

    // Helper function. Check if a column has label and throw an exception if it
    // does.
    void throwIfColumnHasLabel(const size_t columnIndex) const {
        if(columnHasLabel(columnIndex)) {
            throw ColumnHasLabel{"Column " + std::to_string(columnIndex) + 
                                 " already has a label."};
        }
    }

    // Helper function. Check if a column label exists and throw an exception if
    // it does.
    void throwIfColumnLabelExists(const std::string& columnLabel) const {
        if(hasColumnLabel(columnLabel))
            throw ColumnLabelExists{"A column with label '" + columnLabel + 
                                    "' already exists. Column labels have to be"
                                    " unique."};
    }

    // Helper function. Throw if the iterators produce zero elements.
    template<typename Iter>
    void throwIfIterHasZeroElements(const Iter& first, const Iter& last) const {
        if(first == last)
            throw ZeroElements{"Input iterators produce zero elements."};
    }

    // Meta-data.
    MetaData     metadata_;
    // Column label to column index.
    ColumnLabels col_ind_;

    /** \endcond */
}; // AbstractDataTable


/** Enum to specify dimension of data traversal -- row-wise or olumn-wise.    */
enum class TraverseDir {
    RowMajor, 
    ColumnMajor
};


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
template<typename ET = SimTK::Real>
class DataTable_ : public AbstractDataTable {
    static_assert(std::is_arithmetic<ET>::value || std::is_class<ET>::value, 
                  "Template parameter ET must be either an arithmetic type " 
                  "(int, float, double etc) or a class/struct type.");
protected:
    /** \cond */
    // Iterator to support iterating over rows and columns of the DataTable_.
    // The template paramters determine whether the iteration is over rows or
    // columns and whether the iterator allows editing the row/column.
    // RowOrColIter -- Set to true for iterating over rows. 
    //                 Set to false for iterating over columns.
    // IsConst -- Set to true to produce a 'const' iterator. Does not allow
    //            writing to the row/column when dereferenced.
    //            Set to false to produce a 'non-const' iterator. Does allow
    //            writing to the row/column when dereferenced.
    // The iterator provides random access to rows and columns.
    template<bool RowOrColIter, bool IsConst>
    class Iterator {
    public:
        using value_type        = 
            typename std::conditional<RowOrColIter, 
                                      SimTK::RowVectorView_<ET>,
                                      SimTK::VectorView_<ET>>::type;
        using reference         = value_type&;
        using pointer           = value_type*;
        using difference_type   = int;
        using iterator_category = std::random_access_iterator_tag;

        Iterator()                           = delete;
        Iterator(const Iterator&)            = default;
        Iterator(Iterator&&)                 = default;
        Iterator& operator=(const Iterator&) = default;
        Iterator& operator=(Iterator&&)      = default;
        ~Iterator()                          = default;

        Iterator(DataTable_* dt, size_t index) 
            : dt_{dt}, index_{index} {}
        Iterator(const DataTable_* dt, size_t index) 
            : dt_{dt}, index_{index} {}

        value_type operator*() {
            return rowOrCol<RowOrColIter, IsConst>(index_);
        }

        Iterator& operator++() {
            ++index_;
            return *this;
        }

        bool operator!=(const Iterator& rhs) const {
            if(dt_ != rhs.dt_)
                throw IncompatibleIterators{"The iterators are for two "
                        "different DataTables."};
                
            return index_ != rhs.index_;
        }

        bool operator==(const Iterator& rhs) const {
            return !operator!=(rhs);
        }

        Iterator& operator+=(int n) {
            index_ += n;
            return *this;
        }

        Iterator operator+(int n) const {
            return Iterator{dt_, index_ + n};
        }

        Iterator& operator-=(int n) {
            return operator+=(-1 * n);
        }

        Iterator operator-(int n) const {
            return operator+(-1 * n);
        }

        int operator-(const Iterator& rhs) const {
            if(dt_ != rhs.dt_)
                throw IncompatibleIterators{"The iterators are for two "
                        "different DataTables."};

            return index_ - rhs.index_;
        }

        value_type operator[](size_t index) {
            return rowOrCol<RowOrColIter, IsConst>(index);
        }

        bool operator<(const Iterator& rhs) const {
            if(dt_ != rhs.dt_)
                throw IncompatibleIterators{"The iterators are for two "
                        "different DataTables."};

            return index_ < rhs.index_;
        }

        bool operator>(const Iterator& rhs) const {
            if(dt_ != rhs.dt_)
                throw IncompatibleIterators{"The iterators are for two "
                        "different DataTables."};
            
            return index_ > rhs.index_;
        }

        bool operator>=(const Iterator& rhs) const {
            if(dt_ != rhs.dt_)
                throw IncompatibleIterators{"The iterators are for two "
                        "different DataTables."};
            
            return index_ >= rhs.index_;
        }

        bool operator<=(const Iterator& rhs) const {
            if(dt_ != rhs.dt_)
                throw IncompatibleIterators{"The iterators are for two "
                        "different DataTables."};
            
            return index_ <= rhs.index_;
        }

    private:
        // Helper functions. Only one overload is enabled for every combination 
        // of first two template parameters.
        template<bool RowOrCol, bool GetOrUpd, 
                 typename std::enable_if<RowOrCol && GetOrUpd, int>::type = 0>
        value_type rowOrCol(size_t index) const {
            return dt_->getRow(index);
        }
        template<bool RowOrCol, bool GetOrUpd, 
                 typename std::enable_if<RowOrCol && !GetOrUpd, int>::type = 0>
        value_type rowOrCol(size_t index) {
            return dt_->updRow(index);
        }
        template<bool RowOrCol, bool GetOrUpd, 
                 typename std::enable_if<!RowOrCol && GetOrUpd, int>::type = 0>
        value_type rowOrCol(size_t index) const {
            return dt_->getColumn(index);
        }
        template<bool RowOrCol, bool GetOrUpd, 
                 typename std::enable_if<!RowOrCol && !GetOrUpd, int>::type = 0>
        value_type rowOrCol(size_t index) {
            return dt_->updColumn(index);
        }
        
        typename std::conditional<IsConst, 
                                  const DataTable_*, 
                                  DataTable_*>::type dt_;
        size_t index_;
    };

    // Rows or Columns container proxy. 
    template<bool RowsOrColsContainer, bool IsConst>
    class RowsOrColsContainerProxy {
    public:
        RowsOrColsContainerProxy(DataTable_* dt) : dt_{dt} {}
        RowsOrColsContainerProxy(const DataTable_* dt) : dt_{dt} {}
        RowsOrColsContainerProxy()                                    = delete;
        RowsOrColsContainerProxy(const RowsOrColsContainerProxy&)     = default;
        RowsOrColsContainerProxy(RowsOrColsContainerProxy&&)          = default;
        RowsOrColsContainerProxy& operator=(const RowsOrColsContainerProxy&)
                                                                      = default;
        RowsOrColsContainerProxy& operator=(RowsOrColsContainerProxy&&) 
                                                                      = default;

        Iterator<RowsOrColsContainer, IsConst> begin() {
            return Iterator<RowsOrColsContainer, IsConst>{dt_, 0};
        }

        Iterator<RowsOrColsContainer, IsConst> end() {
            return Iterator<RowsOrColsContainer, 
                            IsConst>{dt_, size<RowsOrColsContainer>()};
        }

    private:
        // Helper functions. Only one overload is enabled for every value of
        // first template parameter.
        template<bool RowOrCol, 
                 typename std::enable_if<RowOrCol, int>::type = 0>
        size_t size() const {
            return dt_->getNumRows();
        }
        template<bool RowOrCol, 
                 typename std::enable_if<!RowOrCol, int>::type = 0>
        size_t size() const {
            return dt_->getNumColumns();
        }

        typename std::conditional<IsConst, 
                                  const DataTable_*, 
                                  DataTable_*>::type dt_;
    };
    /** \endcond */

public:
    using value_type    = ET;
    using size_type     = size_t;

    /** \name Create.
        Constructors.                                                         */
    /**@{*/

    /** Construct empty DataTable.                                            */
    DataTable_() = default;

    /** Construct DataTable_ with size [\a numRows x \a numColumns] where each
    entry is initialized with initialValue. Default value for initialValue is 
    \a NaN.
  
    \param numRows Number of rows.
    \param numColumns Number of columns.
    \param initialValue Value to initialize all the entries to.               */
    DataTable_(size_t numRows,
               size_t numColumns,
               const ET& initialValue = ET{SimTK::NaN}) 
        : data_{static_cast<int>(numRows), 
                static_cast<int>(numColumns), 
                initialValue} {}

    /** Construct DataTable_ using an InputIterator which produces one entry (of
    type convertible to type ET) at a time when dereferenced. The entries of 
    DataTable_ are copy initialized using the values produced by the iterator. 
    The DataTable_ can be populated either row-wise or column-wise by specifying
    \a rowMajor or \a columnMajor for the parameter \a traverseDir. Either way,
    the constructor needs to know the length of each row/column from the 
    paramter \a numEntriesInMajor' so it can split the values coming from the 
    iterator into rows/columns. For example, specifying \a RowMajor for 
    \a traverseDir and 10 for \a numEntriesInMajor will populate the DataTable_
    one row at a time with each row's length taken to be 10. If/when the 
    iterator falls short of producing enough values to fill up rows/columns 
    completely, an exception is thrown, which can disabled using 
    \a allowMissing. For the previous example, if the iterator produces say 26 
    elements in total, then rows 0 and 1 get all 10 elements filled up but row 3
    will only get 6 elements. At this point, an exception is thrown if \a 
    allowMissing is false. The constructor also accepts an optional argument \a 
    numMajors, which when specified will speed up the construction of the 
    DataTable_ by allocating enough memory for the entire DataTable_ at once 
    instead of allocating and relocating for every new row added. For the 
    previous example, if it is known that the iterator will produce 50 elements
    in total, with rows of length 10, there were be a total of 5 rows, which
    can be specified using \a numMajors. 
    See <a href="http://en.cppreference.com/w/cpp/concept/InputIterator">this 
    page</a> for details on InputIterator.
      
    \param first Beginning of range covered by the iterator. 
    \param last End of the range covered by the iterator. 
    \param traverseDir Whether to populate the DataTable_ row-wise or 
                       column-wise.  Possible values are:
                       - RowMajor -- Populate the DataTable_ one row at a time.
                       - ColumnMajor -- Populate the DataTable_ one column at a 
                         time.
    \param numEntriesInMajor If \a traverseDir is:
                             - \a RowMajor, then this is the length of each row 
                               in the DataTable_.
                             - \a ColumnMajor, then this is the length of each
                               column in the DataTable_.
    \param allowMissing Allow for missing values. 
                        - false -- NotEnoughElements will be thrown if the input
                          iterator fills up the last row/column only partially. 
                        - true -- No exception thrown if the input iterator
                          fills up the last row/column only partially. Instead,
                          missing elements are set to SimTK::NaN.
    \param numMajors Optional argument for speed. If \a traverseDir is:
                     - \a RowMajor, then this is the number of rows. If the 
                       number of rows produced by the iterator is known in
                       advance, it an be specified here.
                     - \a ColumnMajor, then this is the number of columns. If 
                       the number of columns produced by the iterator is known 
                       in advance, it can be specified here.
  
    \throws ZeroElements When input-iterator does not produce any elements.
                         That is first == last.                                 
    \throws InvalidEntry When the required input argument \a numEntriesInMajor 
                         is zero.
    \throws TooManyElements If \a numMajors is specified and input iterator
                            produces more elements than needed to construct the
                            DataTable_, this exception is thrown.
    \throws NotEnoughElements The argument allowMissing enables/disables this
                              exception. When enabled, if \a traverseDir == \a 
                              RowMajor, this exception is thrown when the input 
                              iterator does not produce enough elements to fill
                              up all the rows completely. If \a traverseDir == 
                              \a ColumnMajor, this exception is thrown when the
                              input iterator does not produce enough elements to
                              fill up all the columns completely.             */
    template<typename InputIt, 
    typename = typename std::enable_if<!std::is_integral<InputIt>::value>::type>
    DataTable_(InputIt first,
               InputIt last,
               size_t numEntriesInMajor,
               TraverseDir traverseDir = TraverseDir::RowMajor,
               bool allowMissing     = false,
               size_t numMajors      = 0) {
        {
        using namespace internal;
        static_assert(is_dereferencable<InputIt>, "Input iterator (InputIt) is "
                      "not dereferencable. It does not support 'operator*()'.");

        static_assert(std::is_constructible<ET, decltype(*first)>::value, 
                      "The type of the value produced by dereferencing the "
                      "input iterator (InputIt) does not match template "
                      "parameter ET used to instantiate DataTable.");

        static_assert(is_eq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator==' and so is not comparable for " 
                      "equality.");

        static_assert(is_neq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator!=' and so is not comparable for " 
                      "inequality.");
        }

        throwIfIterHasZeroElements(first, last);

        if(numEntriesInMajor == 0)
            throw InvalidEntry{"Input argument 'numEntriesInMajor' is required "
                    " cannot be zero."};

        // Optimization. If numMajors is specified, pre-size the data and 
        // avoid having to resize it multiple times later.
        if(numMajors != 0) {
            if(traverseDir == TraverseDir::RowMajor)
                data_.resize(static_cast<int>(numMajors), 
                             static_cast<int>(numEntriesInMajor));
            else if(traverseDir == TraverseDir::ColumnMajor)
                data_.resize(static_cast<int>(numEntriesInMajor), 
                             static_cast<int>(numMajors));
        } else {
            if(traverseDir == TraverseDir::RowMajor)
                data_.resize(1, static_cast<int>(numEntriesInMajor));
            else if(traverseDir == TraverseDir::ColumnMajor)
                data_.resize(static_cast<int>(numEntriesInMajor), 1);
        }
            
        int row{0};
        int col{0};
        while(first != last) {
            data_.set(row, col, *first);
            ++first;
            if(traverseDir == TraverseDir::RowMajor) {
                ++col;
                if(col == static_cast<int>(numEntriesInMajor) && 
                   first != last) {
                    col = 0;
                    ++row;
                    if(numMajors == 0)
                        data_.resizeKeep(data_.nrow() + 1, data_.ncol());
                    else if(row == static_cast<int>(numMajors))
                        throw TooManyElements{"Input iterator produced more "
                                "elements than needed to fill " + 
                                std::to_string(numMajors) + " (numMajors) " 
                                "rows."};
                }
            } else {
                ++row;
                if(row == static_cast<int>(numEntriesInMajor) && 
                   first != last) {
                    row = 0;
                    ++col;
                    if(numMajors == 0)
                        data_.resizeKeep(data_.nrow(), data_.ncol() + 1);
                    else if(col == static_cast<int>(numMajors))
                        throw TooManyElements{"Input iterator produced more "
                                "elements than needed to fill " + 
                                std::to_string(numMajors) + " (numMajors) "
                                "columns."};
                }
            }
        }

        if(!allowMissing) {
            if(traverseDir == TraverseDir::RowMajor) {
                if(numMajors != 0 && row != data_.nrow() - 1)
                    throw NotEnoughElements{"Input iterator did not produce "
                            "enough elements to fill all the rows. Total rows ="
                            " " + std::to_string(data_.nrow()) + " Filled rows "
                            "= " + std::to_string(row) + "."};
                if(col != data_.ncol())
                    throw NotEnoughElements{"Input iterator did not produce " 
                            "enough elements to fill the last row. Expected = " 
                            + std::to_string(data_.ncol()) + ", Received = " + 
                            std::to_string(col)};
            } else if(traverseDir == TraverseDir::ColumnMajor) {
                if(numMajors != 0 && col != data_.ncol() - 1)
                    throw NotEnoughElements{"Input iterator did not produce "
                            "enough elements to fill all the columns. Total "
                            "columns = " + std::to_string(data_.ncol()) + 
                            " Filled columns = " + std::to_string(col) + "."};
                if(row != data_.nrow())
                    throw NotEnoughElements{"Input iterator did not produce "
                            "enough elements to fill the last column. Expected "
                            "= " +  std::to_string(data_.nrow()) + ", "
                            "Received = " + std::to_string(row)};
            }
        }
    }

    /** Construct DataTable_ using a container holding elements of type ET
    (template paramter). The entries of the DataTable_ are copy initialized 
    using the values in the container. The container is required to support an 
    iterator. In other words, the container must have member functions %begin()
    and %end() that emit an iterator to the container. Calling this constructor
    is equivalent to calling the constructor taking an iterator:
    \code
    DataTable_{container.begin(), 
               container.end(), 
               numEntriesInMajor,
               traverseDir,
               allowMissing,
               numMajors};
    \endcode
    For details, see documentation for constructor taking iterator.
    
    Besides being a convenience wrapper for the constructor taking iterator, 
    this constructor will _try_ to apply an optimization by inquiring the number
    of elements in the container by calling the member function %size() on the 
    container, if the container has such a member function. Otherwise, this 
    function is nothing more than a wrapper. Knowing the total number of 
    elements allows this constructor to calculate the argument \a numMajors 
    automatically and apply optimization by allocating memory for the entire 
    DataTable_ once and populating it using elements of the container.        
    If \a numMajors is specified in the call, optimization is applied 
    using that value but container size, if available, is still used to perform
    some error checking.                                                      */
    template<typename Container>
    DataTable_(const Container& container,
               size_t numEntriesInMajor,
               TraverseDir traverseDir = TraverseDir::RowMajor,
               bool allowMissing     = false,
               size_t numMajors      = 0) {
        {
        using namespace internal;
        static_assert(has_mem_begin<Container>, "Input container does not have "
                      "a member function named begin(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(has_mem_end<Container>, "Input container does not have "
                      "a member function named end(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(std::is_same<decltype(container.begin()),
                                   decltype(container.end())>::value,
                      "The member functions begin() and end() of input " 
                      "container do not produce the same type. Input container "
                      "is reuiqred to have members begin() and end() that " 
                      "return an iterator to the container.");
        }

        DataTable__impl(container, 
                        numEntriesInMajor, 
                        traverseDir, 
                        allowMissing, 
                        numMajors);
    }
    
    /**@}*/
    /** \name Copy.
        Copy operations including copy constructor.                           */
    /**@{*/

    /** Copy constructor.                                                     */
    DataTable_(const DataTable_&) = default;

    /** Virtual copy constructor.                                             */
    std::unique_ptr<AbstractDataTable> clone() const override {
        return std::unique_ptr<AbstractDataTable>(new DataTable_{*this});
    }

    /** Copy assignment                                                       */
    DataTable_& operator=(const DataTable_&) = default;

    /**@}*/
    /** \name Move.
        Move operations.                                                      */
    /**@{*/

    /** Move constructor.                                                     */
    DataTable_(DataTable_&&) = default;

    /** Move assignment                                                       */
    DataTable_& operator=(DataTable_&&) = default;

    /**@}*/
    /** \name Destroy.
        Destructor.                                                           */
    /**@{*/

    /** Destructor.                                                           */
    ~DataTable_() override = default;

    /**@}*/
    /** \name Data.
        Data accessors & mutators.                                            */
    /**@{*/

    /** Get number of rows in the DataTable_.                                 */
    size_t getNumRows() const {
        return static_cast<size_t>(data_.nrow()); 
    }

    /** Get number of columns in the DataTable_.                              */
    size_t getNumColumns() const {
        return static_cast<size_t>(data_.ncol()); 
    }

    /** Get a sub-matrix (or block) of the DataTable_. Returned object is not
    writable. Use updMatrix() to obtain a writable reference. For more 
    information on using the result, see SimTK::MatrixView_.                  

    \throws RowDoesNotExist If the row specified by either rowStart or 
                            [rowStart + numRows - 1] does not exist.
    \throws ColumnDoesNotExist If the column specified by either columnStart or
                               [columnStart + numColumns - 1] does not exist. */
    SimTK::MatrixView_<ET> getMatrix(size_t rowStart, 
                                     size_t columnStart,
                                     size_t numRows,
                                     size_t numColumns) const {
        throwIfRowDoesNotExist(rowStart);
        throwIfRowDoesNotExist(rowStart + numRows - 1);
        throwIfColumnDoesNotExist(columnStart);
        throwIfColumnDoesNotExist(columnStart + numColumns - 1);
        
        return data_.block(static_cast<int>(rowStart), 
                           static_cast<int>(columnStart), 
                           static_cast<int>(numRows), 
                           static_cast<int>(numColumns));
    }

    /** Get a sub-matrix (or block) of the DataTable_. Returned object is 
    writable. For more information on using the result, see 
    SimTK::MatrixView_.                                                       

    \throws RowDoesNotExist If the row specified by either rowStart or 
                            [rowStart + numRows - 1] does not exist.
    \throws ColumnDoesNotExist If the column specified by either columnStart or
                               [columnStart + numColumns - 1] does not exist. */
    SimTK::MatrixView_<ET> updMatrix(size_t rowStart, 
                                     size_t columnStart,
                                     size_t numRows,
                                     size_t numColumns) {
        throwIfRowDoesNotExist(rowStart);
        throwIfRowDoesNotExist(rowStart + numRows - 1);
        throwIfColumnDoesNotExist(columnStart);
        throwIfColumnDoesNotExist(columnStart + numColumns - 1);

        return data_.updBlock(static_cast<int>(rowStart), 
                              static_cast<int>(columnStart), 
                              static_cast<int>(numRows), 
                              static_cast<int>(numColumns));
    }

    /** Get a row of the DataTable_ by index. Returned row is read-only. Use 
    updRow() to obtain a writable reference. See SimTK::RowVectorView_ for more
    details.
  
    \throws RowDoesNotExist If the row specified by either rowIndex does not 
                            exist.                                            */
    SimTK::RowVectorView_<ET> getRow(size_t rowIndex) const {
        throwIfRowDoesNotExist(rowIndex);
        return data_.row(static_cast<int>(rowIndex));
    }

    /** Get a row of the DataTable_ by index. Returned row is editable. See 
    SimTK::RowVectorView_ for more details.
  
    \throws RowDoesNotExist If the row specified by either rowIndex does not 
                            exist.                                            */
    SimTK::RowVectorView_<ET> updRow(size_t rowIndex) {
        throwIfRowDoesNotExist(rowIndex);
        return data_.updRow(static_cast<int>(rowIndex));
    }

    /** Get a column of the DataTable_ by index. Returned column is read-only. 
    Use updColumn() to obtain a writable reference. See SimTK::VectorView_ for 
    more details.
  
    \throws ColumnDoesNotExist If the column specified by columnIndex does not
                               exist.                                         */
    SimTK::VectorView_<ET> getColumn(size_t columnIndex) const {
        throwIfColumnDoesNotExist(columnIndex);
        return data_.col(static_cast<int>(columnIndex));
    }

    /** Get a column of the DataTable_ by label. Returned column is read-only. 
    Use updColumn to obtain a writable reference. See SimTK::VectorView_ for 
    more details.
      
    \throws ColumnDoesNotExist If the column label specified is not in the 
                               DataTable_.                                    */
    SimTK::VectorView_<ET> getColumn(const string& columnLabel) const {
        return data_.col(static_cast<int>(getColumnIndex(columnLabel)));
    }

    /** Get a column of the DataTable_ by index. Returned column is editable. 
    See SimTK::VectorView_ for more details.
  
    \throws ColumnDoesNotExist If the column specified by columnIndex does not
                               exist.                                         */
    SimTK::VectorView_<ET> updColumn(size_t columnIndex) {
        throwIfColumnDoesNotExist(columnIndex);
        return data_.updCol(static_cast<int>(columnIndex));
    }

    /** Get a column of the DataTable_ by label. Returned column is editable. 
    See SimTK::VectorView_ for more details.
  
    \throws ColumnDoesNotExist If the column label specified is not in the 
                               DataTable_.                                    */
    SimTK::VectorView_<ET> updColumn(const string& columnLabel) {
        return data_.updCol(static_cast<int>(getColumnIndex(columnLabel)));
    }

    /** Get an element of the DataTable_ by its index-pair(row, column). The 
    returned element is read-only. use updElt() to get a writable reference.

    \throws RowDoesNotExist If the row specified by rowIndex does not exist.
    \throws ColumnDoesNotExist If the column specified by columnIndex does not
                               exist                                          */
    const ET& getElt(size_t rowIndex, size_t columnIndex) const {
        throwIfRowDoesNotExist(rowIndex);
        throwIfColumnDoesNotExist(columnIndex);
        return data_.getElt(static_cast<int>(rowIndex), 
                            static_cast<int>(columnIndex));
    }

    /** Get an element of the DataTable_ by (row, column-label) pair. The 
    returned element is read-only. use updElt to get a writable reference.
   
    \throws RowDoesNotExist If the row specified by rowIndex does not exist.
    \throws ColumnDoesNotExist If the column label specified is not in the 
                               DataTable_.                                    */
    const ET& getElt(size_t rowIndex, const string& columnLabel) const {
        throwIfRowDoesNotExist(rowIndex);
        return data_.getElt(static_cast<int>(rowIndex), 
                            static_cast<int>(getColumnIndex(columnLabel)));
    }

    /** Get an element of the DataTable_ by its index-pair(row, column). The 
    returned element is editable.
  
    \throws RowDoesNotExist If the row specified by rowIndex does not exist.
    \throws ColumnDoesNotExist If the column specified by columnIndex does not
                               exist                                          */
    ET& updElt(size_t rowIndex, size_t columnIndex) {
        throwIfRowDoesNotExist(rowIndex);
        throwIfColumnDoesNotExist(columnIndex);
        return data_.updElt(static_cast<int>(rowIndex), 
                            static_cast<int>(columnIndex));
    }

    /** Get an element of the DataTable_ by (row, column-label) pair. The 
    returned element is editable.

    \throws RowDoesNotExist If the row specified by rowIndex does not exist.
    \throws ColumnDoesNotExist If the column label specified is not in the 
                               DataTable_.                                    */
    ET& updElt(size_t rowIndex, const string& columnLabel) {
        throwIfRowDoesNotExist(rowIndex);
        return data_.updElt(static_cast<int>(rowIndex), 
                            static_cast<int>(getColumnIndex(columnLabel)));
    }

    /** Get a *copy* of the underlying matrix of the DataTable_.              */
    SimTK::Matrix_<ET> copyAsMatrix() const {
        return *(new SimTK::Matrix_<ET>{data_});
    }

    /** Add (append) a row to the DataTable_ using a SimTK::RowVector_. If the 
    DataTable is empty, input row will be the first row. This function can be
    used to populate an empty DataTable_.
  
    \param row The row to be added as a SimTK::RowVector_.
  
    \throws ZeroElements If number of elements in the input row is zero.
    \throws NumberOfColsMismatch If number of columns in the input row does not 
                                 match the number of columns of the 
                                 DataTable_.                                  */
    void addRow(const SimTK::RowVector_<ET>& row) {
        if(row.nrow() == 0 || row.ncol() == 0)
            throw ZeroElements{"Input row has zero length."};
        if(data_.ncol() > 0 && row.size() != data_.ncol())
            throw NumberOfColumnsMismatch{"Input row has incorrect number of " 
                                          "columns. Expected = " + 
                                          std::to_string(data_.ncol()) + 
                                          " Received = " + 
                                          std::to_string(row.size())};

        data_.resizeKeep(data_.nrow() + 1, row.ncol());
        data_.updRow(data_.nrow() - 1).updAsRowVector() = row;
    }

    /** Add (append) a row to the DataTable_ using an InputIterator producing 
    one entry at a time. If this function is called on an empty DataTable_ 
    without providing numColumnsHint, it performs <i>allocation + relocation</i>
    for [log2(ncol) + 1] times where ncol is the actual number of elements 
    produced by the InputIterator. To add multiple rows at once using an 
    InputIterator, use addRows().
  
    \param first Beginning of range covered by the iterator.
    \param last End of the range covered by the iterator.
    \param numColumnsHint Hint for the number of columns in the input iterator. 
                          Can be approximate above or below the actual number. 
                          This is only used when this function is called on an 
                          empty DataTable_. Ignored otherwise. Providing a hint 
                          reduces the number of resize operations which involves
                          memory allocation and relocation.
    \param allowMissing Allow for missing values. Enables/disables the exception
                        NotEnoughElements. This is only used when DataTable_ is
                        non-empty. If this function is called on an empty
                        DataTable_, this argument is ignored.
                        - false -- exception is thrown if the input iterator 
                          fills up the row only partially. 
                        - true -- exception is not thrown even if the input
                        iterator fills up the row only partially. Instead,
                        missing values are set to SimTK::NaN.
  
    \throws ZeroElements If the number of elements produced by the input 
                         iterator is zero.
    \throws InvalidEntry The DataTable is empty and required the input argument 
                         'numColumnsHint' is zero.
    \throws TooManyElements When called on non-empty DataTable_, if the input
                            iterator produces more elements than needed to add
                            a row.
    \throws NotEnoughElements Argument allowMissing enables/disables this 
                              exception. This exception is applicable only
                              when this function is called on a non-empty 
                              DataTable_. When enabled, this exception is thrown
                              if the InputIterator does not produce enough
                              elements to fill up the entire row.             */
    template<typename InputIt>
    void addRow(InputIt first, 
                InputIt last, 
                size_t numColumnsHint = 2,
                bool allowMissing = false) {
        {
        using namespace internal;
        static_assert(is_dereferencable<InputIt>, "Input iterator (InputIt) is "
                      "not dereferencable. It does not support 'operator*()'.");

        static_assert(std::is_constructible<ET, decltype(*first)>::value, 
                      "The type of the value produced by dereferencing the "
                      "input iterator (InputIt) does not match template "
                      "parameter ET used to instantiate DataTable.");

        static_assert(is_eq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator==' and so is not comparable for " 
                      "equality.");

        static_assert(is_neq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator!=' and so is not comparable for " 
                      "inequality.");
        }

        throwIfIterHasZeroElements(first, last);

        if((data_.nrow() == 0 || data_.ncol() == 0) && numColumnsHint == 0)
            throw InvalidEntry{"Input argument 'numColumnsHint' cannot be zero "
                               "when DataTable is empty."};

        if(data_.nrow() > 0 && data_.ncol() > 0) {
            data_.resizeKeep(data_.nrow() + 1, data_.ncol());
            int col{0};
            while(first != last) {
                if(col == data_.ncol())
                    throw TooManyElements{"Input iterator produced more "
                            "elements than needed to fill a row with " + 
                            std::to_string(data_.ncol()) + " columns."};

                data_.set(data_.nrow() - 1, col++, *first);
                ++first;
            }
            if(!allowMissing && col != data_.ncol())
                throw NotEnoughElements{"Input iterator did not produce enough "
                                        "elements to fill the row. Expected = " 
                                        + std::to_string(data_.ncol()) + 
                                        " Received = " + std::to_string(col)};
        } else {
            int col{0};
            size_t ncol{numColumnsHint};
            data_.resizeKeep(1, static_cast<int>(ncol));
            while(first != last) {
                data_.set(0, col++, *first);
                ++first;
                if(col == static_cast<int>(ncol) && first != last) {
                    // If ncol is a power of 2, double it. Otherwise round it to
                    // the next power of 2.
                    ncol = (ncol & (ncol - 1)) == 0 ? 
                           ncol << 2 : rndToNextPowOf2(ncol); 
                    data_.resizeKeep(1, static_cast<int>(ncol));
                }
            }
            if(col != static_cast<int>(ncol))
                data_.resizeKeep(1, col);
        }
    }

    /** Add (append) a row to the DataTable_ using a container holding values
    of a type ET (template parameter). The container is required to support
    an iterator. In other words, the container must have member functions 
    %begin() and %end() that emit an iterator to the container. Calling this
    function is equivalent to calling addRow() taking an iterator:
    \code
    addRow(container.begin(),
           container.end(),
           numColumnsHint,
           allowMissing);
    \endcode
    For details, see documentation for addRow() taking an iterator.

    Besides being a convenience wrapper for addRow() taking an iterator, this
    function will _try_ to apply an optimization when it called to add a row to
    an empty DataTable_. The function will _try_ to inquire the number of 
    elements in the container by calling the member function %size() on the 
    container, if the container has such a member function. Otherwise, this 
    function is nothing more than a convenience wrapper. Knowing the total 
    number of elements allows this function to know the length of the row being 
    added and apply optimization by allocating memory for the row once and 
    populating it using the elements of the container.                          
    If \a numColumnsHint is specified in the call, optimization is applied using
    that value but container size, if available, is still used to perform some 
    error checking.                                                           */
    template<typename Container>
    void addRow(const Container& container, 
                size_t numColumnsHint = 2,
                bool allowMissing = false) {
        {
        using namespace internal;
        static_assert(has_mem_begin<Container>, "Input container does not have "
                      "a member function named begin(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(has_mem_end<Container>, "Input container does not have "
                      "a member function named end(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(std::is_same<decltype(container.begin()),
                                   decltype(container.end())>::value,
                      "The member functions begin() and end() of input " 
                      "container do not produce the same type. Input container "
                      "is reuiqred to have members begin() and end() that " 
                      "return an iterator to the container.");
        }

        addRow_impl(container, numColumnsHint, allowMissing);
    }

    /** Add (append) multiple rows to the DataTable_ using an InputIterator 
    producing one entry at a time. If this function is called on an empty 
    DataTable_, numColumns must be provided. Otherwise, numColumns is ignored. 
    To add just one row, use addRow() instead.
    See also documentation for the constructor of DataTable_ taking an iterator.

    \param first Beginning of range covered by the iterator.
    \param last End of the range covered by the iterator.
    \param numColumns Number of columns to create in the DataTable_. This is 
                      only used (and required) when the function is called on 
                      an empty DataTable_. Ignored otherwise.
    \param allowMissing Allow for missing values. Enables/disables the exception
                        NotEnoughElements.  
                        - false -- exception is thrown if the input iterator 
                          fills up the last row only partially. 
                        - true -- exception is not thrown even if the input
                          iterator fills up the last row only partially. Instead
                          , missing values are set to SimTK::NaN.
    \param numRows Optional argument. Total number of rows being added. If
                   provided, the funciton applies optimization by allocating
                   memory for all the rows at once and populating them.

    \throws ZeroElements If the number of elements produced by the input 
                         iterator is zero.
    \throws InvalidEntry If the function is called on an empty DataTable_ 
                         -- without providing the argument numColumns or 
                         -- providing a zero for numColumns.
    \throws TooManyElements When \a numRows is specified, this exception is
                            thrown if the input iterator produces more elements
                            than needed to fill that many rows.
    \throws NotEnoughElements Arguments allowMissing enables/disables this
                              exception. When enables, this exception is thrown
                              if the input iterator does not produce enough 
                              elements to fill up the last row completely.    */
    template<typename InputIt>
    void addRows(InputIt first, 
                 InputIt last, 
                 size_t numColumns = 0,
                 bool allowMissing = false,
                 size_t numRows    = 0) {
        {
        using namespace internal;
        static_assert(is_dereferencable<InputIt>, "Input iterator (InputIt) is "
                      "not dereferencable. It does not support 'operator*()'.");

        static_assert(std::is_constructible<ET, decltype(*first)>::value, 
                      "The type of the value produced by dereferencing the "
                      "input iterator (InputIt) does not match template "
                      "parameter ET used to instantiate DataTable.");

        static_assert(is_eq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator==' and so is not comparable for " 
                      "equality.");

        static_assert(is_neq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator!=' and so is not comparable for " 
                      "inequality.");
        }
        
        throwIfIterHasZeroElements(first, last);

        if(data_.nrow() == 0 || data_.ncol() == 0) {
            if(numColumns == 0)
                throw InvalidEntry{"DataTable is empty. In order to add rows, "
                        "argument 'numColumns' must be provided and it cannot "
                        "be zero."};
        } else {
            if(numColumns != 0 &&
               static_cast<int>(numColumns) != data_.ncol())
                throw InvalidEntry{"DataTable has " + 
                        std::to_string(data_.nrow()) + " rows and " + 
                        std::to_string(data_.ncol()) + " columns. Argument " 
                        "'numColumns' must be either zero or equal to actual "
                        "number of columns. It is ignored either way but this"
                        " is just to prevent logical errors in the code."};
        }

        int oldNumRows{0};
        if(data_.nrow() == 0 || data_.ncol() == 0) {
            data_.resize(static_cast<int>(numRows ? numRows : 1), 
                         static_cast<int>(numColumns));
        } else {
            oldNumRows = data_.nrow();
            data_.resizeKeep(data_.nrow() + 
                             static_cast<int>(numRows ? numRows : 1), 
                             data_.ncol());
        }

        int row{oldNumRows};
        int col{0};
        while(first != last) {
            data_.set(row, col, *first);
            ++first; ++col;
            if(col == static_cast<int>(data_.ncol()) && first != last) {
                col = 0;
                ++row;
                if(numRows == 0)
                    data_.resizeKeep(data_.nrow() + 1, data_.ncol());
                else if(row == static_cast<int>(data_.nrow()))
                    throw TooManyElements{"Input iterator produced more "
                            "elements than needed to fill " + 
                            std::to_string(numRows) + " (numRows) rows."};
            }
        }

        if(!allowMissing) { 
            if(row != data_.nrow() - 1)
                throw NotEnoughElements{"Input iterator did not produce "
                        "enough elements to fill all the rows. Total rows added"
                        " = " + std::to_string(data_.nrow() - oldNumRows) + 
                        ", Filled rows = " + std::to_string(row - oldNumRows) +
                        "."};
            if(col != data_.ncol())
                throw NotEnoughElements{"Input iterator did not produce enough" 
                        " elements to fill the last row. Expected = " + 
                        std::to_string(data_.ncol()) + ", Received = " + 
                        std::to_string(col) + "."};
        }
    }

    /** Add (append) a rows to the DataTable_ using a container holding values
    of a type ET (template parameter). The container is required to support
    an iterator. In other words, the container must have member functions 
    %begin() and %end() that emit an iterator to the container. Calling this
    function is equivalent to calling addRows() taking an iterator:
    \code
    addRow(container.begin(),
           container.end(),
           numColumns,
           allowMissing,
           numRows);
    \endcode
    For details on arguments, see documentation for addRows() taking an 
    iterator.

    Besides being a convenience wrapper for addRows() taking an iterator, this
    function will _try_ to apply an optimization inquiring the number of 
    elements in the container by calling the member function %size() on the 
    container, if the container such a member function. Otherwise, this function
    is nothing more than a convenience wrapper. Knowing the total number of 
    elements allows this function to calculate the number of rows being added 
    and apply optimization by allocating memory for all the rows once and 
    populating them using the elements of the container.                       
                    
    If \a numRows is specified in the call, optimization is applied using
    that value but container size, if available, is still used to perform some 
    error checking.                                                           */
    template<typename Container>
    void addRows(const Container& container,
                 size_t numColumns = 0,
                 bool allowMissing = false,
                 size_t numRows    = 0) {
        {
        using namespace internal;
        static_assert(has_mem_begin<Container>, "Input container does not have "
                      "a member function named begin(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(has_mem_end<Container>, "Input container does not have "
                      "a member function named end(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(std::is_same<decltype(container.begin()),
                                   decltype(container.end())>::value,
                      "The member functions begin() and end() of input " 
                      "container do not produce the same type. Input container "
                      "is reuiqred to have members begin() and end() that " 
                      "return an iterator to the container.");
        }

        addRows_impl(container, numColumns, allowMissing, numRows);
    }

    /** Add (append) a column to the DataTable_ using a SimTK::Vector_. If the 
    DataTable is empty, input column will be the first column. This function can
    be used to populate an empty DataTable_.
  
    \param column The column to be added as a SimTK::Vector_.
  
    \throws ZeroElements If number of elements in the input column is zero.
    \throws NumberOfRowsMismatch If number of columns in the input column does 
                                 not match the number of rows of the 
                                 DataTable_.                                  */
    void addColumn(const SimTK::Vector_<ET>& column) {
        if(column.nrow() == 0 || column.ncol() == 0)
            throw ZeroElements{"Input column has zero length."};
        if(data_.nrow() > 0 && column.size() != data_.nrow())
            throw NotEnoughElements{"Input column has incorrect number of rows."
                                    "Expected = " + 
                                    std::to_string(data_.nrow()) + 
                                    " Received = " + 
                                    std::to_string(column.size())};

        data_.resizeKeep(column.size(), data_.ncol() + 1);
        data_.updCol(data_.ncol() - 1).updAsVector() = column;
    }

    /** Add (append) a column to the DataTable_ using an InputIterator producing
    one entry at a time. If this function is called on an empty DataTable_ 
    without providing numRowsHint, it performs <i>allocation + relocation</i> 
    for [log2(nrow) + 1] times where nrow is the actual number of elements 
    produced by the InputIterator. To add multiple columns at once using 
    input-iterator, use addColumns().

    \param first Beginning of range covered by the iterator.
    \param last End of the range covered by the iterator.
    \param numRowsHint Hint for the number of rows in the input iterator. Can 
                       be approximate above or below the actual number. This is 
                       only used when this function is called on an empty 
                       DataTable_. Ignored otherwise. Providing a hint reduces 
                       the number of resize operations which involves memory 
                       allocation + r elocation.
    \param allowMissing Allow for missing values. Enables/disables the exception
                        NotEnoughElements. This is only used when DataTable_ is
                        non-empty. 
                        - false -- exception is thrown if the input iterator 
                          fills up the row only partially. 
                        - true -- exception is not thrown even if the input
                          iterator fills up the row only partially. Instead, 
                          missing values are set to SimTK::NaN.

    \throws ZeroElements If the number of elements produced by the input
                         iterator is zero.                                      
    \throws InvalidEntry DataTable is empty and input argument numRowsHint is
                         zero.
    \throws TooManyElements When called on non-empty DataTable_, if the input
                            iterator produces more elements than needed to add
                            a column.
    \throws NotEnoughElements Argument allowMissing enables/disables this
                              exception. This exception is applicable only
                              when this function is called on a non-empty 
                              DataTable_. When enabled, this exception is thrown
                              if the InputIterator does not produce enough 
                              elements to fill up the column completely.      */
    template<typename InputIt>
    void addColumn(InputIt first, 
                   InputIt last, 
                   size_t numRowsHint = 2,
                   bool allowMissing = false) {
        {
        using namespace internal;
        static_assert(is_dereferencable<InputIt>, "Input iterator (InputIt) is "
                      "not dereferencable. It does not support 'operator*()'.");

        static_assert(std::is_constructible<ET, decltype(*first)>::value, 
                      "The type of the value produced by dereferencing the "
                      "input iterator (InputIt) does not match template "
                      "parameter ET used to instantiate DataTable.");

        static_assert(is_eq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator==' and so is not comparable for " 
                      "equality.");

        static_assert(is_neq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator!=' and so is not comparable for " 
                      "inequality.");
        }

        throwIfIterHasZeroElements(first, last);

        if((data_.nrow() == 0 || data_.ncol() == 0) && numRowsHint == 0)
            throw InvalidEntry{"Input argument 'numRowsHint' cannot be zero" 
                               " when DataTable is empty."};

        if(data_.nrow() > 0 && data_.ncol() > 0) {
            data_.resizeKeep(data_.nrow(), data_.ncol() + 1);
            int row{0};
            while(first != last) {
                if(row == data_.nrow())
                    throw TooManyElements{"Input iterator produced more "
                            "elements than needed to fill a column with " +
                            std::to_string(data_.nrow()) + " rows."};

                data_.set(row++, data_.ncol() - 1, *first);
                ++first;
            }
            if(!allowMissing && row != data_.nrow()) 
                throw NotEnoughElements{"Input iterator did not produce enough "
                                        "elements to fill the column. " 
                                        "Expected = " + 
                                        std::to_string(data_.nrow()) + 
                                        " Received = " + std::to_string(row)};
        } else {
            int row{0};
            size_t nrow{numRowsHint};
            data_.resizeKeep(static_cast<int>(nrow), 1);
            while(first != last) {
                data_.set(row++, 0, *first);
                ++first;
                if(row == static_cast<int>(nrow) && first != last) {
                    // If nrow is a power of 2, double it. Otherwise round it to
                    //  the next power of 2.
                    nrow = (nrow & (nrow - 1)) == 0 ? 
                           nrow << 2 : rndToNextPowOf2(nrow); 
                    data_.resizeKeep(static_cast<int>(nrow), 1);
                }
            }
            if(row != static_cast<int>(nrow))
                data_.resizeKeep(row, 1);
        }
    }

    /** Add (append) a column to the DataTable_ using a container holding values
    of a type ET (template parameter). The container is required to support
    an iterator. In other words, the container must have member functions 
    %begin() and %end() that emit an iterator to the container. Calling this
    function is equivalent to calling addColumn() taking an iterator:
    \code
    addColumn(container.begin(),
              container.end(),
              numRowsHint,
              allowMissing);
    \endcode
    For details, see documentation for addColumn() taking an iterator.

    Besides being a convenience wrapper for addColumn() taking an iterator, this
    function will _try_ to apply an optimization when it called to add a column 
    to an empty DataTable_. The function will _try_ to inquire the number of 
    elements in the container by calling the member function %size() on the 
    container, if the container has such a member function. Otherwise, this 
    function is nothing more than a convenience wrapper. Knowing the total 
    number of elements allows this function to know the length of the column 
    being added and apply optimization by allocating memory for the column once
    and populating it using the elements of the container.                      
    If \a numRowsHint is specified in the call, optimization is applied using
    that value but container size, if available, is still used to perform some 
    error checking.                                                           */
    template<typename Container>
    void addColumn(const Container& container, 
                   size_t numRowsHint = 2,
                   bool allowMissing = false) {
        {
        using namespace internal;
        static_assert(has_mem_begin<Container>, "Input container does not have "
                      "a member function named begin(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(has_mem_end<Container>, "Input container does not have "
                      "a member function named end(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(std::is_same<decltype(container.begin()),
                                   decltype(container.end())>::value,
                      "The member functions begin() and end() of input " 
                      "container do not produce the same type. Input container "
                      "is reuiqred to have members begin() and end() that " 
                      "return an iterator to the container.");
        }

        addColumn_impl(container, numRowsHint, allowMissing);
    }

    /** Add (append) multiple columns to the DataTable_ using an InputIterator 
    producing one entry at a time. If this function is called on an empty 
    DataTable_, numRows must be provided. Otherwise, numRows is ignored. To add 
    just one column, use addColumn() instead.

    \param first Beginning of range covered by the iterator.
    \param last End of the range covered by the iterator.
    \param numRows Number of rows to create in the DataTable_. This is only 
                   used (and required) when the function is called on an empty 
                   DataTable_. Ignored otherwise.
    \param allowMissing Allow for missing values. Enables/disables the exception
                        NotEnoughElements. 
                        - false -- exception is thrown if the input iterator
                          fills up the last column only partially.
                        - true -- exception is not thrown even if the input
                          iterator fills up the last column only partially.
                          Instead, missing values are set to SimTK::NaN.
    \param numColumns Optional argument. Total number of columns being added. If
                      provided, the function applies optimization by allocating
                      enough memory for all the columns at once and then
                      populating them.

    \throws ZeroElements If the number of elements produced by the input 
                         iterator is zero.
    \throws InvalidEntry If the function is called on an empty DataTable_ 
                         -- without providing the argument numRows or 
                         -- providing zero for numRows.
    \throws TooManyElements When \a numColumns is specified, this exception is
                            thrown if the input iterator produces more elements
                            than needed to fill that many columns.
    \throws NotEnoughElements Argument allowMissing enables/disables this 
                              exception. When enabled, this exception is thrown
                              if the input iterator does not produce enough 
                              elements to fill up the last column completely. */
    template<typename InputIt>
    void addColumns(InputIt first, 
                    InputIt last, 
                    size_t numRows    = 0,
                    bool allowMissing = false,
                    size_t numColumns = 0) {
        {
        using namespace internal;
        static_assert(is_dereferencable<InputIt>, "Input iterator (InputIt) is "
                      "not dereferencable. It does not support 'operator*()'.");

        static_assert(std::is_constructible<ET, decltype(*first)>::value, 
                      "The type of the value produced by dereferencing the "
                      "input iterator (InputIt) does not match template "
                      "parameter ET used to instantiate DataTable.");

        static_assert(is_eq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator==' and so is not comparable for " 
                      "equality.");

        static_assert(is_neq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator!=' and so is not comparable for " 
                      "inequality.");
        }
        
        throwIfIterHasZeroElements(first, last);

        if(data_.nrow() == 0 || data_.ncol() == 0) {
           if(numRows == 0)
               throw InvalidEntry{"DataTable is empty. In order to add columns,"
                       " argument 'numRows' must be provided and it cannot be "
                       "zero."};
        } else {
            if(numRows != 0 &&
               static_cast<int>(numRows) != data_.nrow())
                throw InvalidEntry{"DataTable has " + 
                        std::to_string(data_.nrow()) + " rows and " +
                        std::to_string(data_.ncol()) + " columns. Argument "
                        "'numRows' must be either zero or equal to actual " 
                        "number of rows. It is ignored either way but this is"
                        " just to prevent logical errors in the code."};
        }

        int oldNumColumns{0};
        if(data_.nrow() == 0 || data_.ncol() == 0) {
            data_.resize(static_cast<int>(numRows), 
                         static_cast<int>(numColumns ? numColumns : 1));
        } else {
            oldNumColumns = data_.ncol();
            data_.resizeKeep(data_.nrow(), 
                             data_.ncol() + 
                             static_cast<int>(numColumns ? numColumns : 1));
        }

        int row{0};
        int col{oldNumColumns};
        while(first != last) {
            data_.set(row, col, *first);
            ++first; ++row;
            if(row == static_cast<int>(data_.nrow()) && first != last) {
                row = 0;
                ++col;
                if(numColumns == 0)
                    data_.resizeKeep(data_.nrow(), data_.ncol() + 1);
                else if(col == static_cast<int>(data_.ncol()))
                    throw TooManyElements{"Input iterator produced more "
                            "elements than needed to fill " +
                            std::to_string(numColumns) + " (numColumns) "
                            "columns"};
            }
        }
        if(!allowMissing) {
            if(col != data_.ncol() - 1)
                throw NotEnoughElements{"Input iterator did not produce "
                        "enough elements to fill all the columns. Total columns"
                        " added = " + 
                        std::to_string(data_.ncol() - oldNumColumns) + 
                        ", Filled columns = " + 
                        std::to_string(col - oldNumColumns) + "."};
            if(row != data_.nrow())
                throw NotEnoughElements{"Input iterator did not produce enough" 
                        " elements to fill the last column. Expected = " + 
                        std::to_string(data_.nrow()) + ", Received = " + 
                        std::to_string(row) + "."};
        }
    }

    /** Add (append) a columns to the DataTable_ using a container holding 
    values of a type ET (template parameter). The container is required to 
    support an iterator. In other words, the container must have member 
    functions %begin() and %end() that emit an iterator to the container. 
    Calling this function is equivalent to calling addRows() taking an iterator:
    \code
    addRow(container.begin(),
           container.end(),
           numRows,
           allowMissing,
           numColumns);
    \endcode
    For details on arguments, see documentation for addColumns() taking an 
    iterator.

    Besides being a convenience wrapper for addColumns() taking an iterator, 
    this function will _try_ to apply an optimization inquiring the number of 
    elements in the container by calling the member function %size() on the 
    container, if the container such a member function. Otherwise, this function
    is nothing more than a convenience wrapper. Knowing the total number of 
    elements allows this function to calculate the number of columns being added
    and apply optimization by allocating memory for all the columns once and 
    populating them using the elements of the container.                       
                    
    If \a numColumns is specified in the call, optimization is applied using
    that value but container size, if available, is still used to perform some 
    error checking.                                                           */
    template<typename Container>
    void addColumns(const Container& container,
                    size_t numRows    = 0,
                    bool allowMissing = false,
                    size_t numColumns = 0) {
        {
        using namespace internal;
        static_assert(has_mem_begin<Container>, "Input container does not have "
                      "a member function named begin(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(has_mem_end<Container>, "Input container does not have "
                      "a member function named end(). Input container is " 
                      "required to have members begin() and end() that return " 
                      "an iterator to the container.");

        static_assert(std::is_same<decltype(container.begin()),
                                   decltype(container.end())>::value,
                      "The member functions begin() and end() of input " 
                      "container do not produce the same type. Input container "
                      "is reuiqred to have members begin() and end() that " 
                      "return an iterator to the container.");
        }

        addColumns_impl(container, numRows, allowMissing, numColumns);
    }

    /** Get all the rows of the DataTable_. Returns an object that can be used
    in a range-for statement. The returned object supports begin() and end()
    functions to retrieve begin and end iterators respectively. Dereferencing
    the iterator will produce a SimTK::RowVectorView_. The result is not
    writable.                                                                 */
    RowsOrColsContainerProxy<true, true> getRows() const {
        throwIfDataTableEmpty();

        return this;
    }

    /** Get all the rows of the DataTable_. Returns an object that can be used
    in a range-for statement. The returned object supports begin() and end()
    functions to retrieve begin and end iterators respectively. Dereferencing
    the iterator will produce a SimTK::RowVectorView_. The result is writable.*/
    RowsOrColsContainerProxy<true, false> updRows() {
        throwIfDataTableEmpty();

        return this;
    }

    /** Get all the rows of the DataTable_. Returns an object that can be used
    in a range-for statement. The returned object supports begin() and end()
    functions to retrieve begin and end iterators respectively. Dereferencing
    the iterator will produce a SimTK::RowVectorView_. The result is not
    writable.                                                                 */
    RowsOrColsContainerProxy<false, true> getColumns() const {
        throwIfDataTableEmpty();

        return this;
    }

    /** Get all the rows of the DataTable_. Returns an object that can be used
    in a range-for statement. The returned object supports begin() and end()
    functions to retrieve begin and end iterators respectively. Dereferencing
    the iterator will produce a SimTK::RowVectorView_. The result is writable.*/
    RowsOrColsContainerProxy<false, false> updColumns() {
        throwIfDataTableEmpty();

        return this;
    }

    /** Get a const iterator (representing the beginning) over rows.          */
    Iterator<true, true> rowsCBegin() const {
        throwIfDataTableEmpty();

        return {this, 0};
    }

    /** Get a const iterator (representing the end) over rows.                */
    Iterator<true, true> rowsCEnd() const {
        throwIfDataTableEmpty();

        return {this, data_.nrow()};
    }

    /** Get a const iterator (representing the beginning) over rows.          */
    Iterator<true, true> rowsBegin() const {
        throwIfDataTableEmpty();

        return rowsCBegin();
    }

    /** Get a const iterator (representing the end) over rows.                */
    Iterator<true, true> rowsEnd() const {
        throwIfDataTableEmpty();

        return rowsCEnd();
    }

    /** Get a non-const iterator (representing the beginning) over rows.      */
    Iterator<true, false> rowsBegin() {
        throwIfDataTableEmpty();

        return {this, 0};
    }

    /** Get a non-const iterator (representing the end) over rows.            */
    Iterator<true, false> rowsEnd() {
        throwIfDataTableEmpty();

        return {this, data_.nrow()};
    }

    /** Get a const iterator (representing the beginning) over columns.       */
    Iterator<false, true> columnsCBegin() const {
        throwIfDataTableEmpty();

        return {this, 0};
    }

    /** Get a const iterator (representing the end) over columns.             */
    Iterator<false, true> columnsCEnd() const {
        throwIfDataTableEmpty();

        return {this, data_.ncol()};
    }

    /** Get a const iterator (representing the beginning) over columns.       */
    Iterator<false, true> columnsBegin() const {
        throwIfDataTableEmpty();

        return columnsCBegin();
    }

    /** Get a const iterator (representing the end) over columns.             */
    Iterator<false, true> columnsEnd() const {
        throwIfDataTableEmpty();

        return columnsCEnd();
    }

    /** Get a non-const iterator (representing the beginning) over columns.   */
    Iterator<false, false> columnsBegin() {
        throwIfDataTableEmpty();

        return {this, 0};
    }

    /** Get a non-const iterator (representing the end) over columns.         */
    Iterator<false, false> columnsEnd() {
        throwIfDataTableEmpty();

        return {this, data_.ncol()};
    }

    /** Add/concatenate rows of another DataTable_ to this DataTable_.
    Concatenation can be done in two ways:
    - If \a matchColumnLabels is true, Columns from the input DataTable_ are 
      appended to the columns of this DataTable_ by matching column labels. For
      example, a column labeled "column-three" in this DataTable_ will be 
      appended with column labeled the same in the input DataTable_ regardless 
      of the column's index in the input DataTable_. This can be used if the 
      columns in the two DataTables are not in the same order. Both 
      DataTable_(s) must have all their columns labeled.
    - If \a matchColumnLabels is false, the underlying matrix from the input
      DataTable_ will be appended to underlying matrix of this DataTable_. This
      DataTable_ will retain its column labels. This can be used if it is known
      that the columns of the input DataTable_ are in the same order as columns
      of this DataTable_. This alternative is more efficient than the previous.
      The two DataTable_(s) need not have all their columns labeled.

    Metadata is not added from the other DataTable_. To create a new DataTable_
    by concatenating two existing DataTable_(s), use OpenSim::concatenateRows().

    \throws EmptyDataTable If either this table or the input table is empty.
    \throws NumberOfColumsMismatch If input DataTable_ has different number of
                                   columns for concatenation to work.
    \throws MissingColumnLabels If this DataTable_ or the input DataTable_ is 
                                missing label for any column. Applicable only
                                when \a matchColumnLabels is true.
    \throws ColumnLabelsMismatch If any column label from this DataTable_ is not
                                 found in the input DataTable_. Applicable only
                                 when \a matchColumnLabels is true.
    \throws InvalidEntry If trying to add a DataTable_ to itself.             */
    virtual void concatenateRows(const DataTable_& table,
                                 bool matchColumnLabels = true) {
        if(&data_ == &table.data_)
            throw InvalidEntry{"Cannot concatenate a DataTable to itself."};

        throwIfDataTableEmpty();
        table.throwIfDataTableEmpty();

        if(data_.ncol() != table.data_.ncol()) 
            throw NumberOfColumnsMismatch{"DataTables have different number"
                                          " of columns. Table 1 = " + 
                                          std::to_string(data_.ncol()) + 
                                          " Table 2 = " + 
                                          std::to_string(table.data_.ncol())};

        if(matchColumnLabels) {
            throwIfMissingColumnLabels();
            table.throwIfMissingColumnLabels();
        }
        
        int row_start{data_.nrow()};
        data_.resizeKeep(data_.nrow() + table.data_.nrow(), data_.ncol());

        if(matchColumnLabels) {
            for(const auto& label_ind : col_ind_) {
                try {
                    auto column = data_.updBlock(row_start, label_ind.second, 
                                                 table.data_.nrow(), 1);
                    column = table.getColumn(label_ind.first);
                } catch (ColumnDoesNotExist&) {
                    data_.resizeKeep(row_start, data_.ncol());
                    throw ColumnLabelsMismatch{"Column label " + 
                            label_ind.first + " exists in DataTable 1 but not "
                            "in DataTable 2. For concatenation, all labels in "
                            "DataTable 1 must exist in DataTable 2 and vice " 
                            "versa."};
                }
            }
        } else {
            data_.updBlock(row_start, 0,
                           table.data_.nrow(), data_.ncol()) = table.data_;
        }
    }

    /** Add/concatenate columns of another DataTable_ to this DataTable_. 
    Concatenation can be done in two ways:
    - If \a discardColumnLabels is false, column labels from the input 
      DataTable_ are added to this DataTable_. For this to happen, both 
      DataTables must have all their columns labeled and together, all the
      column labels across both tables must be unique.
    - If \a discardColumnLabels is true, column labels of the input DataTable_
      are discarded during concatenation. After concatenation, these columns
      will have no labels.

    Metadata are not added. To create a new DataTable_ by concatenating two 
    existing DataTable_(s), use OpenSim::concatenateColumns().

    \throws EmptyDataTable If either this table or the input table is empty.
    \throws NumberOfRowsMismatch If input DataTable_ has incorrect number of
                                 rows for concatenation to work.
    \throws MissingColumnLabels If this DataTable_ or the input DataTable_ is 
                                missing label for any column. Applicable only
                                when \a dicardColumnLabels is false.
    \throws DuplicateColumnLabels If any column label in this DataTable_ also
                                  exists in the input DataTable_. Applicable 
                                  only when \a discardColumnLabels is false.
    \throws InvalidEntry If trying to concatenation a DataTable_ to itself.   */
    virtual void concatenateColumns(const DataTable_& table, 
                                    bool discardColumnLabels = false) {
        if(&data_ == &table.data_)
            throw InvalidEntry{"Cannot concatenate a DataTable to itself."};

        throwIfDataTableEmpty();
        table.throwIfDataTableEmpty();

        if(data_.nrow() != table.data_.nrow())
            throw NumberOfRowsMismatch{"Input DataTable has different number of"
                                       " rows. Table 1 = " + 
                                       std::to_string(data_.nrow()) + 
                                       " Table 2 = " 
                                       + std::to_string(table.data_.nrow())};

        if(!discardColumnLabels) {
            throwIfMissingColumnLabels();
            table.throwIfMissingColumnLabels();
            for(const auto& label_ind : col_ind_)
                if(table.col_ind_.find(label_ind.first) != table.col_ind_.end())
                    throw DuplicateColumnLabels{"Both DataTables have column "
                            "label '" + label_ind.first + "'. Column labels "
                            "have to be unique across both tables for "
                            "concatenation to work."};

            col_ind_.insert(table.col_ind_.cbegin(), table.col_ind_.cend());
        }

        int col_start{data_.ncol()};
        data_.resizeKeep(data_.nrow(), data_.ncol() + table.data_.ncol());
        data_.updBlock(0, col_start, 
                       data_.nrow(), table.data_.ncol()) = table.data_;
    }

    /** Clear the data of this DataTable_. After this operation, the DataTable_
    will be of size 0x0 and all column labels will be cleared as well.        */
    void clearData() {
        data_.clear();
        clearColumnLabels();
    }

    /** Resize the DataTable_, retaining as much of the old data as will fit. 
    New memory will be allocated and the existing entries will be copied over 
    to the extent they will fit. If columns are dropped during this call, the 
    labels for the lost columns will also be lost. *Be careful not to shrink 
    the DataTable unintentionally*.                                           */
    void resizeKeep(size_t numRows, size_t numColumns) {
        if(numRows == 0)
            throw InvalidEntry{"Input argument 'numRows' cannot be zero."
                               "To clear all data, use clearData()."};
        if(numColumns == 0)
            throw InvalidEntry{"Input argument 'numColumns' cannot be zero."
                               "To clear all data, use clearData()."};

        if(static_cast<int>(numColumns) < data_.ncol())
            for(size_t c_ind = numColumns; 
                c_ind < static_cast<size_t>(data_.ncol()); 
                ++c_ind) 
                removeColumnLabel(c_ind);

        data_.resizeKeep(static_cast<int>(numRows), 
                         static_cast<int>(numColumns));
    }

    /** Check if a row exists by its index.                                   */
    bool hasRow(size_t rowIndex) const {
        return (rowIndex >= 0 &&
                rowIndex < static_cast<size_t>(data_.nrow()));
    }

    /** Check if a column exists by its index.                                */
    bool hasColumn(size_t columnIndex) const override {
        return (columnIndex >= 0 && 
                columnIndex < static_cast<size_t>(data_.ncol()));
    }

    using AbstractDataTable::hasColumn;

protected:
    /** \cond */
    // Helper functions. Function overloads detect presence of size() member
    // function in the container and use it to perform optimization and error
    // checking.
    template<typename Container, 
             typename = decltype(std::declval<Container>().size())>
    void DataTable__impl(const Container& container,
                         size_t numEntriesInMajor,
                         TraverseDir traverseDir,
                         bool allowMissing,
                         size_t numMajors) {
        if(numMajors == 0) {
            auto res = std::div(static_cast<int>(container.size()), 
                                static_cast<int>(numEntriesInMajor));

            if(!allowMissing && res.rem != 0) {
                if(traverseDir == TraverseDir::RowMajor)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements to add full rows. Last "
                            "row received " + std::to_string(res.rem) + 
                            " elements. Expected " + 
                            std::to_string(numEntriesInMajor) + " elements " 
                            "(numEntriesInMajor). Missing values are not " 
                            "allowed (allowMissing)."};
                else if(traverseDir == TraverseDir::ColumnMajor)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements to add full columns. Last"
                            " column received " + std::to_string(res.rem) +
                            " elements. Expected " + 
                            std::to_string(numEntriesInMajor) + " elements " 
                            "(numEntriesInMajor). Missing values are not " 
                            "allowed (allowMissing)."};
            }

            numMajors = static_cast<size_t>(res.rem ? 
                                            res.quot + 1 : res.quot);
        } else {
            if(numMajors * numEntriesInMajor < container.size()) {
                if(traverseDir == TraverseDir::RowMajor)
                    throw TooManyElements{"The container has more elements than"
                            " needed to add " + std::to_string(numMajors) + 
                            " rows (numMajors) with "
                            + std::to_string(numEntriesInMajor) + " columns "
                            "(numEntriesInMajor). Expected = " + 
                            std::to_string(numMajors * numEntriesInMajor) + 
                            " elements,  Received = " + 
                            std::to_string(container.size()) + " elements."};
                else if(traverseDir == TraverseDir::ColumnMajor)
                    throw TooManyElements{"The container has more elements than"
                            " needed to add " + std::to_string(numMajors) + 
                            " columns (numMajors) with " + 
                            std::to_string(numEntriesInMajor) + 
                            " rows (numEntriesInMajor). Expected = " + 
                            std::to_string(numMajors * numEntriesInMajor) + 
                            " elements,  Received = " + 
                            std::to_string(container.size()) + " elements."};
            } else if(numMajors * numEntriesInMajor > container.size() &&
                      !allowMissing) {
                if(traverseDir == TraverseDir::RowMajor)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements to add " + std::to_string(numMajors) + 
                            " rows (numMajors) with " +
                            std::to_string(numEntriesInMajor) + " columns " 
                            "(numEntriesInMajor). Expected = " + 
                            std::to_string(numMajors * numEntriesInMajor) +
                            " elements. Received = " + 
                            std::to_string(container.size()) + " elements."};
                if(traverseDir == TraverseDir::ColumnMajor)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements to add " + std::to_string(numMajors) + 
                            " columns (numMajors) with " + 
                            std::to_string(numEntriesInMajor) + " rows " 
                            "(numEntriesInMajor). Expected = " + 
                            std::to_string(numMajors * numEntriesInMajor) +
                            " elements. Received = " + 
                            std::to_string(container.size()) + " elements."};
                
            }
        }

        DataTable_(container.begin(),
                   container.end(),
                   numEntriesInMajor,
                   traverseDir,
                   allowMissing,
                   numMajors);
    }
    template<typename Container>
    void DataTable__impl(const Container& container,
                         size_t numEntriesInMajor,
                         TraverseDir traverseDir,
                         unsigned allowMissing,
                         size_t numMajors) {
        DataTable_(container.begin(),
                   container.end(),
                   numEntriesInMajor,
                   traverseDir,
                   allowMissing,
                   numMajors);
    }

    // Helper funcitons. Function overloads detect the presence of size() member
    // function in the container and use it to apply optimization.
    template<typename Container>
    void addRow_impl(const Container& container, 
                     decltype(std::declval<Container>().size()),
                     bool allowMissing) {
        addRow(container.begin(), 
               container.end(), 
               container.size(), 
               allowMissing);
    }
    template<typename Container>
    void addRow_impl(const Container& container,
                     size_t numColumnsHint,
                     unsigned allowMissing) {
        addRow(container.begin(),
               container.end(),
               numColumnsHint,
               allowMissing);
    }

    // Helper functions. Function overloads detect the presence of size() member
    // function in the container and use it to apply optimization and perform 
    // error checking.
    template<typename Container,
             typename = decltype(std::declval<Container>().size())>
    void addRows_impl(const Container& container,
                      size_t numColumns,
                      bool allowMissing,
                      size_t numRows) {
        if(data_.nrow() == 0 || data_.ncol() == 0) {
            if(numColumns == 0)
                throw InvalidEntry{"DataTable is empty. Argument 'numColumns' "
                        "must be provided and it cannot be zero."};
            if(numRows == 0) {
                auto res = std::div(static_cast<int>(container.size()), 
                                    static_cast<int>(numColumns));

                if(!allowMissing && res.rem != 0)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements add full rows. Last "
                            "row received " + std::to_string(res.rem) + 
                            " elements. Expected " + 
                            std::to_string(numColumns) + " elements " 
                            "(numColumns). Missing values are not " 
                            "allowed (allowMissing)."};
                
                numRows = static_cast<size_t>(res.rem ? 
                                              res.quot + 1 : res.quot);
            } else {
                if(numRows * numColumns < container.size())
                    throw TooManyElements{"The container has more elements than"
                            " needed to add " + std::to_string(numRows) + 
                            " rows (numRows) with "
                            + std::to_string(numColumns) + " columns "
                            "(numColumns). Expected = " + 
                            std::to_string(numRows * numColumns) + 
                            " elements,  Received = " + 
                            std::to_string(container.size()) + " elements."};
                if(numRows * numColumns > container.size() && !allowMissing)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements to add " + std::to_string(numRows) + 
                            " rows (numRows) with " + std::to_string(numColumns)
                            + " columns (numColumns). Expected = " + 
                            std::to_string(numRows * numColumns) +
                            " elements. Received = " + 
                            std::to_string(container.size()) + " elements."};
            }
        } else {
            if(numRows == 0) {
                auto res = std::div(static_cast<int>(container.size()), 
                                    static_cast<int>(data_.ncol()));
                
                if(!allowMissing && res.rem != 0)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements to add full rows. Last "
                            "row received " + std::to_string(res.rem) + 
                            " elements. Expected " + 
                            std::to_string(data_.ncol()) + " elements " 
                            "(getNumColumns()). Missing values are not " 
                            "allowed (allowMissing)."};

                numRows = static_cast<size_t>(res.rem ? 
                                              res.quot + 1 : res.quot);
            } else {
                if(numRows * static_cast<size_t>(data_.ncol()) < 
                   container.size())
                    throw TooManyElements{"The container has more elements than"
                            " needed to add " + std::to_string(numRows) + 
                            " rows (numRows) with " + 
                            std::to_string(data_.ncol()) + " columns "
                            "(getNumColumns()). Expected = " + 
                            std::to_string(numRows * 
                                           static_cast<size_t>(data_.ncol())) + 
                            " elements,  Received = " + 
                            std::to_string(container.size()) + " elements."};
                if(numRows * static_cast<size_t>(data_.ncol()) > 
                   container.size() && !allowMissing)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements to add " + std::to_string(numRows) + 
                            " rows (numRows) with " + 
                            std::to_string(data_.ncol()) +
                            " columns (numColumns). Expected = " + 
                            std::to_string(numRows * 
                                           static_cast<size_t>(data_.ncol())) +
                            " elements. Received = " + 
                            std::to_string(container.size()) + " elements."};
            }
        }

        addRows(container.begin(),
                container.end(),
                numColumns,
                allowMissing,
                numRows);
    }
    template<typename Container>
    void addRows_impl(const Container& container,
                      size_t numColumns,
                      unsigned allowMissing,
                      size_t numRows) {
        addRows(container.begin(),
                container.end(),
                numColumns,
                allowMissing,
                numRows);
    }

    // Helper functions. Function overloads detect the presence of size() member
    // function in the container and use it to apply optimization.
    template<typename Container>
    void addColumn_impl(const Container& container, 
                        decltype(std::declval<Container>().size()),
                        bool allowMissing) {
        addColumn(container.begin(), 
                  container.end(), 
                  container.size(), 
                  allowMissing);
    }
    template<typename Container>
    void addColumn_impl(const Container& container,
                        size_t numColumnsHint,
                        unsigned allowMissing) {
        addColumn(container.begin(),
                  container.end(),
                  numColumnsHint,
                  allowMissing);
    }

    // Helper functions. Function overlaods detect the presence of size() member
    // function in the container and use it to apply optimization and perform
    // error checking.
    template<typename Container,
             typename = decltype(std::declval<Container>().size())>
    void addColumns_impl(const Container& container,
                         size_t numRows,
                         bool allowMissing,
                         size_t numColumns) {
        if(data_.nrow() == 0 || data_.ncol() == 0) {
            if(numRows == 0)
                throw InvalidEntry{"DataTable is empty. Argument 'numRows' "
                        "must be provided and it cannot be zero."};
            if(numColumns == 0) {
                auto res = std::div(static_cast<int>(container.size()), 
                                    static_cast<int>(numRows));

                if(!allowMissing && res.rem != 0)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements add full columns. Last "
                            "column received " + std::to_string(res.rem) + 
                            " elements. Expected " + 
                            std::to_string(numRows) + " elements " 
                            "(numRows). Missing values are not " 
                            "allowed (allowMissing)."};
                
                numColumns = static_cast<size_t>(res.rem ? 
                                                 res.quot + 1 : res.quot);
            } else {
                if(numRows * numColumns < container.size())
                    throw TooManyElements{"The container has more elements than"
                            " needed to add " + std::to_string(numColumns) + 
                            " columns (numColumns) with "
                            + std::to_string(numRows) + " rows "
                            "(numRows). Expected = " + 
                            std::to_string(numRows * numColumns) + 
                            " elements,  Received = " + 
                            std::to_string(container.size()) + " elements."};
                if(numRows * numColumns > container.size() && !allowMissing)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements to add " + std::to_string(numColumns) + 
                            " columns (numColumns) with " + 
                            std::to_string(numRows) + " rows (numRows). "
                            "Expected = " + std::to_string(numRows * numColumns)
                            + " elements, Received = " + 
                            std::to_string(container.size()) + " elements."};
            }
        } else {
            if(numColumns == 0) {
                auto res = std::div(static_cast<int>(container.size()), 
                                    static_cast<int>(data_.nrow()));
                
                if(!allowMissing && res.rem != 0)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements to add full columns. Last "
                            "column received " + std::to_string(res.rem) + 
                            " elements. Expected " + 
                            std::to_string(data_.nrow()) + " elements " 
                            "(getNumRows()). Missing values are not " 
                            "allowed (allowMissing)."};

                numColumns = static_cast<size_t>(res.rem ? 
                                                 res.quot + 1 : res.quot);
            } else {
                if(static_cast<size_t>(data_.nrow()) * numColumns < 
                   container.size())
                    throw TooManyElements{"The container has more elements than"
                            " needed to add " + std::to_string(numColumns) + 
                            " columns (numColumns) with " + 
                            std::to_string(data_.nrow()) + " rows "
                            "(getNumRows()). Expected = " + 
                            std::to_string(static_cast<size_t>(data_.nrow()) * 
                                           numColumns) + 
                            " elements,  Received = " + 
                            std::to_string(container.size()) + " elements."};
                if(static_cast<size_t>(data_.nrow()) * numColumns > 
                   container.size() && !allowMissing)
                    throw NotEnoughElements{"The container does not have enough"
                            " elements to add " + std::to_string(numColumns) + 
                            " columns (numColumns) with " + 
                            std::to_string(data_.nrow()) +
                            " rows (numRows). Expected = " + 
                            std::to_string(static_cast<size_t>(data_.nrow()) * 
                                           numColumns) +
                            " elements. Received = " + 
                            std::to_string(container.size()) + " elements."};
            }
        }

        addColumns(container.begin(),
                   container.end(),
                   numRows,
                   allowMissing,
                   numColumns);
    }
    template<typename Container>
    void addColumns_impl(const Container& container,
                         size_t numRows,
                         unsigned allowMissing,
                         size_t numColumns) {
        addColumns(container.begin(),
                   container.end(),
                   numRows,
                   allowMissing,
                   numColumns);
    }

    // Helper function. Throw if the DataTable_ is empty.
    void throwIfDataTableEmpty() const {
        if(data_.nrow() == 0 || data_.ncol() == 0)
            throw EmptyDataTable{"DataTable is empty."};
    }

    // Helper function. Check if a row exists and throw an exception if it does
    // not.
    void throwIfRowDoesNotExist(const size_t rowIndex) const {
        if(!hasRow(rowIndex))
            throw RowDoesNotExist{"Row " + std::to_string(rowIndex) + 
                                  " does not exist. Index out of range."};
    }

    // Helper function. Throw if any column is missing label.
    void throwIfMissingColumnLabels() const {
        if(getNumColumnLabels() != getNumColumns())
            throw MissingColumnLabels{"DataTable has columns with no "
                    "labels. All columns need to have labels for "
                    "concatenation."};
    }

    // Helper function. Round to next highest power of 2. Works only for 
    // 32 bits.
    size_t rndToNextPowOf2(size_t num) const {
        assert(static_cast<unsigned long long>(num) <= 
               static_cast<unsigned long long>(0xFFFFFFFF));

        --num;
        num |= (num >>  1); // Highest  2 bits are set by end of this.
        num |= (num >>  2); // Highest  4 bits are set by end of this.
        num |= (num >>  4); // Highest  8 bits are set by end of this.
        num |= (num >>  8); // Highest 16 bits are set by end of this.
        num |= (num >> 16); // Highest 32 bits are set by end of this.
        return ++num;
    }

    // Matrix of data. This excludes timestamp column.
    SimTK::Matrix_<ET> data_;
    /** \endcond */
};  // DataTable_

/** See DataTable_ for details on the interface.                              */
using DataTable = DataTable_<SimTK::Real>;


/** Concatenate two DataTable_(s) by row and produce a new DataTable_.        
See DataTable_::concatenateRows() for details.                                */
template<typename ET>
DataTable_<ET> concatenateRows(const DataTable_<ET>& dt1, 
                               const DataTable_<ET>& dt2) {
    DataTable_<ET> dt{dt1};
    dt.concatenateRows(dt2);
    return dt;
}


/** Concatenate two DataTable_(s) by column and produce a new DataTable_.     
See DataTable_::concatenateColumns() for details.                             */
template<typename ET>
DataTable_<ET> concatenateColumns(const DataTable_<ET>& dt1, 
                                  const DataTable_<ET>& dt2) {
    DataTable_<ET> dt{dt1};
    dt.concatenateColumns(dt2);
    return dt;
}

} // namespace OpenSim

#endif //OPENSIM_COMMON_DATATABLE_H_
