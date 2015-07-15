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
This file defines the TimeSeriesDataTable class, which is used by OpenSim to 
provide an in-memory container for data access and manipulation.              */

#ifndef OPENSIM_COMMON_TIMESERIESDATATABLE_H_
#define OPENSIM_COMMON_TIMESERIESDATATABLE_H_

#include "OpenSim/Common/DataTable.h"


namespace OpenSim {

/** \cond */
class TimestampsEmpty : public OpenSim::Exception {
public:
    TimestampsEmpty(const std::string& expl) : Exception(expl) {}
};


class DataHasZeroRows : public OpenSim::Exception {
public:
    DataHasZeroRows(const std::string& expl) : Exception(expl) {}
};


class TimestampsLengthIncorrect : public OpenSim::Exception {
public:
    TimestampsLengthIncorrect(const std::string& expl) : Exception(expl) {}
};


class TimestampDoesNotExist : public OpenSim::Exception {
public:
    TimestampDoesNotExist(const std::string& expl) : Exception(expl) {}
};


class TimestampBreaksInvariant : public OpenSim::Exception {
public:
    TimestampBreaksInvariant(const std::string& expl) : Exception(expl) {}
};

class TimestampsColumnFull : public OpenSim::Exception {
public:
    TimestampsColumnFull(const std::string& expl) : Exception(expl) {}
};
/** \endcond */


enum class NearestDir {
    LessOrGreaterThanEqual,
    LessThanEqual,
    GreaterThanEqual
};


/** TimeSeriesDataTable_ is a DataTable_ that adds support for a time-series 
column. 

The timestamp column is enforced to be strictly increasing. Entries in the 
time-series column can be used to access the rows of the DataTable.           */
template<typename ET = SimTK::Real, typename TS = float>
class TimeSeriesDataTable_ : public DataTable_<ET> {
    static_assert(std::is_arithmetic<TS>::value, "Template argument 'TS' "
                  "representing timestamp must be an arithmetic type (eg. int, "
                  "float, double etc.).");

protected:
    /** \cond */
    using string = std::string;
    using Timestamps = std::vector<TS>;
    using TimestampsIter = typename Timestamps::iterator;
    using TimestampsConstIter = typename Timestamps::const_iterator;

    class TimestampsContainerProxy {
    public:
        TimestampsContainerProxy(const TimeSeriesDataTable_* tsdt) : 
            tsdt_{tsdt} {}
        TimestampsContainerProxy()                                    = delete;
        TimestampsContainerProxy(const TimestampsContainerProxy&)     = default;
        TimestampsContainerProxy(TimestampsContainerProxy&&)          = default;
        TimestampsContainerProxy& operator=(const TimestampsContainerProxy&)
                                                                      = default;
        TimestampsContainerProxy& operator=(TimestampsContainerProxy&&) 
                                                                      = default;

        TimestampsConstIter cbegin() const {
            return tsdt_->timestampsBegin();
        }

        TimestampsConstIter cend() const {
            return tsdt_->timestampsEnd();
        }

        TimestampsConstIter begin() const {
            return cbegin();
        }

        TimestampsConstIter end() const {
            return cend();
        }

    private:
        const TimeSeriesDataTable_* tsdt_;
    };
    /** \endcond */

public:
    using timestamp_type = TS;

    /** Inherit constructors.                                                 */
    using DataTable_<ET>::DataTable_;

    /** Copy.                                                                 */
    TimeSeriesDataTable_& operator=(const TimeSeriesDataTable_&) = default;
    TimeSeriesDataTable_& operator=(TimeSeriesDataTable_&&)      = default;

    /** Destroy.                                                              */
    ~TimeSeriesDataTable_() override = default;

    /** Clear the timestamp column.                                           */
    void clearTimestamps() {
        timestamps_.clear();
    }

    /** Check if the timestamp column is empty.                               */
    bool timestampsEmpty() const {
        return timestamps_.empty();
    }

    /** Check if the DataTable has a timestamp.                               

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimestampsLengthIncorrect If the length of the timestamp column does
                                      not match the number of rows in the
                                      DataTable.                              */
    template<typename Timestamp>
    bool hasTimestamp(Timestamp timestamp) const {
        throwIfDataHasZeroRows();
        throwIfTimestampsLengthIncorrect();

        return std::binary_search(timestamps_.cbegin(), 
                                  timestamps_.cend(), 
                                  timestamp);
    }

    /** Add (append) a timestamp to the timestamp column.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimestampsColumnFull If the length of the timestamp column is 
                                 already equal to the number of rows in the
                                 DataTable. At this point no timestamps can be
                                 added.
    \throws TimestampBreaksInvariant If the new timestamp breaks the invariant
                                     that the timestamp column must be
                                     increasing.                              */
    void addTimestamp(TS timestamp) {
        throwIfDataHasZeroRows();
        throwIfTimestampsFull();
        throwIfTimestampBreaksInvariantPrev(timestamps_.size(), timestamp);

        timestamps_.push_back(timestamp);
    }

    /** Add (append) timestamps to the timestamp column using an iterator. This
    is equivalent to calling addTimestamp() with a single timestamp multiple
    times.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimestampsColumnFull If the iterator produces more values than
                                 needed to fill up the timestamp column. The
                                 timestamp column is not allowed to have
                                 length past the number of rows.
    \throws TimestampBreaksInvariant If the a new timestamp breaks the invariant
                                     that the timestamp column must be
                                     increasing.
    \throws ZeroElements If the input iterator produces zero elements.        */
    template<typename InputIt>
    void addTimestamps(InputIt first, 
                       InputIt last) {
        {
        using namespace internal;
        static_assert(is_dereferencable<InputIt>, "Input iterator (InputIt) is "
                      "not dereferencable. It does not support 'operator*()'.");

        static_assert(std::is_constructible<TS, decltype(*first)>::value, 
                      "The type of the value produced by dereferencing the "
                      "input iterator (InputIt) does not match template "
                      "parameter TS (timestamp) used to instantiate "
                      "DataTable.");

        static_assert(is_eq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator==' and so is not comparable for " 
                      "equality.");

        static_assert(is_neq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator!=' and so is not comparable for " 
                      "inequality.");
        }

        if(first == last)
            throw ZeroElements{"Input iterator produced zero elements"};

        throwIfDataHasZeroRows();

        while(first != last) {
            throwIfTimestampsFull();
            throwIfTimestampBreaksInvariantPrev(timestamps_.size(), *first);

            timestamps_.push_back(*first);
            ++first;
        }
    }

    /** Add (append) timestamps to the timestamp column using a container. The
    container is required to support an iterator. In other words, the container
    must have member functions %begin() and %end() that emit an iterator to the
    container. Calling this function is equivalent to calling addTimestamps
    taking an iterator:
    \code
    addTimestamps(container.begin(), container.end());
    \endcode                                                                  */
    template<typename Container>
    void addTimestamps(const Container& container) {
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

        addTimestamps(container.begin(), container.end());
    }

    /** Add (append) timestamps to the timestamp column using a braced "{}" list
    of values (std::initializer_list). See addTimestamps() taking an iterator
    for details.                                                              */
    template<typename Timestamp>
    void addTimestamps(const std::initializer_list<Timestamp>& list) {
        addTimestamps(list.begin(), list.end());
    }

    /** Add a timestamp and a row in one function call. 
    - The first argument is forwarded to addTimestamp(). See documentation for
      addTimestamp().
    - Rest of the arguments are forwarded to DataTable_::addRow(). See
      documentation for addRow() member function overloads of DataTable_.     */
    template<typename... ArgsToAddRow>
    void addTimestampAndRow(TS timestamp, ArgsToAddRow&&... argsToAddRow) {
        this->addRow(std::forward<ArgsToAddRow>(argsToAddRow)...);
        addTimestamp(timestamp);
    }

    /** Add multiple timestamps and rows in one function call.
    - The first two arguments are iterators for timestamp. These are forwarded
      to addTimestamps() taking an iterator. See documentation for 
      addTimestamps() taking an iterator.
    - Rest of the arguments are forwarded to DataTable_::addRows(). See
      documentation for addRows() member function overloads of DataTable_.    */
    template<typename TimestampInputIt, typename... ArgsToAddRows>
    void addTimestampsAndRows(TimestampInputIt timestampFirst,
                              TimestampInputIt timestampLast,
                              ArgsToAddRows&&... argsToAddRows) {
        this->addRows(std::forward<ArgsToAddRows>(argsToAddRows)...);
        addTimestamps(timestampFirst, timestampLast);
    }

    /** Add multiple timestamps and rows in one function call using containers.
    - The first argument is the timestamp container and is forwarded to the
      function addTimestamps() taking a container. See its documentation for
      details.
    - Rest of the arguments are forwarded to DataTable_::addRows(). See
      documentation for addRows() overloads of DataTable_.                    */
    template<typename TimestampContainer, typename... ArgsToAddRows>
    void addTimestampsAndRows(const TimestampContainer& timestampContainer,
                              ArgsToAddRows&&... argsToAddRows) {
        this->addRows(std::forward<ArgsToAddRows>(argsToAddRows)...);
        addTimestamps(timestampContainer);
    }

    /** Add multiple timestamps and rows in one function call using a braced 
    "{}" list of values (std::initializer_list).
    - The first argument is a list containing timestamps. It is forwarded to the
      function addTimestamps() taking a std::initializer_list. See its 
      documentation for details.
    - The rest of the arguments are forwarded to DataTable_::addRows(). See its
      documenation for details.                                               */
    template<typename Timestamp, typename... ArgsToAddRows>
    void addTimestampsAndRows(const std::initializer_list<Timestamp>& list,
                              ArgsToAddRows&&... argsToAddRows) {
        this->addRows(std::forward<ArgsToAddRows>(argsToAddRows)...);
        addTimestamps(list);
    }

    /** Get the timestamp of a row.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimestampsLengthIncorrect If the length of the timestamp column does
                                      not match the number of rows in the
                                      DataTable.                             
    \throws RowDoesNotExist If the row specified by rowIndex does not exist.  */
    TS getTimestamp(size_t rowIndex) const {
        throwIfDataHasZeroRows();
        throwIfTimestampsLengthIncorrect();
        this->throwIfRowDoesNotExist(rowIndex);

        return timestamps_[rowIndex];
    }

    /** Get the timestamp that is closest to the given timestamp. Closeness can
    be specified with \a direction argument to retrieve:
    - Closest timestamp that is less than or equal to the given timestamp.
    - Closest timestamp that is greater than or equal to the given timestamp.
    - Closer of the above two.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimestampsLengthIncorrect If the length of the timestamp column does
                                      not match the number of rows in the
                                      DataTable.                             
    \throws TimestampDoesNotExist (1) When direction is \a LessThanEqual and 
                                  there is no timestamp satisfying that 
                                  criteria.
                                  (2) When direction is \a GreaterThanEqual and
                                  there is no timestamp satisfying that
                                  criteria.                                   */
    TS getTimestamp(TS timestamp, NearestDir direction) {
        throwIfDataHasZeroRows();
        throwIfTimestampsLengthIncorrect();

        auto geq_iter = std::lower_bound(timestamps_.cbegin(), 
                                         timestamps_.cend(),
                                         timestamp);

        if(direction == NearestDir::LessOrGreaterThanEqual) {
            if(geq_iter == timestamps_.cend())
                return timestamps_.back();

            if(*geq_iter == timestamp)
                return timestamp;

            if(geq_iter == timestamps_.cbegin())
                return timestamps_.front();

            if(*geq_iter -  timestamp <= timestamp - *(geq_iter - 1))
                return *geq_iter;
            else
                return *(--geq_iter);
        } else if(direction == NearestDir::LessThanEqual) {
            if(geq_iter == timestamps_.cend())
                return timestamps_.back();

            if(*geq_iter == timestamp)
                return timestamp;

            if(geq_iter == timestamps_.cbegin())
                throw TimestampDoesNotExist{"There is no timestamp less-than/"
                        "equal-to " + std::to_string(timestamp) + "."};

            return *(--geq_iter);
        } else if(direction == NearestDir::GreaterThanEqual) {
            if(geq_iter == timestamps_.cend())
                throw TimestampDoesNotExist{"There is no timestamp " 
                        "greater-than/equal-to " + std::to_string(timestamp) + 
                        "."};

            return *geq_iter;
        }
    }

    /** Get all the timestamps in a container. Returns an object that can be
    used in a range-for statement to iterate over the timestamps. The iterator
    does not allow writing.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimestampsLengthIncorrect If the length of the timestamp column does
                                      not match the number of rows in the
                                      DataTable.                              */
    TimestampsContainerProxy getTimestamps() const {
        throwIfDataHasZeroRows();
        throwIfTimestampsLengthIncorrect();

        return this;
    }

    /** Change the timestamp for a row.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws RowDoesNotExist If the row specified by \a rowIndex does not exist. 
    \throws TimestampDoesNotExist If the row specified by \a rowIndex does not
                                  have an associated timestamp.
    \throws TimestampBreaksInvariant If the new timestamp breaks the invariant
                                     that the timestamp column must be
                                     increasing.                              */
    void changeTimestampOfRow(size_t rowIndex, TS newTimestamp) {
        throwIfDataHasZeroRows();
        this->throwIfRowDoesNotExist(rowIndex);
        throwIfIndexExceedsTimestampLength(rowIndex);
        throwIfBreaksInvariantPrev(rowIndex, newTimestamp);
        throwIfBreaksInvariantNext(rowIndex, newTimestamp);

        timestamps_[rowIndex] = newTimestamp;
    }

    /** Change timestamp.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimestampsEmpty If the timestamp column is empty.
    \throws TimestampDoesNotExist If oldTimestamp does not exist in the 
                                  timestamp column.                           */
    void changeTimestamp(TS oldTimestamp, TS newTimestamp) {
        throwIfDataHasZeroRows();
        throwIfTimestampsEmpty();

        auto iter = std::lower_bound(timestamps_.begin(), 
                                     timestamps_.end(),
                                     oldTimestamp);

        if(*iter != oldTimestamp)
            throw TimestampDoesNotExist{"Timestamp '" + 
                    std::to_string(oldTimestamp) + "' does not exist."};
            

        throwifBreaksInvariantPrev(iter - timestamps_.cbegin(), newTimestamp);
        throwifBreaksInvariantNext(iter - timestamps_.cbegin(), newTimestamp);

        *iter = newTimestamp;
    }

    /** Change multiple timestamps starting at a given row using an iterator.
    The old timestamps at those rows will be replaced with new timestamps
    produced by the iterator.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws RowDoesNotExist If the row for which the iterator is trying to 
                            replace the timestamp does not exist. This can 
                            happen for example if the iterator produces more
                            elements than needed.
    \throws TimestampDoesNotExist If the row for which the iterator is tyring to
                                  replace timestamp does not have an associated
                                  timestamp.
    \throws TimestampBreaksInvariant If the new timestamp breaks the invariant
                                     that the timestamp column must be
                                     increasing.                              */
    template<typename InputIt>
    void changeTimestamps(InputIt first, 
                          InputIt last, 
                          size_t startAtRowIndex = 0) {
        {
        using namespace internal;
        static_assert(is_dereferencable<InputIt>, "Input iterator (InputIt) is "
                      "not dereferencable. It does not support 'operator*()'.");

        static_assert(std::is_constructible<TS, decltype(*first)>::value, 
                      "The type of the value produced by dereferencing the "
                      "input iterator (InputIt) does not match template "
                      "parameter TS (timestamp) used to instantiate "
                      "DataTable.");

        static_assert(is_eq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator==' and so is not comparable for " 
                      "equality.");

        static_assert(is_neq_comparable<InputIt>, "Input iterator does not " 
                      "support 'operator!=' and so is not comparable for " 
                      "inequality.");
        }
        
        throwIfDataHasZeroRows();

        size_t rowIndex{startAtRowIndex};
        while(first != last) {
            this->throwIfRowDoesNotExist(rowIndex);
            throwIfIndexExceedsTimestampLength(rowIndex);
            throwIfTimestampBreaksInvariantPrev(rowIndex, *first);
            throwIfTimestampBreaksInvariantNext(rowIndex, *first);
            
            timestamps_[rowIndex] = *first;

            ++first; ++rowIndex;
        }
    }

    /** Change multiple timestamps starting at a given row using a container.
    The old timestamps at those rows will be replaced with timestamps from the
    container. Calling this function is equivalent to caling:
    \code
    changeTimestamps(container.begin(),
                     container.end(),
                     startAtRowIndex)
    \endcode                                                                  
    See documentation for changeTimestamps() taking an iterator pair.         */
    template<typename Container>
    void changeTimestamps(const Container& container, 
                          size_t startAtRowIndex = 0) {
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

        changeTimestamps(container.begin(), container.end(), startAtRowIndex);
    }

    /** Change multiple timestamps starting at a given row using a braced "{}"
    list of timestamps. Calling this function is equivalent to calling:
    \code
    changeTimestamps(list.begin(), list.end(), startAtRowIndex)
    \endcode
    See documenation of changeTimestamps() taking an iterator for details.    */
    template<typename Timestamp>
    void changeTimestamps(const std::initializer_list<Timestamp>& list,
                          size_t startAtRowIndex = 0) {
        changeTimestamps(list.begin(), list.end(), startAtRowIndex);
    }

    /** Get the row index of a timestamp.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimestampsLengthIncorrect If the timestamp column length does not
                                      match the number of rows in the DataTable.
    \throws TimestampDoesNotExist If the given timestamp is not found in the
                                  timestamp column.                           */
    size_t getRowIndex(TS timestamp) const {
        throwIfDataHasZeroRows();
        throwIfTimestampsLengthIncorrect();

        auto iter = std::lower_bound(timestamps_.cbegin(), 
                                     timestamps_.cend(), 
                                     timestamp);

        if(*iter != timestamp)
            throw TimestampDoesNotExist{"Timestamp '" + 
                    std::to_string(timestamp) + "' does not exist."};
            
        return iter - timestamps_.cbegin();
    }

    /** Get the row index of the row whose timestamp is closest to the given
    timestamp. Closeness can be specified with \a direction argument to 
    retrieve:
    - Closest timestamp that is less than or equal to the given timestamp.
    - Closest timestamp that is greater than or equal to the given timestamp.
    - Closer of the above two.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimestampsLengthIncorrect If the length of the timestamp column does
                                      not match the number of rows in the
                                      DataTable.
    \throws TimestampDoesNotExist (1) When direction is \a LessThanEqual and
                                  there is no timestamp satisfying that 
                                  criteria.
                                  (2) When direction is \a GreaterThanEqual and
                                  there is no timestamp satisfying that
                                  criteria.                                   */
    size_t getRowIndex(TS timestamp, NearestDir direction ) const {
        throwIfDataHasZeroRows();
        throwIfTimestampsLengthIncorrect();

        auto geq_iter = std::lower_bound(timestamps_.cbegin(),
                                         timestamps_.cend(),
                                         timestamp);

        if(direction == NearestDir::LessOrGreaterThanEqual) {
            if(geq_iter == timestamps_.cend())
                return timestamps_.size() - 1;
            
            if(*geq_iter == timestamp)
                return geq_iter - timestamps_.cbegin();

            if(geq_iter == timestamps_.cbegin())
                return 0;

            if(*geq_iter - timestamp <= timestamp - *(geq_iter - 1))
                return geq_iter - timestamps_.cbegin();
            else
                return geq_iter - timestamps_.cbegin() - 1;
        } else if(direction == NearestDir::LessThanEqual) {
            if(geq_iter == timestamps_.cend())
                return timestamps_.size() - 1;

            if(*geq_iter == timestamp)
                return geq_iter - timestamps_.cbegin();

            if(geq_iter == timestamps_.cbegin())
                throw TimestampDoesNotExist{"There is no timestamp less-than/"
                        "equal-to " + std::to_string(timestamp) + "."};

            return geq_iter - timestamps_.cbegin() - 1;
        } else if(direction == NearestDir::GreaterThanEqual) {
            if(geq_iter == timestamps_.cend())
                throw TimestampDoesNotExist{"There is no timestamp "
                        "greater-than/equal-to " + std::to_string(timestamp) + 
                        "."};

            return geq_iter - timestamps_.cbegin();
        } 
        // This is to suppress compiler warning. Control *cannot* reach here.
        return 0;
    }

    /** Get the row corresponding to the given timestamp. This is equivalent to
    calling:
    \code
    getRow(getRowIndex(timestamp))
    \endcode
    See documentation for getRowIndex() and getRow() for details.             */
    SimTK::RowVectorView_<ET> getRowOfTimestamp(TS timestamp) const {
        return this->getRow(getRowIndex(timestamp));
    }

    /** Get the row with timestamp closest to the given timestamp. Closeness can
    be specified with \a direction argument. The returned row is not writable.
    This is equivalent to calling:
    \code
    getRow(getRowIndex(timestamp, direction))
    \endcode
    See documenation for getRowIndex() and getRow() for details.              */
    SimTK::RowVectorView_<ET> getRowOfTimestamp(TS timestamp, 
                                                NearestDir direction) const {
        return this->getRow(getRowIndex(timestamp, direction));
    }

    /** Update the row corresponding to the given timestamp. The returned row is
    writable. This is equivalent to calling:
    \code
    updRow(getRowIndex(timestamp))
    \endcode
    See documentation for getRowIndex() and updRow() for details.             */
    SimTK::RowVectorView_<ET> updRowOfTimestamp(TS timestamp) {
        return this->updRow(getRowIndex(timestamp));
    }

    /** Update the row with timestamp closest to the given timestamp. Closeness 
    can be specified with \a direction argument. The returned row is writable.
    This is equivalent to calling:
    \code
    updRow(getRowIndex(timestamp, direction))
    \endcode
    See documenation for getRowIndex() and updRow() for details.              */
    SimTK::RowVectorView_<ET> updRowOfTimestamp(TS timestamp, 
                                                NearestDir direction) {
        return this->updRow(getRowIndex(timestamp, direction));
    }

    /** Get the element for (timestamp, column-index) pair. The returned element
    is not writable. This is equivalent to calling:
    \code
    getElt(getRowIndex(timestamp), columnIndex)
    \endcode
    See documentation for getRowIndex() and getElt() for details.             */
    const ET& getEltOfTimestamp(TS timestamp, size_t columnIndex) const {
        return this->getElt(getRowIndex(timestamp), columnIndex);
    }

    /** Get the element for (timestamp, column-label) pair. The returned element
    is not writable. This is equivalent to calling:
    \code
    getElt(getRowIndex(timestamp), columnLabel)
    \endcode
    See documentation for getRowIndex() and getElt() for details.             */
    const ET& getEltOfTimestamp(TS timestamp, const string& columnLabel) const {
        return this->getElt(getRowIndex(timestamp), columnLabel);
    }

    /** Get the element for (row-index, column-index) pair where the row-index
    is the index of the row whose timestamp is closest to the given timestamp.
    Closeness can be specified with \a direction argument. The returned element
    is not writable.
    This is equivalent to calling:
    \code
    getElt(getRowIndex(timestamp, direction), columnIndex)
    \endcode
    See documentation for getRowIndex() and getElt() for details.             */
    const ET& getEltOfTimestamp(TS timestamp, 
                                size_t columnIndex, 
                                NearestDir direction) const {
        return this->getElt(getRowIndex(timestamp, direction), columnIndex);
    }

    /** Get the element for (row-index, column-label) pair where the row-index
    is the index of the row whose timestamp is closest to the given timestamp.
    Closeness can be specified with \a direction argument. The returned element
    is not writable.
    This is equivalent to calling:
    \code
    getElt(getRowIndex(timestamp, direction), columnLabel)
    \endcode
    See documentation for getRowIndex() and getElt() for details.             */
    const ET& getEltOfTimestamp(TS timestamp,
                                const string& columnLabel,
                                NearestDir direction) const {
        return this->getElt(getRowIndex(timestamp, direction), columnLabel);
    }

    /** Update the element for (timestamp, column-index) pair. The returned 
    element is writable. This is equivalent to calling:
    \code
    updElt(getRowIndex(timestamp), columnIndex)
    \endcode
    See documentation for getRowIndex() and updElt() for details.             */
    ET& updEltOfTimestamp(TS timestamp, size_t columnIndex) {
        return this->updElt(getRowIndex(timestamp), columnIndex);
    }

    /** Update the element for (timestamp, column-label) pair. The returned 
    element is writable. This is equivalent to calling:
    \code
    updElt(getRowIndex(timestamp), columnLabel)
    \endcode
    See documentation for getRowIndex() and updElt() for details.             */
    ET& updEltOfTimestamp(TS timestamp, const string& columnLabel) {
        return this->updElt(getRowIndex(timestamp), columnLabel);
    }

    /** Update the element for (row-index, column-index) pair where the 
    row-index is the index of the row whose timestamp is closest to the given 
    timestamp. Closeness can be specified with \a direction argument. The 
    returned element is writable.
    This is equivalent to calling:
    \code
    updElt(getRowIndex(timestamp, direction), columnIndex)
    \endcode
    See documentation for getRowIndex() and updElt() for details.             */
    ET& updEltOfTimestamp(TS timestamp, 
                          size_t columnIndex, 
                          NearestDir direction) {
        return this->updElt(getRowIndex(timestamp, direction), columnIndex);
    }

    /** Update the element for (row-index, column-label) pair where the 
    row-index is the index of the row whose timestamp is closest to the given 
    timestamp. Closeness can be specified with \a direction argument. The 
    returned element is writable.
    This is equivalent to calling:
    \code
    updElt(getRowIndex(timestamp, direction), columnLabel)
    \endcode
    See documentation for getRowIndex() and updElt() for details.             */
    ET& updEltOfTimestamp(TS timestamp,
                          const string& columnLabel,
                          NearestDir direction) {
        return this->updElt(getRowIndex(timestamp, direction), columnLabel);
    }

    /** Get a const iterator (representing the beginning) to iterate over 
    timestamps. Get the sentinel iterator using timestampsCEnd(). The iterator 
    does not allow writing to timestamps.                                     */
    TimestampsConstIter timestampsCBegin() const {
        return timestamps_.cbegin();
    }

    /** Get a const iterator (representing the end) to iterate over timestamps.
    Get the beginning iterator using timestampsCBegin().                      */
    TimestampsConstIter timestampsCEnd() const {
        return timestamps_.cend();
    }

    /** Get a const iterator (representing the beginning) to iterate over 
    timestamps. Get the sentinel iterator using timestampsEnd(). The iterator 
    does not allow writing to timestamps.                                     */
    TimestampsConstIter timestampsBegin() const {
        return timestamps_.cbegin();
    }

    /** Get a const iterator (representing the end) to iterate over timestamps. 
    Get the beginning iterator using timestampsBegin().                       */
    TimestampsConstIter timestampsEnd() const {
        return timestamps_.cend();
    }

    /** Get a non-const iterator (representing the beginning) to iterate over 
    timestamps. Get the sentinel iterator using timestampsEnd(). The iterator 
    does allow writing to timestamps.                                         */
    TimestampsConstIter timestampsBegin() {
        return timestamps_.begin();
    }

    /** Get a non-const iterator (representing the end) to iterate over 
    timestamps. Get the beginning iterator using timestampsBegin().           */
    TimestampsConstIter timestampsEnd() {
        return timestamps_.end();
    }

protected:
    /** \cond */
    void throwIfTimestampsEmpty() const {
        if(timestamps_.empty())
            throw TimestampsEmpty{"Timestamp column is empty. Use setTimestamps"
                                  "() to set the timestamp column."};
    }

    void throwIfDataHasZeroRows() const {
        if(this->getNumRows() == 0)
            throw DataHasZeroRows{"DataTable currently has zero rows. There " 
                                  "can be no timestamps without data."};
    }

    void throwIfTimestampsLengthIncorrect() const {
        if(this->getNumRows() != timestamps_.size())
            throw TimestampsLengthIncorrect{"Timestamp column length (" + 
                    std::to_string(timestamps_.size()) + ") does not match the "
                    "number of rows (" + std::to_string(this->getNumRows()) + 
                    ") in the DataTable. Add timestamps to fix it."};
    }

    void throwIfTimestampBreaksInvariantPrev(const size_t rowIndex, 
                                             const TS newTimestamp) const {
        if(rowIndex > 0 && 
           timestamps_[rowIndex - 1] >= newTimestamp)
            throw TimestampBreaksInvariant{"The input timestamp '" + 
                    std::to_string(newTimestamp) + "' at row " + 
                    std::to_string(rowIndex) + 
                    " is less-than/equal-to previous timestamp '" + 
                    std::to_string(timestamps_[rowIndex - 1]) + "' at row " + 
                    std::to_string(rowIndex - 1) + " and so breaks the " 
                    "invariant that timestamp column must be increasing."};
    }

    void throwIfTimestampBreaksInvariantNext(const size_t rowIndex, 
                                             const TS newTimestamp) const {
        if(rowIndex < timestamps_.size() - 1 && 
           timestamps_[rowIndex + 1] <= newTimestamp)
            throw TimestampBreaksInvariant{"The input timestamp '" + 
                    std::to_string(newTimestamp) + "' at row " + 
                    std::to_string(rowIndex) + 
                    " is greater-than/equal-to next timestamp '" + 
                    std::to_string(timestamps_[rowIndex + 1]) + "' at row " + 
                    std::to_string(rowIndex + 1) + " and so breaks the " 
                    "invariant that timestamp column must be increasing."};
    }

    void throwIfIndexExceedsTimestampLength(const size_t rowIndex) const {
        if(rowIndex > timestamps_.size() - 1)
            throw TimestampDoesNotExist{"Timestamp column length is " + 
                    std::to_string(timestamps_.size()) + ". There is no " 
                    "timestamp for row " + std::to_string(rowIndex) + ". Use "
                    "addTimestamp(s) to add timestamps."};
    }

    void throwIfTimestampsFull() const {
        if(this->getNumRows() == timestamps_.size())
            throw TimestampsColumnFull{"Both timestamp column length and number"
                    " of rows currently are " + 
                    std::to_string(timestamps_.size()) + ". Timestamp column "
                    "length cannot exceed number of rows in DataTable. Add a " 
                    "row before adding another timestamp."};
    }

    Timestamps timestamps_;

    /** \endcond */
};

} // namespace OpenSim

#endif // OPENSIM_COMMON_TIMESERIESDATATABLE_H_
