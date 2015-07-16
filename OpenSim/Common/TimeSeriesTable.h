/* -------------------------------------------------------------------------- *
 *                            OpenSim:  TimeSeriesTable.h                     *
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
This file defines the TimeSeriesTable_ class, which is used by OpenSim to 
provide an in-memory container for data access and manipulation.              */

#ifndef OPENSIM_COMMON_TIMESERIESDATATABLE_H_
#define OPENSIM_COMMON_TIMESERIESDATATABLE_H_

#include "OpenSim/Common/DataTable.h"


namespace OpenSim {

/** \cond */
class TimeColumnEmpty : public OpenSim::Exception {
public:
    TimeColumnEmpty(const std::string& expl) : Exception(expl) {}
};


class DataHasZeroRows : public OpenSim::Exception {
public:
    DataHasZeroRows(const std::string& expl) : Exception(expl) {}
};


class TimeColumnLengthIncorrect : public OpenSim::Exception {
public:
    TimeColumnLengthIncorrect(const std::string& expl) : Exception(expl) {}
};


class TimeDoesNotExist : public OpenSim::Exception {
public:
    TimeDoesNotExist(const std::string& expl) : Exception(expl) {}
};


class TimeBreaksInvariant : public OpenSim::Exception {
public:
    TimeBreaksInvariant(const std::string& expl) : Exception(expl) {}
};

class TimeColumnFull : public OpenSim::Exception {
public:
    TimeColumnFull(const std::string& expl) : Exception(expl) {}
};
/** \endcond */

enum class NearestDir {
    LessOrGreaterThanEqual,
    LessThanEqual,
    GreaterThanEqual
};

/** TimeSeriesTable_ is a DataTable_ that adds support for a time column. 
The time column can be of any arithmetic type -- float, double, int, long etc. 
In this documentaion, words time & timestamp are used interchangeably to mean
an entry of the time column.

The time column is enforced to be strictly increasing. Entries in the 
time-series column can be used to access the rows of the DataTable.           

\tparam ET Type of the entries in the underlying matrix. Defaults to
           SimTK::Real (alias for double).
\Tparam TS Type of the time column.                                           */
template<typename ET = SimTK::Real, typename TS = SimTK::Real>
class TimeSeriesTable_ : public DataTable_<ET> {
    static_assert(std::is_arithmetic<TS>::value, "Template argument 'TS' "
                  "representing type of time column must be an arithmetic type "
                  "(eg. int, float, double etc.).");

protected:
    /** \cond */
    using string = std::string;
    using TimeColumn = std::vector<TS>;
    using TimeColumnIter = typename TimeColumn::iterator;
    using TimeColumnConstIter = typename TimeColumn::const_iterator;

    class TimesContainerProxy {
    public:
        TimesContainerProxy(const TimeSeriesTable_* tsdt) : tsdt_{tsdt} {}
        TimesContainerProxy()                                         = delete;
        TimesContainerProxy(const TimesContainerProxy&)               = default;
        TimesContainerProxy(TimesContainerProxy&&)                    = default;
        TimesContainerProxy& operator=(const TimesContainerProxy&)    = default;
        TimesContainerProxy& operator=(TimesContainerProxy&&)         = default;

        TimeColumnConstIter cbegin() const {
            return tsdt_->timesBegin();
        }

        TimeColumnConstIter cend() const {
            return tsdt_->timesEnd();
        }

        TimeColumnConstIter begin() const {
            return cbegin();
        }

        TimeColumnConstIter end() const {
            return cend();
        }

    private:
        const TimeSeriesTable_* tsdt_;
    };
    /** \endcond */

public:
    using time_type = TS;

    /** Inherit constructors.                                                 */
    using DataTable_<ET>::DataTable_;

    /** Copy.                                                                 */
    TimeSeriesTable_& operator=(const TimeSeriesTable_&) = default;
    TimeSeriesTable_& operator=(TimeSeriesTable_&&)      = default;

    /** Destroy.                                                              */
    ~TimeSeriesTable_() override = default;

    /** Clear the time column.                                                */
    void clearTimeColumn() {
        times_.clear();
    }

    /** Check if the time column is empty.                                    */
    bool isTimeColumnEmpty() const {
        return times_.empty();
    }

    /** Check if the DataTable has a timestamp. Time complexity for this 
    function is O(log n) where 'n' is the length of the time column.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimeColumnLengthIncorrect If the length of the time column does not 
                                      match the number of rows in the 
                                      DataTable.                              */
    template<typename Time>
    bool hasTime(Time time) const {
        throwIfDataHasZeroRows();
        throwIfTimeColumnLengthIncorrect();

        return std::binary_search(times_.cbegin(), times_.cend(), time);
    }

    /** Get range (min, max) of time column. Returns a pair (std::pair) of 
    values where the first value is the min and the second value is the max.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimeColumnLengthIncorrect If the length of the time column does not 
                                      match the number of rows in the 
                                      DataTable.                              */
    std::pair<TS, TS> getTimeRange() {
        throwIfDataHasZeroRows();
        throwIfTimeColumnLengthIncorrect();

        return {times_.front(), times_.back()};
    }

    /** Add (append) a time to the time column.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimesColumnFull If the length of the time column is already equal to
                            the number of rows in the DataTable. At this point 
                            no timestamps can be added.
    \throws TimeBreaksInvariant If the new timestamp breaks the invariant that 
                                the times column must be increasing.          */
    void addTime(TS time) {
        throwIfDataHasZeroRows();
        throwIfTimeColumnFull();
        throwIfTimeBreaksInvariantPrev(times_.size(), time);

        times_.push_back(time);
    }

    /** Add (append) timestamps to the time column using an iterator. This is 
    equivalent to calling addTime() with a single timestamp multiple times.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimeColumnFull If the iterator produces more values than needed to 
                           fill up the timestamp column. The time column is not 
                           allowed to have length past the number of rows.
    \throws TimeBreaksInvariant If the a new timestamp breaks the invariant
                                that the timestamp column must be increasing.
    \throws ZeroElements If the input iterator produces zero elements.        */
    template<typename InputIt>
    void addTimes(InputIt first, InputIt last) {
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
            throwIfTimeColumnFull();
            throwIfTimeBreaksInvariantPrev(times_.size(), *first);

            times_.push_back(*first);
            ++first;
        }
    }

    /** Add (append) timestamps to the time column using a container. The
    container is required to support an iterator. In other words, the container
    must have member functions %begin() and %end() that emit an iterator to the
    container. Calling this function is equivalent to calling addTimes()
    taking an iterator:
    \code
    addTimes(container.begin(), container.end());
    \endcode                                                                  */
    template<typename Container>
    void addTimes(const Container& container) {
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

        addTimes(container.begin(), container.end());
    }

    /** Add (append) timestamps to the time column using a braced "{}" list of 
    values (std::initializer_list). See addTimes() taking an iterator for 
    details.                                                                  */
    template<typename Time>
    void addTimes(const std::initializer_list<Time>& list) {
        addTimes(list.begin(), list.end());
    }

    /** Add a timestamp and a row in one function call. 
    - The first argument is forwarded to addTime(). See documentation for
      addTime().
    - Rest of the arguments are forwarded to DataTable_::addRow(). See
      documentation for addRow() member function overloads of DataTable_.     */
    template<typename... ArgsToAddRow>
    void addTimeAndRow(TS time, ArgsToAddRow&&... argsToAddRow) {
        this->addRow(std::forward<ArgsToAddRow>(argsToAddRow)...);
        addTime(time);
    }

    /** Add multiple timestamps and rows in one function call.
    - The first two arguments are iterators for timestamp. These are forwarded
      to addTimestamps() taking an iterator. See documentation for 
      addTimestamps() taking an iterator.
    - Rest of the arguments are forwarded to DataTable_::addRows(). See
      documentation for addRows() member function overloads of DataTable_.    */
    template<typename TimeInputIt, typename... ArgsToAddRows>
    void addTimesAndRows(TimeInputIt timeFirst,
                         TimeInputIt timeLast,
                         ArgsToAddRows&&... argsToAddRows) {
        this->addRows(std::forward<ArgsToAddRows>(argsToAddRows)...);
        addTimes(timeFirst, timeLast);
    }

    /** Add multiple timestamps and rows in one function call using containers.
    - The first argument is the timestamp container and is forwarded to the
      function addTimes() taking a container. See its documentation for
      details.
    - Rest of the arguments are forwarded to DataTable_::addRows(). See
      documentation for addRows() overloads of DataTable_.                    */
    template<typename TimeContainer, typename... ArgsToAddRows>
    void addTimesAndRows(const TimeContainer& timeContainer,
                         ArgsToAddRows&&... argsToAddRows) {
        this->addRows(std::forward<ArgsToAddRows>(argsToAddRows)...);
        addTimes(timeContainer);
    }

    /** Add multiple timestamps and rows in one function call using a braced 
    "{}" list of values (std::initializer_list).
    - The first argument is a list containing timestamps. It is forwarded to the
      function addTimes() taking a std::initializer_list. See its 
      documentation for details.
    - The rest of the arguments are forwarded to DataTable_::addRows(). See its
      documenation for details.                                               */
    template<typename Time, typename... ArgsToAddRows>
    void addTimesAndRows(const std::initializer_list<Time>& list,
                         ArgsToAddRows&&... argsToAddRows) {
        this->addRows(std::forward<ArgsToAddRows>(argsToAddRows)...);
        addTimes(list);
    }

    /** Get the timestamp of a row. Time complexity is O(1).

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimestampsLengthIncorrect If the length of the timestamp column does
                                      not match the number of rows in the
                                      DataTable.                             
    \throws RowDoesNotExist If the row specified by rowIndex does not exist.  */
    TS getTime(size_t rowIndex) const {
        throwIfDataHasZeroRows();
        throwIfTimeColumnLengthIncorrect();
        this->throwIfRowDoesNotExist(rowIndex);

        return times_[rowIndex];
    }

    /** Get the timestamp that is closest/nearest to the given timestamp. 
    If the argument \a direction is:
    - LessThanEqual -- The timestamp returned is less than or equal to (<=) the 
      given timestamp.
    - GreaterThanEqual -- The timestamp returned is greater than or equal to 
      (>=) the given timestamp.
    - LessOrGreaterThanEqual -- The timestamp returned is best of the above two.
    Time complexity is O(log n) where n is time column length.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimeColumnLengthIncorrect If the length of the timestamp column does
                                      not match the number of rows in the
                                      DataTable.                              
    \throws TimeDoesNotExist This is applicable only for cases when \a direction
                             is either \a LessThanEqual or \a GreaterThanEqual. 
                             This exception is thrown if there is no timestamp 
                             satisfying the criteria.                         */
    TS getTime(TS time, NearestDir direction) {
        throwIfDataHasZeroRows();
        throwIfTimeColumnLengthIncorrect();

        auto geq_iter = std::lower_bound(times_.cbegin(), times_.cend(), time);
        
        if(direction == NearestDir::LessOrGreaterThanEqual) {
            if(geq_iter == times_.cend())
                return times_.back();

            if(*geq_iter == time)
                return time;

            if(geq_iter == times_.cbegin())
                return times_.front();

            if(*geq_iter -  time <= time - *(geq_iter - 1))
                return *geq_iter;
            else
                return *(--geq_iter);
        } else if(direction == NearestDir::LessThanEqual) {
            if(geq_iter == times_.cend())
                return times_.back();

            if(*geq_iter == time)
                return time;

            if(geq_iter == times_.cbegin())
                throw TimeDoesNotExist{"There is no timestamp less-than/"
                        "equal-to " + std::to_string(time) + "."};

            return *(--geq_iter);
        } else {
            if(geq_iter == times_.cend())
                throw TimeDoesNotExist{"There is no timestamp " 
                        "greater-than/equal-to " + std::to_string(time) + 
                        "."};

            return *geq_iter;
        }
    }

    /** Get all the timestamps in a container. Returns an object that can be
    used in a range-for statement to iterate over the timestamps. The iterator
    does not allow writing.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimeColumnLengthIncorrect If the length of the timestamp column does
                                      not match the number of rows in the
                                      DataTable.                              */
    TimesContainerProxy getTimes() const {
        throwIfDataHasZeroRows();
        throwIfTimeColumnLengthIncorrect();

        return this;
    }

    /** Change the timestamp for a row. Time complexity is O(1).

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws RowDoesNotExist If the row specified by \a rowIndex does not exist. 
    \throws TimeDoesNotExist If the row specified by \a rowIndex does not
                             have an associated timestamp.
    \throws TimeBreaksInvariant If the new timestamp breaks the invariant
                                that the timestamp column must be increasing. */
    void changeTimeOfRow(size_t rowIndex, TS newTime) {
        throwIfDataHasZeroRows();
        this->throwIfRowDoesNotExist(rowIndex);
        throwIfIndexExceedsTimeColumnLength(rowIndex);
        throwIfTimeBreaksInvariantPrev(rowIndex, newTime);
        throwIfTimeBreaksInvariantNext(rowIndex, newTime);

        times_[rowIndex] = newTime;
    }

    /** Change timestamp. Time complexity is O(log n) where n is time column
    length.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimeColumnEmpty If the timestamp column is empty.
    \throws TimeDoesNotExist If oldTimestamp does not exist in the 
                             timestamp column.                                */
    void changeTime(TS oldTime, TS newTime) {
        throwIfDataHasZeroRows();
        throwIfTimeColumnEmpty();

        auto iter = std::lower_bound(times_.begin(), times_.end(), oldTime);

        if(*iter != oldTime)
            throw TimeDoesNotExist{"Timestamp '" + 
                    std::to_string(oldTime) + "' does not exist."};
            

        throwifTimeBreaksInvariantPrev(iter - times_.cbegin(), newTime);
        throwifTimeBreaksInvariantNext(iter - times_.cbegin(), newTime);

        *iter = newTime;
    }

    /** Change multiple timestamps starting at a given row using an iterator.
    The old timestamps at those rows will be replaced with new timestamps
    produced by the iterator. Time complexity is o(n) where n is the number
    elements produced by the iterator.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws RowDoesNotExist If the row for which the iterator is trying to 
                            replace the timestamp does not exist. This can 
                            happen for example if the iterator produces more
                            elements than needed.
    \throws TimeDoesNotExist If the row for which the iterator is tyring to
                             replace timestamp does not have an associated
                             timestamp.
    \throws TimeBreaksInvariant If the new timestamp breaks the invariant
                                that the timestamp column must be increasing. */
    template<typename InputIt>
    void changeTimes(InputIt first, InputIt last, size_t startAtRowIndex = 0) {
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
            throwIfIndexExceedsTimeColumnLength(rowIndex);
            throwIfTimeBreaksInvariantPrev(rowIndex, *first);
            throwIfTimeBreaksInvariantNext(rowIndex, *first);
            
            times_[rowIndex] = *first;

            ++first; ++rowIndex;
        }
    }

    /** Change multiple timestamps starting at a given row using a container.
    The old timestamps at those rows will be replaced with timestamps from the
    container. Calling this function is equivalent to caling:
    \code
    changeTimes(container.begin(), container.end(), startAtRowIndex)
    \endcode                                                                  
    See documentation for changeTimes() taking an iterator pair.              */
    template<typename Container>
    void changeTimes(const Container& container, size_t startAtRowIndex = 0) {
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

        changeTimes(container.begin(), container.end(), startAtRowIndex);
    }

    /** Change multiple timestamps starting at a given row using a braced "{}"
    list of timestamps. Calling this function is equivalent to calling:
    \code
    changeTimes(list.begin(), list.end(), startAtRowIndex)
    \endcode
    See documenation of changeTimes() taking an iterator for details.         */
    template<typename Time>
    void changeTimes(const std::initializer_list<Time>& list,
                     size_t startAtRowIndex = 0) {
        changeTimes(list.begin(), list.end(), startAtRowIndex);
    }

    /** Get the row index of a timestamp. Time complexity is O(log n) where n
    is the length of the time column.

    \throws DataHasZeroRows If the DataTable currently has zero rows.
    \throws TimeColumnLengthIncorrect If the timestamp column length does not
                                      match the number of rows in the DataTable.
    \throws TimeDoesNotExist If the given timestamp is not found in the
                             timestamp column.                                */
    size_t getRowIndex(TS time) const {
        throwIfDataHasZeroRows();
        throwIfTimeColumnLengthIncorrect();

        auto iter = std::lower_bound(times_.cbegin(), times_.cend(), time);

        if(*iter != time)
            throw TimeDoesNotExist{"Timestamp '" + 
                    std::to_string(time) + "' does not exist."};
            
        return iter - times_.cbegin();
    }

    /** Get the row index of the row whose timestamp is closest to the given
    timestamp. Closeness can be specified with \a direction argument to 
    retrieve:
    - Closest timestamp that is less than or equal to the given timestamp.
    - Closest timestamp that is greater than or equal to the given timestamp.
    - Closer of the above two.
    Time complexity is o(log n) where n is the length of the time column.

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
    size_t getRowIndex(TS time, NearestDir direction ) const {
        throwIfDataHasZeroRows();
        throwIfTimeColumnLengthIncorrect();

        auto geq_iter = std::lower_bound(times_.cbegin(), times_.cend(), time);

        if(direction == NearestDir::LessOrGreaterThanEqual) {
            if(geq_iter == times_.cend())
                return times_.size() - 1;
            
            if(*geq_iter == time)
                return geq_iter - times_.cbegin();

            if(geq_iter == times_.cbegin())
                return 0;

            if(*geq_iter - time <= time - *(geq_iter - 1))
                return geq_iter - times_.cbegin();
            else
                return geq_iter - times_.cbegin() - 1;
        } else if(direction == NearestDir::LessThanEqual) {
            if(geq_iter == times_.cend())
                return times_.size() - 1;

            if(*geq_iter == time)
                return geq_iter - times_.cbegin();

            if(geq_iter == times_.cbegin())
                throw TimeDoesNotExist{"There is no timestamp less-than/"
                        "equal-to " + std::to_string(time) + "."};

            return geq_iter - times_.cbegin() - 1;
        } else {
            if(geq_iter == times_.cend())
                throw TimeDoesNotExist{"There is no timestamp "
                        "greater-than/equal-to " + std::to_string(time) + "."};

            return geq_iter - times_.cbegin();
        } 
    }

    /** Get the row corresponding to the given timestamp. This is equivalent to
    calling:
    \code
    getRow(getRowIndex(time))
    \endcode
    See documentation for getRowIndex() and getRow() for details.             */
    SimTK::RowVectorView_<ET> getRowAtTime(TS time) const {
        return this->getRow(getRowIndex(time));
    }

    /** Get the row with timestamp closest to the given timestamp. Closeness can
    be specified with \a direction argument. The returned row is not writable.
    This is equivalent to calling:
    \code
    getRow(getRowIndex(time, direction))
    \endcode
    See documenation for getRowIndex() and getRow() for details.              */
    SimTK::RowVectorView_<ET> getRowAtTime(TS time, 
                                           NearestDir direction) const {
        return this->getRow(getRowIndex(time, direction));
    }

    /** Update the row corresponding to the given timestamp. The returned row is
    writable. This is equivalent to calling:
    \code
    updRow(getRowIndex(time))
    \endcode
    See documentation for getRowIndex() and updRow() for details.             */
    SimTK::RowVectorView_<ET> updRowAtTime(TS time) {
        return this->updRow(getRowIndex(time));
    }

    /** Update the row with timestamp closest to the given timestamp. Closeness 
    can be specified with \a direction argument. The returned row is writable.
    This is equivalent to calling:
    \code
    updRow(getRowIndex(time, direction))
    \endcode
    See documenation for getRowIndex() and updRow() for details.              */
    SimTK::RowVectorView_<ET> updRowAtTime(TS time, 
                                           NearestDir direction) {
        return this->updRow(getRowIndex(time, direction));
    }

    /** Get the element for (timestamp, column-index) pair. The returned element
    is not writable. This is equivalent to calling:
    \code
    getElt(getRowIndex(time), columnIndex)
    \endcode
    See documentation for getRowIndex() and getElt() for details.             */
    const ET& getEltAtTime(TS time, size_t columnIndex) const {
        return this->getElt(getRowIndex(time), columnIndex);
    }

    /** Get the element for (timestamp, column-label) pair. The returned element
    is not writable. This is equivalent to calling:
    \code
    getElt(getRowIndex(time), columnLabel)
    \endcode
    See documentation for getRowIndex() and getElt() for details.             */
    const ET& getEltAtTime(TS time, const string& columnLabel) const {
        return this->getElt(getRowIndex(time), columnLabel);
    }

    /** Get the element for (row-index, column-index) pair where the row-index
    is the index of the row whose timestamp is closest to the given timestamp.
    Closeness can be specified with \a direction argument. The returned element
    is not writable.
    This is equivalent to calling:
    \code
    getElt(getRowIndex(time, direction), columnIndex)
    \endcode
    See documentation for getRowIndex() and getElt() for details.             */
    const ET& getEltAtTime(TS time, 
                           size_t columnIndex, 
                           NearestDir direction) const {
        return this->getElt(getRowIndex(time, direction), columnIndex);
    }

    /** Get the element for (row-index, column-label) pair where the row-index
    is the index of the row whose timestamp is closest to the given timestamp.
    Closeness can be specified with \a direction argument. The returned element
    is not writable.
    This is equivalent to calling:
    \code
    getElt(getRowIndex(time, direction), columnLabel)
    \endcode
    See documentation for getRowIndex() and getElt() for details.             */
    const ET& getEltAtTime(TS time,
                           const string& columnLabel,
                           NearestDir direction) const {
        return this->getElt(getRowIndex(time, direction), columnLabel);
    }

    /** Update the element for (timestamp, column-index) pair. The returned 
    element is writable. This is equivalent to calling:
    \code
    updElt(getRowIndex(time), columnIndex)
    \endcode
    See documentation for getRowIndex() and updElt() for details.             */
    ET& updEltAtTime(TS time, size_t columnIndex) {
        return this->updElt(getRowIndex(time), columnIndex);
    }

    /** Update the element for (timestamp, column-label) pair. The returned 
    element is writable. This is equivalent to calling:
    \code
    updElt(getRowIndex(time), columnLabel)
    \endcode
    See documentation for getRowIndex() and updElt() for details.             */
    ET& updEltAtTime(TS time, const string& columnLabel) {
        return this->updElt(getRowIndex(time), columnLabel);
    }

    /** Update the element for (row-index, column-index) pair where the 
    row-index is the index of the row whose timestamp is closest to the given 
    timestamp. Closeness can be specified with \a direction argument. The 
    returned element is writable.
    This is equivalent to calling:
    \code
    updElt(getRowIndex(time, direction), columnIndex)
    \endcode
    See documentation for getRowIndex() and updElt() for details.             */
    ET& updEltAtTime(TS time, size_t columnIndex, NearestDir direction) {
        return this->updElt(getRowIndex(time, direction), columnIndex);
    }

    /** Update the element for (row-index, column-label) pair where the 
    row-index is the index of the row whose timestamp is closest to the given 
    timestamp. Closeness can be specified with \a direction argument. The 
    returned element is writable.
    This is equivalent to calling:
    \code
    updElt(getRowIndex(time, direction), columnLabel)
    \endcode
    See documentation for getRowIndex() and updElt() for details.             */
    ET& updEltAtTime(TS time, const string& columnLabel, NearestDir direction) {
        return this->updElt(getRowIndex(time, direction), columnLabel);
    }

    /** Get a const iterator (representing the beginning) to iterate over 
    timestamps. Get the sentinel iterator using timestampsCEnd(). The iterator 
    does not allow writing to timestamps.                                     */
    TimeColumnConstIter timesCBegin() const {
        return times_.cbegin();
    }

    /** Get a const iterator (representing the end) to iterate over timestamps.
    Get the beginning iterator using timestampsCBegin().                      */
    TimeColumnConstIter timesCEnd() const {
        return times_.cend();
    }

    /** Get a const iterator (representing the beginning) to iterate over 
    timestamps. Get the sentinel iterator using timestampsEnd(). The iterator 
    does not allow writing to timestamps.                                     */
    TimeColumnConstIter timesBegin() const {
        return times_.cbegin();
    }

    /** Get a const iterator (representing the end) to iterate over timestamps. 
    Get the beginning iterator using timestampsBegin().                       */
    TimeColumnConstIter timesEnd() const {
        return times_.cend();
    }

    /** Get a non-const iterator (representing the beginning) to iterate over 
    timestamps. Get the sentinel iterator using timestampsEnd(). The iterator 
    does allow writing to timestamps.                                         */
    TimeColumnConstIter timesBegin() {
        return times_.begin();
    }

    /** Get a non-const iterator (representing the end) to iterate over 
    timestamps. Get the beginning iterator using timestampsBegin().           */
    TimeColumnConstIter timesEnd() {
        return times_.end();
    }

protected:
    /** \cond */
    void throwIfTimeColumnEmpty() const {
        if(times_.empty())
            throw TimeColumnEmpty{"Timestamp column is empty. Use setTimestamps"
                                  "() to set the timestamp column."};
    }

    void throwIfDataHasZeroRows() const {
        if(this->getNumRows() == 0)
            throw DataHasZeroRows{"DataTable currently has zero rows. There " 
                                  "can be no timestamps without data."};
    }

    void throwIfTimeColumnLengthIncorrect() const {
        if(this->getNumRows() != times_.size())
            throw TimeColumnLengthIncorrect{"Timestamp column length (" + 
                    std::to_string(times_.size()) + ") does not match the "
                    "number of rows (" + std::to_string(this->getNumRows()) + 
                    ") in the DataTable. Add timestamps to fix it."};
    }

    void throwIfTimeBreaksInvariantPrev(const size_t rowIndex, 
                                        const TS newTime) const {
        if(rowIndex > 0 && 
           times_[rowIndex - 1] >= newTime)
            throw TimeBreaksInvariant{"The input timestamp '" + 
                    std::to_string(newTime) + "' at row " + 
                    std::to_string(rowIndex) + 
                    " is less-than/equal-to previous timestamp '" + 
                    std::to_string(times_[rowIndex - 1]) + "' at row " + 
                    std::to_string(rowIndex - 1) + " and so breaks the " 
                    "invariant that timestamp column must be increasing."};
    }

    void throwIfTimeBreaksInvariantNext(const size_t rowIndex, 
                                        const TS newTime) const {
        if(rowIndex < times_.size() - 1 && 
           times_[rowIndex + 1] <= newTime)
            throw TimeBreaksInvariant{"The input timestamp '" + 
                    std::to_string(newTime) + "' at row " + 
                    std::to_string(rowIndex) + 
                    " is greater-than/equal-to next timestamp '" + 
                    std::to_string(times_[rowIndex + 1]) + "' at row " + 
                    std::to_string(rowIndex + 1) + " and so breaks the " 
                    "invariant that timestamp column must be increasing."};
    }

    void throwIfIndexExceedsTimeColumnLength(const size_t rowIndex) const {
        if(rowIndex > times_.size() - 1)
            throw TimeDoesNotExist{"Timestamp column length is " + 
                    std::to_string(times_.size()) + ". There is no " 
                    "timestamp for row " + std::to_string(rowIndex) + ". Use "
                    "addTimestamp(s) to add timestamps."};
    }

    void throwIfTimeColumnFull() const {
        if(this->getNumRows() == times_.size())
            throw TimeColumnFull{"Both timestamp column length and number"
                    " of rows currently are " + 
                    std::to_string(times_.size()) + ". Timestamp column "
                    "length cannot exceed number of rows in DataTable. Add a " 
                    "row before adding another timestamp."};
    }

    TimeColumn times_;

    /** \endcond */
};


using TimeSeriesTable = TimeSeriesTable_<SimTK::Real, SimTK::Real>;


} // namespace OpenSim

#endif // OPENSIM_COMMON_TIMESERIESDATATABLE_H_
