#ifndef OPENSIM_ARRAY_H_
#define OPENSIM_ARRAY_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Array.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Adam Kewley, Frank C. Anderson                                  *
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

/* Note: This code was originally developed by Realistic Dynamics Inc.
 * Author: Frank C. Anderson
 */

#include "Assertion.h"
#include "Exception.h"
#include "Logger.h"
#include "osimCommonDLL.h"
#include <algorithm>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <spdlog/fmt/bundled/ostream.h>
#include <type_traits>
#include <utility>
#include <vector>

namespace OpenSim {

// Not intended for end users => private
namespace detail {
// backwards-compat hack: the original implementation allows for references
// to booleans, which won't work if using std::vector<bool> specialization
// Moved outside the template class to ensure linking works properly in VS 17.14
// See issue (#4081) for details.
class BoolLike final {
public:
    BoolLike(bool value_) : value{value_} {}
    operator bool& () { return value; }
    operator const bool& () const { return value; }
private:
    bool value;
};
}

/**
 * A class for storing an array of values of type T.  The capacity of the class
 * grows as needed.  To use this template for a class of type T, class T should
 * implement the following methods:  default constructor, copy constructor,
 * assignment operator (=), equality operator (==), and less than
 * operator (<).
 */
template<class T>
class Array {

public:
    Array(Array const&) = default;
    Array(Array&&) noexcept = default;
    Array& operator=(Array const&) = default;
    Array& operator=(Array&&) noexcept = default;
    ~Array() noexcept = default;

    Array(std::initializer_list<T> initList) :
    _defaultValue{}, _storage(initList.begin(), initList.end())
    {}

    explicit Array(T aDefaultValue = T(), int aSize = 0, int aCapacity = 1) :
        _defaultValue{std::move(aDefaultValue)}
    {
        _storage.reserve(aCapacity);
        _storage.resize(aSize, _defaultValue);
    }

    // A non-operator version of operator== which we can use in Java
    // NOTE: I tried to name it "equals" since that's the standard way to compare objects in
    // Java, but didn't seem to work...  - Eran
    bool arrayEquals(const Array& aArray) const
    {
        return *this == aArray;
    }

#ifndef SWIG

    /**
     * Get the array element at a specified index.  This overloaded operator
     * can be used both to set and get element values:
     * @code
     *      Array<T> array(2);
     *      T value = array[i];
     *      array[i] = value;
     * @endcode
     *
     * This operator is intended for accessing array elements with as little
     * overhead as possible, so no error checking is performed.
     * The caller must make sure the specified index is within the bounds of
     * the array.  If error checking is desired, use Array::get().
     *
     * @param aIndex Index of the desired element (0 <= aIndex < _size).
     * @return Reference to the array element.
     * @see get().
     */
    T& operator[](int aIndex) const
    {
        return const_cast<T&>(static_cast<T const&>(_storage[aIndex]));  // uh oh (legacy behavior)
    }

    /**
     * Determine if two arrays are equal.
     *
     * Two arrays are equal if their contents are equal.  That is, each array
     * must be the same length and their corresponding array elements must be
     * equal.
     */
    friend bool operator==(const Array& lhs, const Array& rhs)
    {
        return lhs._storage == rhs._storage;
    }

    /**
     * Implementation of the output operator.
     * The output for an array looks like the following:\n\n
     *
     * T[0] T[1] T[2] ... T[size-1].
     *
     * @param out Output stream.
     * @param rhs Array to be output.
     * @return Reference to the output stream.
     */
    friend std::ostream& operator<<(std::ostream& out, const Array& rhs)
    {
        for (auto const& el : rhs._storage) {
            out << " " << el;
        }
        return out;
    }

    /**
     * Ensure that the capacity of this array is at least the specified amount.
     * Note that the newly allocated array elements are not initialized.
     *
     * @param aCapacity Desired capacity.
     * @return true if the capacity was successfully obtained, false otherwise.
     */
    bool ensureCapacity(int aCapacity)
    {
        _storage.reserve(aCapacity);
        return true;  // it can't fail (memory exhaustion == exception in C++)
    }
#endif

    /**
     * Trim the capacity of this array so that it is one larger than the size
     * of this array.  This is useful for reducing the amount of memory used
     * by this array.  This capacity is kept at one larger than the size so
     * that, for example, an array of characters can be treated as a NULL
     * terminated string.
     */
    void trim()
    {
        _storage.push_back(_defaultValue);
        _storage.shrink_to_fit();
        _storage.pop_back();
    }
#ifndef SWIG

    /**
     * Get the capacity of this storage instance.
     */
    int getCapacity() const
    {
        return static_cast<int>(_storage.capacity());
    }

    /**
     * deprecated (legacy): now has no effect
     *
     * OLD BEHAVIOR: set the amount by which the capacity is increased when the
     * capacity of the array is exceeded. If the specified increment is
     * negative, the capacity is set to double whenever the capacity is
     * exceeded.
     */
    [[deprecated("this no longer does anything")]]
    void setCapacityIncrement(int)
    {
        // noop
    }

    /**
     * deprecated (legacy): has no effect
     *
     * OLD BEHAVIOR: get the amount by which the capacity is increased.
     */
    [[deprecated("this no longer does anything")]]
    int getCapacityIncrement() const
    {
        return -1;
    }

#endif

    /**
     * Set the size of the array.  This method can be used to either increase
     * or decrease the size of the array.  If this size of the array is
     * increased, the new elements are initialized to the default value
     * that was specified at the time of construction.
     *
     * Note that the size of an array is different than its capacity.  The size
     * indicates how many valid elements are stored in an array.  The capacity
     * indicates how much the size of the array can be increased without
     * allocated more memory.  At all times size <= capacity.
     *
     * @param aSize Desired size of the array.  The size must be greater than
     * or equal to zero.
     */
    bool setSize(int aSize)
    {
        if (aSize < 0) {
            aSize = 0;  // legacy behavior
        }
        _storage.resize(aSize, _defaultValue);
        return true;
    }

    /**
     * Get the size of the array.
     *
     * @return Size of the array.
     */
    int getSize() const
    {
        return static_cast<int>(_storage.size());
    }

    /** Alternate name for getSize(). **/
    int size() const
    {
        return getSize();
    }

    /**
     * Append a value onto the array.
     *
     * @param aValue Value to be appended.
     * @return New size of the array, or, equivalently, the index to the new
     * first empty element of the array.
     */
    int append(const T& aValue)
    {
        _storage.push_back(aValue);
        return size();
    }

    /**
     * Append an array of values.
     *
     * @param aArray Array of values to append.
     * @return New size of the array, or, equivalently, the index to the new
     * first empty element of the array.
     */
    int append(const Array& aArray)
    {
        _storage.insert(_storage.end(), aArray._storage.begin(), aArray._storage.end());
        return size();
    }
#ifndef SWIG
    /**
     * Append an array of values.
     *
     * @param aSize Size of the array to append.
     * @param aArray Array of values to append.
     * @return New size of the array, or, equivalently, the index to the new
     * first empty element of the array.
     */
    int append(int aSize, const T* aArray)
    {
        if (aSize > 0 && aArray) {
            _storage.insert(_storage.end(), aArray, aArray + aSize);
        }
        return size();
    }
#endif
    /**
     * Insert a value into the array at a specified index.
     *
     * This method is relatively computationally costly since many of the array
     * elements may need to be shifted.
     *
     * @param aValue Value to be inserted.
     * @param aIndex Index at which to insert the new value.  All current elements
     * from aIndex to the end of the array are shifted one place in the direction
     * of the end of the array.  If the specified index is greater than the
     * current size of the array, the size of the array is increased to aIndex+1
     * and the intervening new elements are initialized to the default value that
     * was specified at the time of construction.
     * @return Size of the array after the insertion.
     */
    int insert(int aIndex, const T& aValue)
    {
        if (aIndex < 0) {
            OPENSIM_THROW(Exception, "Array.insert: aIndex was less than 0.");
        }
        else if (aIndex >= static_cast<int>(_storage.size())) {
            setSize(aIndex+1);
            _storage[aIndex] = aValue;
        }
        else {
            _storage.insert(_storage.begin() + aIndex, aValue);
        }
        return size();
    }

    /**
     * Remove a value from the array at a specified index.
     *
     * This method is relatively computationally costly since many of the array
     * elements may need to be shifted.
     *
     * @param aIndex Index of the value to remove.  All elements from aIndex to
     * the end of the array are shifted one place toward the beginning of
     * the array.  If aIndex is less than 0 or greater than or equal to the
     * current size of the array, no element is removed.
     * @return Size of the array after the removal.
     */
    int remove(int aIndex)
    {
        OPENSIM_ASSERT(0 <= aIndex && aIndex < size() && "Array::remove received an out-of-bounds index");
        _storage.erase(_storage.begin() + aIndex);
        return size();
    }

    /**
     * Set the value at a specified index.
     *
     * @param aIndex Index of the array element to be set.  It is permissible
     * for aIndex to be past the current end of the array- the capacity will
     * be increased if necessary.  Values between the current end of the array
     * and aIndex are not initialized.
     * @param aValue Value.
     */
    void set(int aIndex, const T& aValue)
    {
        if (aIndex < 0) {
            return;
        }

        if (aIndex+1 >= static_cast<int>(_storage.capacity())) {
            setSize(aIndex+1);
        }

        _storage[aIndex] = aValue;
    }
#ifndef SWIG

    /**
     * Get a pointer to the low-level array.
     *
     * @return Pointer to the low-level array.
     */
    T* get()
    {
        return _storage.data();
    }

    /**
     * Get a pointer to the low-level array.
     *
     * @return Pointer to the low-level array.
     */
    const T* get() const
    {
        return _storage.data();
    }
#endif

    /**
     * Get a const reference to the value at a specified array index.
     *
     * If the index is negative or passed the end of the array, an exception
     * is thrown.
     *
     * For faster execution, the array elements can be accessed through the
     * overloaded operator[], which does no bounds checking.
     *
     * @param aIndex Index of the desired array element.
     * @return const reference to the array element.
     * @throws Exception if (aIndex<0)||(aIndex>=_size).
     * @see operator[].
     */
    const T& get(int aIndex) const
    {
        if (0 <= aIndex && aIndex < size()) {
            return _storage[aIndex];
        }
        else {
            OPENSIM_THROW(Exception, "Array index out of bounds");
        }
    }

#ifndef SWIG

    /**
     * Get a writable reference to value at a specified array index.
     *
     * If the index is negative or passed the end of the array, an exception
     * is thrown.
     *
     * For faster execution, the array elements can be accessed through the
     * overloaded operator[], which does no bounds checking.
     *
     * @param aIndex Index of the desired array element.
     * @return Writable reference to the array element.
     * @throws Exception if (aIndex<0)||(aIndex>=_size).
     * @see operator[].
     */
    T& updElt(int aIndex) const
    {
        if (0 <= aIndex && aIndex < size()) {
            return const_cast<T&>(static_cast<T const&>(_storage[aIndex]));  // uh oh (legacy behavior)
        }
        else {
            OPENSIM_THROW(Exception, "Array index out of bounds");
        }
    }
#endif

#ifdef SWIG
  %extend {
    T getitem(int index) {
      return self->get(index);
    }
    void setitem(int index, T val) {
      self->set(index,val);
    }
  }
#endif
    /**
     * Get the last value in the array.
     *
     * @return Last value in the array.
     * @throws Exception if the array is empty.
     */
    const T& getLast() const
    {
        if (_storage.empty()) {
            OPENSIM_THROW(Exception, "Array is empty");
        }
        return _storage.back();
    }
#ifndef SWIG
    /**
     * Get writable reference to last value in the array.
     *
     * @return writable reference to Last value in the array.
     * @throws Exception if the array is empty.
     */
    T& updLast() const
    {
        if (_storage.empty()) {
            OPENSIM_THROW(Exception, "Array is empty");
        }
        return const_cast<T&>(static_cast<T const&>(_storage.back()));  // uh oh (legacy behavior)
    }
#endif

    /**
     * Linear search for an element matching a given value.
     *
     * @param aValue Value to which the array elements are compared.
     * @return Index of the array element matching aValue. If there is more than
     * one such elements with the same value the index of the first of these elements
     * is returned.  If no match is found, -1 is returned.
     */
    int findIndex(const T& aValue) const
    {
        const auto it = std::find(_storage.begin(), _storage.end(), aValue);
        return it != _storage.end() ? static_cast<int>(std::distance(_storage.begin(), it)) : -1;
    }

    /**
     * Linear search in reverse for an element matching a given value.
     *
     * @param aValue Value to which the array elements are compared.
     * @return Index of the array element matching aValue. If there is more than
     * one such elements with the same value the index of the last of these elements
     * is returned.  If no match is found, -1 is returned.
     */
    int rfindIndex(const T& aValue) const
    {
        const auto it = std::find(_storage.rbegin(), _storage.rend(), aValue);
        if (it != _storage.rend()) {
            auto idx = std::distance(_storage.begin(), it.base()) - 1;
            return static_cast<int>(idx);
        }
        else {
            return -1;
        }
    }

    /**
     * Search for the largest value in the array that is less than or
     * equal to a specified value.  If there is more than one element with this
     * largest value, the index of the first of these elements can optionally be
     * found, but this can be up to twice as costly.
     *
     * This method assumes that the array element values monotonically
     * increase as the array index increases.  Note that monotonically
     * increase means never decrease, so it is permissible for elements to
     *
     * A binary search is performed (i.e., the array is repeatedly subdivided
     * into two bins one of which must contain the specified until the
     * appropriate element is identified), so the performance of this method
     * is approximately ln(n), where n is the size of the array.
     *
     * @param aValue Value to which the array elements are compared.
     * @param aFindFirst DEPRECATED: this is now ALWAYS `true` - regardless of
     * what you are calling it with. This makes the behavior predictable on all
     * platforms.
     *
     * OLD BEHAVIOR: If true, find the first element that satisfies the search.
     * OLD BEHAVIOR: If false, the index of any element that satisfies the
     * search can be returned. Which index will be returned depends on the
     * length of the array and is therefore somewhat arbitrary.
     * OLD BEHAVIOR: By default, this flag is false (now: it is always true)
     * @param aLo Lowest array index to consider in the search.
     * @param aHi Highest array index to consider in the search.
     * @return Index of the array element that has the largest value that is less
     * than or equal to aValue. If an error is encountered (e.g., the array
     * is empty), or if the array contains no element that is less than or
     * equal to aValue, -1 is returned.
     */
    int searchBinary(
        const T& aValue,
        bool aFindFirst = false,
        int aLo = -1,
        int aHi = -1) const
    {
        // compute search range
        auto begin = _storage.begin() + (aLo < 0 ? 0 : aLo);
        auto end   = _storage.begin() + ((aHi < 0 || aHi >= size()) ? size() : aHi);

        if (begin == end) {
            // empty range: the array contains no element
            return -1;
        }
        if (begin > end) {
            // edge-case: the caller provided out-of-order indices, silently fix it
            std::swap(begin, end);
        }

        // search for the first element that is greater than or equal to the value
        auto const it = std::lower_bound(begin, end, aValue);

        if (it == end) {
            // all elements are less-than `aValue`, return the largest (last) one
            return static_cast<int>(std::distance(_storage.begin(), end)) - 1;
        }
        else if (*it == aValue) {
            // `it` directly points to the first occurrence of `aValue`
            return static_cast<int>(std::distance(_storage.begin(), it));
        }
        else if (it != begin) {
            // `it` points to the first value that is larger than `aValue`, the
            // value immediately preceding `it` is the largest value smaller than
            // `aValue`
            return static_cast<int>(std::distance(_storage.begin(), it)) - 1;
        }
        else {
            // `it` points to the first value that is larger than `aValue`, but
            // there are no preceding values that are smaller than `aValue`
            return -1;
        }
    }

private:
    T _defaultValue;

    using storage = std::conditional_t<
        std::is_same<T, bool>::value,
        std::vector<detail::BoolLike>,
        std::vector<T>
    >;

    storage _storage;
};

}; //namespace

#ifndef SWIG
// fmt library serializers for OpenSim Array objects
template <>
struct fmt::formatter<OpenSim::Array<double>> : ostream_formatter {};
#endif

#endif // OPENSIM_ARRAY_H_
