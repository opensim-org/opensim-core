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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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


#include <iostream>
#include "osimCommonDLL.h"
#include "Exception.h"

static const int Array_CAPMIN = 1;

//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * A class for storing an array of values of type T.  The capacity of the class
 * grows as needed.  To use this template for a class of type T, class T should
 * implement the following methods:  default constructor, copy constructor,
 * assignment operator (=), equality operator (==), and less than
 * operator (<).
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
template<class T> class Array
{

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// DATA
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
protected:
    /** Size of the array.  Also the index of the first empty array element. */
    int _size;
    /** Current capacity of the array. */
    int _capacity;
    /** Increment by which the current capacity is increased when the capacity
    of this storage instance is reached.  If negative, capacity doubles. */
    int _capacityIncrement;
    /** Default value of elements. */
    T _defaultValue;
    /** Array of values. */
    T *_array;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// METHODS
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//=============================================================================
// CONSTRUCTION
//=============================================================================
public:
//_____________________________________________________________________________
/**
 * Destructor.
 *
 * When the array is deleted, references to elements of this array become
 * invalid.
 */
virtual ~Array()
{
    if(_array!=NULL) { delete[] _array;  _array = NULL; }
}
//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aDefaultValue Default value of an array element.  This value
 * is used to initialize array elements as the size of the array is
 * changed.
 * @param aSize Initial size of the array.  The array elements are
 * initialized to aDefaultValue.
 * @param aCapacity Initial capacity of the array.  The initial capacity
 * is guaranteed to be at least as large as aSize + 1.
 */
Array(const T &aDefaultValue=T(),int aSize=0,int aCapacity=Array_CAPMIN)
{
    setNull();

    // DEFAULT VALUE
    _defaultValue = aDefaultValue;

    // CAPACITY
    int newCapacity;
    int min = aSize + 1;
    if(min < aCapacity) min = aCapacity;
    computeNewCapacity(min,newCapacity);
    ensureCapacity(newCapacity);

    // SIZE
    _size = aSize;
    if(_size<0) _size=0;
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aArray Array to be copied.
 */
Array(const Array<T> &aArray)
{
    setNull();
    *this = aArray;
}

private:
//_____________________________________________________________________________
/**
 * %Set all member variables to their null or default values.
 */
void setNull()
{
    _size = 0;
    _capacityIncrement = -1;
    _capacity = 0;
    _array = NULL;
}


//=============================================================================
// OPERATORS
//=============================================================================
public:
// A non-operator version of operator== which we can use in Java
// NOTE: I tried to name it "equals" since that's the standard way to compare objects in
// Java, but didn't seem to work...  - Eran
bool arrayEquals(const Array<T> &aArray) const
{
    return *this == aArray;
}
#ifndef SWIG
//-----------------------------------------------------------------------------
// BRACKETS ([])
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
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
    return(_array[aIndex]);
}

//-----------------------------------------------------------------------------
// ASSIGNMENT (=)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this array to a specified array.
 * This operator makes a complete copy of the specified array; all member
 * variables are copied.  So, the result is two identical, independent arrays.
 *
 * @param aArray Array to be copied.
 * @return Reference to this array.
 */
Array<T>& operator=(const Array<T> &aArray)
{
    _size = aArray._size;
    _capacity = aArray._capacity;
    _capacityIncrement = aArray._capacityIncrement;
    _defaultValue = aArray._defaultValue;

    // ARRAY
    if(_array!=NULL) delete[] _array;
    _array = new T[_capacity];
    for(int i=0;i<_capacity;i++) _array[i] = aArray._array[i];

    return(*this);
}

//-----------------------------------------------------------------------------
// EQUALITY (==)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine if two arrays are equal.
 *
 * Two arrays are equal if their contents are equal.  That is, each array
 * must be the same length and their corresponding array elements must be
 * equal.
 *
 * @param aArray Array to be tested as equal.
 * @return True if equal, false if not equal.
 */
bool operator==(const Array<T> &aArray) const
{
    if(_size != aArray._size) return(false);

    int i;
    for(i=0;i<_size;i++) {
        if( !(_array[i]==aArray._array[i]) ) return(false);
    }

    return(true);
}

//-----------------------------------------------------------------------------
// OUTPUT (<<)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Implementation of the output operator.
 * The output for an array looks like the following:\n\n
 *
 * T[0] T[1] T[2] ... T[size-1].
 *
 * @param aOut Output stream.
 * @param aArray Array to be output.
 * @return Reference to the output stream.
 */
friend std::ostream& operator<<(std::ostream &aOut,const Array<T> &aArray)
{
    //removed since this is used for serialization now aOut << "Array["<<aArray.getSize()<<"]=";

    int i;
    for(i=0;i<aArray.getSize();i++)  {
        aOut << " ";
        aOut << aArray[i];
    }

    return(aOut);
}

friend
std::istream& operator>>(std::istream& in, Array<T>& out) 
{   
    //return readArrayFromStream<T>(in, out); 
    return in;
}
/*
template <class T> inline std::istream& readArrayFromStream<T>(std::istream& in, Array<T>& out)
{
    SimTK::Array_<T> simtkArray;
    readArrayFromStream(in, simtkArray);
    out.setSize(simtkArray.getSize());
    for(int i=0; i<simtkArray.getSize(); i++)
        out[i] = simtkArray[i];
}*/

//=============================================================================
// CAPACITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute a new capacity that is at least as large as a specified minimum
 * capacity; this method does not change the capacity, it simply computes
 * a new recommended capacity.
 *
 * If the capacity increment is negative, the current capacity is
 * doubled until the computed capacity is greater than or equal to the
 * specified minimum capacity.  If the capacity increment is positive, the
 * current capacity increment by this amount until the computed capacity is
 * greater than or equal to the specified minimum capacity.  If the capacity
 * increment is zero, the computed capacity is set to the current capacity
 * and false is returned.
 *
 * @param rNewCapacity New computed capacity.
 * @param aMinCapacity Minimum new computed capacity.  The computed capacity
 * is incremented until it is at least as large as aMinCapacity, assuming
 * the capacity increment is not zero.
 * @return True if the new capacity was increased, false otherwise (i.e.,
 * if the capacity increment is set to 0).
 * @see setCapacityIncrement()
 */
bool computeNewCapacity(int aMinCapacity,int &rNewCapacity)
{
    rNewCapacity = _capacity;
    if(rNewCapacity < Array_CAPMIN) rNewCapacity = Array_CAPMIN;

    // CHECK FOR ZERO INCREMENT
    if(_capacityIncrement == 0) {
        std::cout << "Array.computeNewCapacity: WARN- capacity is set";
        std::cout << " not to increase (i.e., _capacityIncrement==0).\n";
        return(false);
    }

    // INCREMENT UNTIL LARGER THAN THE MINIMUM SIZE
    while(rNewCapacity < aMinCapacity) {
        if(_capacityIncrement < 0) {
            rNewCapacity = 2 * rNewCapacity;
        } else {
            rNewCapacity = rNewCapacity + _capacityIncrement;
        }
    }

    return(true);
}
//_____________________________________________________________________________
/**
 * Ensure that the capacity of this array is at least the specified amount.
 * Note that the newly allocated array elements are not initialized.
 *
 * @param aCapacity Desired capacity.
 * @return true if the capacity was successfully obtained, false otherwise.
 */
bool ensureCapacity(int aCapacity)
{
    // CHECK REQUESTED CAPACITY
    if(aCapacity < Array_CAPMIN) aCapacity = Array_CAPMIN;
    if(_capacity>=aCapacity) return(true);

    // ALLOCATE THE NEW ARRAY
    int i;
    T *newArray = new T[aCapacity];
    if(newArray==NULL) {
        std::cout << "Array.ensureCapacity: ERR- failed to increase capacity.\n";
        return(false);
    }

    // COPY CURRENT ARRAY
    if(_array!=NULL) {
        for(i=0;i<_size;i++) newArray[i] = _array[i];
        for(i=_size;i<aCapacity;i++) newArray[i] = _defaultValue;
        delete []_array;  _array=NULL;
    } else {
        for(i=0;i<aCapacity;i++) newArray[i] = _defaultValue;
    }
    
    // REASSIGN
    _capacity = aCapacity;
    _array = newArray;

    return(true);
}
#endif
//_____________________________________________________________________________
/**
 * Trim the capacity of this array so that it is one larger than the size
 * of this array.  This is useful for reducing the amount of memory used
 * by this array.  This capacity is kept at one larger than the size so
 * that, for example, an array of characters can be treated as a NULL
 * terminated string.
 */
void trim()
{
    // COMPUTE NEW CAPACITY
    int newCapacity = _size + 1;
    if(newCapacity>=_capacity) return;
    if(newCapacity<Array_CAPMIN) newCapacity = Array_CAPMIN;

    // ALLOCATE TEMPORARY ARRAY
    int i;
    T *array = new T[newCapacity];
    if(array==NULL) {
        std::cout << "Array.trim: ERR- unable to allocate temporary array.\n";
        return;
    }

    // COPY CURRENT ARRAY
    for(i=0;i<_size;i++) array[i] = _array[i];

    // DELETE OLD ARRAY
    delete[] _array;

    // REASSIGN ARRAY POINTER
    _array = array;

    // SET CORRECT CAPACITY
    _capacity = newCapacity;
}
#ifndef SWIG
//_____________________________________________________________________________
/**
 * Get the capacity of this storage instance.
 */
int getCapacity() const
{

    return(_capacity);
}

//-----------------------------------------------------------------------------
// CAPACITY INCREMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * %Set the amount by which the capacity is increased when the capacity of
 * of the array in exceeded.
 * If the specified increment is negative, the capacity is set to double
 * whenever the capacity is exceeded.
 *
 * @param aIncrement Desired capacity increment.
 */
void setCapacityIncrement(int aIncrement)
{
    _capacityIncrement = aIncrement;
}
//_____________________________________________________________________________
/**
 * Get the amount by which the capacity is increased.
 */
int getCapacityIncrement() const
{
    return(_capacityIncrement);
}
#endif
//=============================================================================
// STORAGE OPERATIONS
//=============================================================================
//-----------------------------------------------------------------------------
// SIZE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * %Set the size of the array.  This method can be used to either increase
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
    if(aSize==_size) return(true);
    if(aSize<0) aSize = 0;
    bool success = true;
    if(aSize<_size) {
        int i;
        for(i=(_size-1);i>=aSize;i--) _array[i] = _defaultValue;
        _size = aSize;
    } else if(aSize<=_capacity) {
        _size = aSize;
    } else {
        int newCapacity;
        success = computeNewCapacity(aSize+1,newCapacity);
        if(!success) return(false);
        success = ensureCapacity(newCapacity);
        if(success) _size = aSize;
    }

    return(success);
}
//_____________________________________________________________________________
/**
 * Get the size of the array.
 *
 * @return Size of the array.
 */
int getSize() const
{
    return(_size);
}

/** Alternate name for getSize(). **/
int size() const {return getSize();}

//-----------------------------------------------------------------------------
// APPEND
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Append a value onto the array.
 *
 * @param aValue Value to be appended.
 * @return New size of the array, or, equivalently, the index to the new
 * first empty element of the array.
 */
int append(const T &aValue)
{
    // ENSURE CAPACITY
    if((_size+1)>=_capacity) {
        int newCapacity;
        bool success;
        success = computeNewCapacity(_size+1,newCapacity);
        if(!success) return(_size);
        success = ensureCapacity(newCapacity);
        if(!success) return(_size);
    }

    // SET
    _array[_size] = aValue;
    _size++;

    return(_size);
}
//_____________________________________________________________________________
/**
 * Append an array of values.
 *
 * @param aArray Array of values to append.
 * @return New size of the array, or, equivalently, the index to the new
 * first empty element of the array.
 */
int append(const Array<T> &aArray)
{
    // LOOP THROUGH THE ELEMENTS
    int i,n=aArray.getSize();
    for(i=0;i<n;i++) {
        append(aArray[i]);
    }

    return(_size);
}
#ifndef SWIG
//_____________________________________________________________________________
/**
 * Append an array of values.
 *
 * @param aSize Size of the array to append.
 * @param aArray Array of values to append.
 * @return New size of the array, or, equivalently, the index to the new
 * first empty element of the array.
 */
int append(int aSize,const T *aArray)
{
    if(aSize<0) return(_size);
    if(aArray==NULL) return(_size);

    // LOOP THROUGH THE ELEMENTS
    int i;
    for(i=0;i<aSize;i++) {
        append(aArray[i]);
    }

    return(_size);
}
#endif
//-----------------------------------------------------------------------------
// INSERT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
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
int insert(int aIndex,const T &aValue)
{
    // NEGATIVE INDEX
    if(aIndex<0) {
        std::cout << "Array.insert: ERR- aIndex was less than 0.\n";
        return(_size);
    }

    // INDEX PAST END OF ARRAY
    if(aIndex>=_size) {
        setSize(aIndex+1);
        _array[aIndex] = aValue;
        return(_size);
    }

    // ENSURE CAPACITY
    if((_size+1)>=_capacity) {
        int newCapacity;
        bool success;
        success = computeNewCapacity(_size+1,newCapacity);
        if(!success) return(_size);
        success = ensureCapacity(newCapacity);
        if(!success) return(_size);
    }

    // SHIFT ARRAY
    int i;
    for(i=_size;i>aIndex;i--) {
        _array[i] = _array[i-1];
    }

    // SET
    _array[aIndex] = aValue;
    _size++;

    return(_size);
}

//-----------------------------------------------------------------------------
// REMOVE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
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
    if(aIndex<0) {
        std::cout << "Array.remove: ERR- aIndex was less than 0.\n";
        return(_size);
    }
    if(aIndex>=_size) {
        std::cout << "Array.remove: ERR- aIndex was greater than or equal the ";
        std::cout << "size of the array.\n";
        return(_size);
    }

    // SHIFT ARRAY
    int i;
    _size--;
    for(i=aIndex;i<_size;i++) {
        _array[i] = _array[i+1];
    }
    _array[_size] = _defaultValue;

    return(_size);
}

//-----------------------------------------------------------------------------
// SET AND GET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * %Set the value at a specified index.
 *
 * @param aIndex Index of the array element to be set.  It is permissible
 * for aIndex to be past the current end of the array- the capacity will
 * be increased if necessary.  Values between the current end of the array
 * and aIndex are not initialized.
 * @param aValue Value.
 */
void set(int aIndex,const T &aValue)
{
    if(aIndex<0) return;

    // ENSURE CAPACITY
    bool success = false;
    if((aIndex+2)>=_capacity) {
        int newCapacity;
        success = computeNewCapacity(aIndex+2,newCapacity);
        if(!success) return;
        success = ensureCapacity(newCapacity);
        if(!success) return;
    }

    // SET
    _array[aIndex] = aValue;

    // FIRST EMPTY
    if(aIndex>=_size)  _size = aIndex+1;
}
#ifndef SWIG
//_____________________________________________________________________________
/**
 * Get a pointer to the low-level array.
 *
 * @return Pointer to the low-level array.
 */
T* get()
{
    return(_array);
}
//_____________________________________________________________________________
/**
 * Get a pointer to the low-level array.
 *
 * @return Pointer to the low-level array.
 */

const T* get() const
{
    return(_array);
}
#endif
//_____________________________________________________________________________
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
    if((aIndex<0)||(aIndex>=_size)) {
        throw(Exception("Array index out of bounds."));
    }
    return(_array[aIndex]);
}

#ifndef SWIG
//_____________________________________________________________________________
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
    if((aIndex<0)||(aIndex>=_size)) {
        throw(Exception("Array index out of bounds."));
    }
    return(_array[aIndex]);
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
//_____________________________________________________________________________
/**
 * Get the last value in the array.
 *
 * @return Last value in the array.
 * @throws Exception if the array is empty.
 */
const T& getLast() const
{
    if(_size<=0) {
        throw(Exception("Array is empty."));
    }
    return(_array[_size-1]);
}
#ifndef SWIG
//_____________________________________________________________________________
/**
 * Get writable reference to last value in the array.
 *
 * @return writable reference to Last value in the array.
 * @throws Exception if the array is empty.
 */
T& updLast() const
{
    if(_size<=0) {
        throw(Exception("Array is empty."));
    }
    return(_array[_size-1]);
}
#endif
//=============================================================================
// SEARCH
//=============================================================================
//_____________________________________________________________________________
/**
 * Linear search for an element matching a given value.
 *
 * @param aValue Value to which the array elements are compared.
 * @return Index of the array element matching aValue. If there is more than
 * one such elements with the same value the index of the first of these elements
 * is returned.  If no match is found, -1 is returned.
 */
int findIndex(const T &aValue) const
{
    for(int i=0;i<_size;i++) if(_array[i]==aValue) return i;
    return -1;
}
//_____________________________________________________________________________
/**
 * Linear search in reverse for an element matching a given value.
 *
 * @param aValue Value to which the array elements are compared.
 * @return Index of the array element matching aValue. If there is more than
 * one such elements with the same value the index of the last of these elements
 * is returned.  If no match is found, -1 is returned.
 */
int rfindIndex(const T &aValue) const
{
    for(int i=_size-1;i>=0;i--) if(_array[i]==aValue) return i;
    return -1;
}
//_____________________________________________________________________________
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
 * @param aFindFirst If true, find the first element that satisfies
 * the search.  If false, the index of any element that satisfies the
 * search can be returned- which index will be returned depends on the
 * length of the array and is therefore somewhat arbitrary. By default,
 * this flag is false.
 * @param aLo Lowest array index to consider in the search.
 * @param aHi Highest array index to consider in the search.
 * @return Index of the array element that has the largest value that is less
 * than or equal to aValue.  If there is more than one such elements with the
 * same value and aFindFirst is set to true, the index of the first of
 * these elements is returned.  If an error is encountered (e.g., the array
 * is empty), or the array contains no element that is less than or equal
 * to aValue, -1 is returned.
 */
int searchBinary(const T &aValue,bool aFindFirst=false,
                      int aLo=-1,int aHi=-1) const
{
    if(_size<=0) return(-1);
    int lo = aLo;  if(lo<0) lo = 0;
    int hi = aHi;  if((hi<0)||(hi>=_size)) hi = _size - 1;
    int mid = -1;

    // CHECK lo AND hi
    if(lo>hi) return(-1);

    // SEARCH
    while(lo <= hi) {
        mid = (lo + hi) / 2;
        if(aValue < _array[mid]) {
            hi = mid - 1;
        } else if(_array[mid] < aValue) {
            lo = mid + 1;
        } else {
            break;
        }
    }

    // MAKE SURE LESS THAN
    if(aValue < _array[mid]) mid--;
    if(mid<=0) {
        return(mid);
    }

    // FIND FIRST
    if(aFindFirst) {
        if(_array[mid-1]<_array[mid]) {
            return(mid);
        }
        lo = aLo;  if(lo<0) lo = 0;
        hi = mid;
        int mid2 = mid;
        T value2 = _array[mid];
        while(lo <= hi) {
            mid2 = (lo + hi) / 2;
            if(_array[mid2] == value2) {
                hi = mid2 - 1;
            } else if(_array[mid2] < value2) {
                lo = mid2 + 1;
            }
        }
        if(_array[mid2]<value2) mid2++;
        if(mid2<mid) mid = mid2;
    }

    return(mid);
}


//=============================================================================
};  // END of class Array

}; //namespace
//=============================================================================
//=============================================================================







#endif // OPENSIM_ARRAY_H_
