#ifndef _ArrayPtrs_h_
#define _ArrayPtrs_h_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  ArrayPtrs.h                            *
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


#include "osimCommonDLL.h"
#include <iostream>
#include "Exception.h"


//=============================================================================
//=============================================================================
/**
 * A class for storing an array of pointers to objects of type T.
 *
 * In contrast to class Array<T>, when an object is added to this array
 * a copy is not made.  Rather, a pointer to the added object is
 * stored in the array.
 *
 * When an ArrayPtrs object falls out of scope or is deleted, all objects
 * pointed to by the pointers in the array are deleted unless the array
 * is set not to own the memory associated with the objects to which its
 * array points.
 *
 * The capacity of the class grows as needed.  To use this template for a
 * class of type T, class T should implement the following methods:
 * default constructor, copy constructor, T* clone(),
 * assignment operator (=), equality operator (==), less than
 * operator (<), and the output operator (<<).
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

template<class T> class ArrayPtrs
{
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// DATA
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
protected:
    /** Flag indicating whether this ArrayPtrs object is responsible for
    (owns) the memory associated with the pointers in its array and therefore
    should issue deletes for the pointers upon destruction.  By default,
    _memoryOwner = true. */
    bool _memoryOwner;
    /** Size of the array.  Also the index of the first empty array element. */
    int _size;
    /** Current capacity of the array. */
    int _capacity;
    /** Increment by which the current capacity is increased when the capacity
    of this array is reached.  If negative, capacity doubles. */
    int _capacityIncrement;
    /** Array of pointers to objects of type T. */
    T **_array;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// METHODS
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//=============================================================================
// CONSTRUCTION
//=============================================================================
public:

// This typedef is used to avoid "duplicate const" errors with SWIG.
typedef typename std::add_const<T>::type ConstT;

//_____________________________________________________________________________
/**
 * Destructor.
 *
 * When the array is deleted, if this array is the memory owner, pointers
 * held by this array are also deleted.
 *
 * @see setMemoryOwner()
 */
virtual ~ArrayPtrs()
{
    if(_memoryOwner) clearAndDestroy();

    // ARRAY
    delete[] _array;
    _array = NULL;
}
//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aCapacity Initial capacity of the array.  The capacity
 * must be 1 or greater.
 */
explicit ArrayPtrs(int aCapacity=1)
{
    setNull();

    // CAPACITY
    if(aCapacity<1)  aCapacity = 1;
    ensureCapacity(aCapacity);

}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aArray Array to be copied.
 */
ArrayPtrs(const ArrayPtrs<T> &aArray)
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
    _memoryOwner = true;
    _size = 0;
    _capacityIncrement = -1;
    _capacity = 0;
    _array = NULL;
}

public:
//_____________________________________________________________________________
/**
 * Destroy all objects pointed to by this array and set the size of the
 * array to zero.  When this method is called, the objects pointed to by
 * this array are destroyed (deleted) even if this array is not set as
 * the memory owner.
 *
 * @see setMemoryOwner()
 */
void clearAndDestroy()
{
    if(_array==NULL) return;
    
    int i;
    for(i=0;i<_size;i++) {
        delete _array[i];  _array[i]=NULL;
    }

    _size = 0;
}


//=============================================================================
// OPERATORS
//=============================================================================
public:
//-----------------------------------------------------------------------------
// ASSIGNMENT (=)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this array to a specified array.
 * This operator makes a complete copy of the specified array; all member
 * variables and objects in the array are copied.  Because all objects are
 * copied, this object takes ownership of the newly allocated objects (i.e.,
 * _memoryOwner is set to true. So, the result is two independent,
 * identical arrays, with the possible exception of the _memoryOwner flag.
 *
 * @param aArray Array to be copied.
 * @return Reference to this array.
 */
#ifndef SWIG
ArrayPtrs<T>& operator=(const ArrayPtrs<T> &aArray)
{
    // DELETE OLD ARRAY
    if(_memoryOwner) clearAndDestroy();

    // COPY MEMBER VARIABLES
    _size = aArray._size;
    _capacity = aArray._capacity;
    _capacityIncrement = aArray._capacityIncrement;

    // ARRAY
    int i;
    if(_array!=NULL) delete[] _array;
    _array = new T*[_capacity];
    for(i=0;i<_size;i++) {
        if(aArray._array[i]!=NULL)  _array[i] = aArray._array[i]->clone();
    }

    // TAKE OWNERSHIP OF MEMORY
    _memoryOwner = true;

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
bool operator==(const ArrayPtrs<T> &aArray) const
{
    if(_size != aArray._size) return(false);

    int i;
    for(i=0;i<_size;i++) {
        if( !(_array[i]==aArray._array[i]) ) return(false);
    }

    return(true);
}

//-----------------------------------------------------------------------------
// BRACKETS ([])
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a pointer to the array object at a specified index.
 *
 * This operator is intended for accessing array elements with as little
 * overhead as possible, so no error checking is performed.
 * The caller must make sure the specified index is within the bounds of
 * the array.  If error checking is desired, use ArrayPtrs::get().
 *
 * @param aIndex Index of the desired element (0 <= aIndex < _size).
 * @return Reference to the array element.
 * @throws Exception if a NULL pointer is encountered.
 * @see get().
 */
T* operator[](int aIndex) const
{
    return( _array[aIndex] );
}
//_____________________________________________________________________________
//-----------------------------------------------------------------------------
// OUTPUT (<<)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Implementation of the output operator.
 * The output for an array looks like the following:\n\n
 *
 * ArrayPtrs[size] = T[0] T[1] T[2] ... T[size-1].
 *
 * @param aOut Output stream.
 * @param aArray Array to be output.
 * @return Reference to the output stream.
 */
friend std::ostream& operator<<(std::ostream &aOut,const ArrayPtrs<T> &aArray)
{
    aOut << "ArrayPtrs[" << aArray.getSize() <<"] =";

    int i;
    T* obj;
    for(i=0;i<aArray.getSize();i++)  {
        aOut << " ";
        obj = aArray[i];
        if(obj==NULL) {
            aOut << "NULL";
        } else {
            // The following line is having trouble compiling on IRIX.
            //aOut << *obj;
        }
    }

    return(aOut);
}


//=============================================================================
// MEMORY OWNERSHIP
//=============================================================================
//_____________________________________________________________________________
/**
 * %Set whether or not this array owns the memory pointed to by the pointers
 * in its array.
 *
 * @param aTrueFalse If true, all the memory associated with each of the
 * pointers in this array are deleted upon the array's destruction.  If
 * false, deletes are not issued for each of the pointers.
 */
void setMemoryOwner(bool aTrueFalse)
{
    _memoryOwner = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not this array owns the memory pointed to by the pointers
 * in its array.
 *
 * If the array is set to own the memory pointed to by its pointers, this
 * array issues deletes for all these pointers upon the array's destruction.
 * If not, this array does not issue deletes.
 *
 * @return True if this array owns the memory; false otherwise.
 */
bool getMemoryOwner() const
{
    return(_memoryOwner);
}


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
 * current capacity increments by this amount until the computed capacity is
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
    if(rNewCapacity < 1) rNewCapacity = 1;

    // CHECK FOR ZERO INCREMENT
    if(_capacityIncrement == 0) {
        std::cout << "ArrayPtrs.computeNewCapacity: WARN- capacity is set";
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
 * The newly allocated array elements are initialized to NULL.
 *
 * @param aCapacity Desired capacity.
 * @return true if the capacity was successfully obtained, false otherwise.
 */
bool ensureCapacity(int aCapacity)
{
    // CHECK REQUESTED CAPACITY
    if(aCapacity < 1) aCapacity = 1;
    if(_capacity>=aCapacity) return(true);

    // ALLOCATE THE NEW ARRAY
    int i;
    T **newArray = new T*[aCapacity];
    if(newArray==NULL) {
        std::cout << "ArrayPtrs.ensureCapacity: ERR- failed to increase capacity.\n";
        return(false);
    }

    // COPY CURRENT ARRAY
    if(_array!=NULL) {
        for(i=0;i<_size;i++) newArray[i] = _array[i];
        for(i=_size;i<aCapacity;i++) newArray[i] = NULL;
        delete[] _array;
    } else {
        for(i=0;i<aCapacity;i++) newArray[i] = NULL;
    }
    
    // REASSIGN
    _capacity = aCapacity;
    _array = newArray;

    return(true);
}

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
    if(newCapacity<1) newCapacity = 1;

    // TEMPORARY ARRAY
    T **array = _array;

    // ALLOCATE NEW ARRAY
    _array = new T*[newCapacity];
    if(_array==NULL) {
        std::cout << "ArrayPtrs.trim: ERR- unable to allocate array.\n";
        return;
    }

    // RESET PREVIOUS VALUES
    int i;
    for(i=0;i<_size;i++) _array[i] = array[i];
    _array[_size] = NULL;

    // SET CORRECT CAPACITY
    _capacity = newCapacity;

    // DELETE OLD ARRAY
    delete[] array;
}
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
 * If the specified increment is negative or this method
 * is called with no argument, the capacity is set to double whenever
 * the capacity is exceeded.
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
 * %Set the size of the array.  This method can be used only to decrease
 * the size of the array.  If the size of an array is decreased, all objects
 * in the array that become invalid as a result of the decrease are
 * deleted.
 *
 * Note that the size of an array is different than its capacity.  The size
 * indicates how many valid elements are stored in an array.  The capacity
 * indicates how much the size of the array can be increased without
 * allocated more memory.  At all times size <= capacity.
 *
 * @param aSize Desired size of the array.  The size must be greater than
 * or equal to zero and less than or equal to the current size of the
 * array.
 * @return True if the requested size change was carried out, false
 * otherwise.
 */
bool setSize(int aSize)
{
    if(aSize==_size) return(true);
    if(aSize>_size) return(false);
    if(aSize<0) aSize = 0;
    if(aSize<_size) {
        int i;
        for(i=(_size-1);i>=aSize;i--) {
            if(_array[i]!=NULL) {
                if(getMemoryOwner()) { delete _array[i]; }
                _array[i] = NULL;
            }
        }
        _size = aSize;
    }

    return(true);
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
// INDEX
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the index of an object by specifying its name.
 *
 * @param aObject Address of the object whose index is sought.
 * @param aStartIndex Index at which to start searching.  If the object is
 * not found at or following aStartIndex, the array is searched from
 * its beginning.
 * @return Index of the object with the address aObject.  If no such object
 * exists in the array, -1 is returned.
 */
int getIndex(ConstT *aObject,int aStartIndex=0) const
{
    if(aStartIndex<0) aStartIndex=0;
    if(aStartIndex>=getSize()) aStartIndex=0;

    // SEARCH STARTING FROM aStartIndex
    int i;
    for(i=aStartIndex;i<getSize();i++) {
        if(_array[i] == aObject) return(i);
    }

    // SEARCH FROM BEGINNING
    for(i=0;i<aStartIndex;i++) {
        if(_array[i] == aObject) return(i);
    }

    return(-1);
}
//_____________________________________________________________________________
/**
 * Get the index of an object by specifying its name.
 *
 * @param aName Name of the object whose index is sought.
 * @param aStartIndex Index at which to start searching.  If the object is
 * not found at or following aStartIndex, the array is searched from
 * its beginning.
 * @return Index of the object named aName.  If no such object exists in
 * the array, -1 is returned.
 */
int getIndex(const std::string &aName,int aStartIndex=0) const
{
    if(aStartIndex<0) aStartIndex=0;
    if(aStartIndex>=getSize()) aStartIndex=0;

    // SEARCH STARTING FROM aStartIndex
    int i;
    for(i=aStartIndex;i<getSize();i++) {
        if(_array[i]->getName() == aName) return(i);
    }

    // SEARCH FROM BEGINNING
    for(i=0;i<aStartIndex;i++) {
        if(_array[i]->getName() == aName) return(i);
    }

    return(-1);
}

//-----------------------------------------------------------------------------
// APPEND
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Append to the array.  A copy of the specified object is NOT made.
 *
 * @param aObject Object to be appended.
 * @return True if the append was successful, false otherwise.
 */
bool append(T *aObject)
{
    if(aObject==NULL) {
        std::cout<<"ArrayPtrs.append: ERR- NULL pointer."<<std::endl;
        return(false);
    }

    // ENSURE CAPACITY
    if((_size+1)>=_capacity) {
        int newCapacity;
        bool success;
        success = computeNewCapacity(_size+1,newCapacity);
        if(!success) return(success);
        success = ensureCapacity(newCapacity);
        if(!success) return(success);
    }

    // SET
    _array[_size] = aObject;
    _size++;

    return(true);
}
//_____________________________________________________________________________
/**
 * Append an array of objects.  Copies of the objects are NOT made
 *
 * @param aArray Array of objects to be appended.
 * @return True if the append was successful, false otherwise.
 */
bool append(ArrayPtrs<T> &aArray)
{
    // LOOP THROUGH THE ELEMENTS
    bool success;
    int i,n=aArray.getSize();
    for(i=0;i<n;i++) {
        success = append(aArray[i]);
        if(!success) return(success);
    }

    return(true);
}

//-----------------------------------------------------------------------------
// INSERT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Insert an object into the array at a specified index.  A copy of the
 * specified object is NOT made.
 *
 * This method is relatively computationally costly since many of the array
 * elements may need to be shifted.
 *
 * @param aObject Object to be inserted.
 * @param aIndex Index at which to insert the new object.  All current elements
 * from aIndex to the end of the array are shifted one place in the direction
 * of the end of the array.  The specified index must be less than or
 * equal to the size of the array.  Note that if aIndex is equal to the
 * size of the array, the insertion is equivalent to an append.
 * @return True if the insertion was successful, false otherwise.
 */
bool insert(int aIndex,T *aObject)
{
    // NULL POINTER
    if(aObject==NULL) {
        std::cout<<"ArrayPtrs.insert: ERR- NULL pointer."<<std::endl;
        return(false);
    }

    // NEGATIVE INDEX
    if(aIndex<0) {
        std::cout << "ArrayPtrs.insert: ERR- aIndex was less than 0.\n";
        return(false);
    }

    // INDEX PAST END OF ARRAY
    if(aIndex>_size) {
        return(false);
    }

    // ENSURE CAPACITY
    if((_size+1)>=_capacity) {
        int newCapacity;
        bool success;
        success = computeNewCapacity(_size+1,newCapacity);
        if(!success) return(success);
        success = ensureCapacity(newCapacity);
        if(!success) return(success);
    }

    // SHIFT ARRAY
    int i;
    for(i=_size;i>aIndex;i--) {
        _array[i] = _array[i-1];
    }

    // SET
    _array[aIndex] = aObject;
    _size++;

    return(true);
}

//-----------------------------------------------------------------------------
// REMOVE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Remove an object from the array at a specified index.
 * The object is deleted when it is removed.
 *
 * This method is relatively computationally costly since many of the array
 * elements may need to be shifted.
 *
 * @param aIndex Index of the value to remove.  All elements from aIndex to
 * the end of the array are shifted one place toward the beginning of
 * the array.  If aIndex is less than 0 or greater than or equal to the
 * current size of the array, no element is removed.
 * @return True if the removal was successful, false otherwise.
 */
bool remove(int aIndex)
{
    if(aIndex<0) {
        return(false);
    }
    if(aIndex>=_size) {
        return(false);
    }

    // DELETE CURRENT OBJECT
    if(getMemoryOwner()&&(_array[aIndex]!=NULL)) delete _array[aIndex];

    // SHIFT ARRAY
    int i;
    _size--;
    for(i=aIndex;i<_size;i++) {
        _array[i] = _array[i+1];
    }
    _array[_size] = NULL;

    return(true);
}
//_____________________________________________________________________________
/**
 * Remove an object from the array by specifying its address.
 * If this array is set as the memory owner, the object is deleted when it
 * is removed.
 *
 * This method is relatively computationally costly since many of the array
 * elements may need to be shifted.
 *
 * @param aObject Pointer to the object to be removed.  If an object with the
 * specified address is not found, no action is taken.
 * @return True if the removal was successful, false otherwise.
 */
bool remove(ConstT* aObject)
{
    int index = getIndex(aObject);
    return( remove(index) );
}


//-----------------------------------------------------------------------------
// SET AND GET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * %Set the object at a specified index.  A copy of the object is NOT made.
 *
 * If the set method is successful and the array is set as the memory
 * owner, the previous object stored at the specified index is deleted.
 *
 * @param aIndex Index of the array element to be set.  aIndex must be
 * greater than zero and less than or equal to the size of the array.  Note
 * that if aIndex is equal to the size of the array, the set is equivalent
 * to an append.
 * @param aObject Object to be set.
 * @return True if the set was successful, false otherwise.
 * @see setMemoryOwner()
 */
bool set(int aIndex,T *aObject)
{
    if(aIndex<0) return(false);
    if(aIndex>_size) return(false);

    // APPEND
    if(aIndex==_size) {
        bool success;
        success = append(aObject);
        return(success);
    }

    // SET
    if(getMemoryOwner() && (_array[aIndex]!=NULL)) delete _array[aIndex];
    _array[aIndex] = aObject;

    return(true);
}
//_____________________________________________________________________________
/**
 * Get the object at a specified array index.
 *
 * If the index is negative or passed the end of the array, an exception
 * is thrown.
 *
 * For faster execution, the array elements can be accessed through the
 * overloaded operator[], which does no bounds checking.
 *
 * @param aIndex Array index of the desired object.
 * @return Pointer to the desired object.
 * @throws Exception if (aIndex<0)||(aIndex>=_size) or if the pointer
 * at aIndex is NULL.
 * @see operator[].
 */
T* get(int aIndex)
{
    if((aIndex<0)||(aIndex>=_size)) {
        throw(Exception("ArrayPtrs.get: Array index out of bounds."));
    }
    if(_array[aIndex]==NULL) {
        throw(Exception("ArrayPtrs.get: NULL pointer.",
            __FILE__,__LINE__));
    }

    return(_array[aIndex]);
}
//_____________________________________________________________________________
/**
 * Get the object at a specified array index.
 *
 * If the index is negative or passed the end of the array, an exception
 * is thrown.
 *
 * For faster execution, the array elements can be accessed through the
 * overloaded operator[], which does no bounds checking.
 *
 * @param aIndex Array index of the desired object.
 * @return Pointer to the desired object.
 * @throws Exception if (aIndex<0)||(aIndex>=_size) or if the pointer
 * at aIndex is NULL.
 * @see operator[].
 */
#ifndef SWIG
const T* get(int aIndex) const
{
    if((aIndex<0)||(aIndex>=_size)) {
        throw(Exception("ArrayPtrs.get: Array index out of bounds."));
    }
    if(_array[aIndex]==NULL) {
        throw(Exception("ArrayPtrs.get: NULL pointer.",
            __FILE__,__LINE__));
    }

    return(_array[aIndex]);
}
#endif
//_____________________________________________________________________________
/**
 * Get the first object that has a specified name.
 *
 * If the array doesn't contain an object of the specified name, an
 * exception is thrown.
 *
 * @param aName Name of the desired object.
 * @return Pointer to the object.
 * @throws Exception if no such object exists.
 * @see getIndex()
 */
T* get(const std::string &aName)
{
    int index = getIndex(aName);
    if(index==-1) {
        std::string msg = "ArrayPtrs.get(aName): No object with name ";
        msg += aName;
        throw( Exception(msg,__FILE__,__LINE__) );
    }
    return(_array[index]);
}
//_____________________________________________________________________________
/**
 * Get the first object that has a specified name.
 *
 * If the array doesn't contain an object of the specified name, an
 * exception is thrown.
 *
 * @param aName Name of the desired object.
 * @return Pointer to the object.
 * @throws Exception if no such object exists.
 * @see getIndex()
 */
#ifndef SWIG
const T* get(const std::string &aName) const
{
    int index = getIndex(aName);
    if(index==-1) {
        std::string msg = "ArrayPtrs.get(aName): No object with name ";
        msg += aName;
        throw( Exception(msg,__FILE__,__LINE__) );
    }
    return(_array[index]);
}
#endif
//_____________________________________________________________________________
/**
 * Get the last value in the array.
 *
 * @return Last value in the array.
 * @throws Exception if the array is empty.
 */
T* getLast() const
{
    if(_size<=0) {
        throw(Exception("Array is empty."));
    }
    return(_array[_size-1]);
}


//=============================================================================
// SEARCH
//=============================================================================
//_____________________________________________________________________________
/**
 * Search for the largest value in the array that is less than or
 * equal to a specified value.  If there is more than one element with this
 * largest value, the index of the first of these elements can optionally be
 * found, but this can be up to twice as costly.
 *
 * This method assumes that the array element values monotonically
 * increase as the array index increases.  Note that monotonically
 * increase means never decrease, so it is permissible for adjacent elements
 * to have the same value.
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
int searchBinary(ConstT& aObject,bool aFindFirst=false,
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
        if(aObject < *_array[mid]) {
            hi = mid - 1;
        } else if(*_array[mid] < aObject) {
            lo = mid + 1;
        } else {
            break;
        }
    }

    // MAKE SURE LESS THAN
    if(aObject < *_array[mid]) mid--;
    if(mid<=0) {
        return(mid);
    }

    // FIND FIRST
    if(aFindFirst) {
        if(*_array[mid-1]<*_array[mid]) {
            return(mid);
        }
        lo = aLo;  if(lo<0) lo = 0;
        hi = mid;
        int mid2 = mid;
        T *obj2 = _array[mid];
        while(lo <= hi) {
            mid2 = (lo + hi) / 2;
            if(*_array[mid2] == *obj2) {
                hi = mid2 - 1;
            } else if(*_array[mid2] < *obj2) {
                lo = mid2 + 1;
            }
        }
        if(*_array[mid2] < *obj2) mid2++;
        if(mid2<mid) mid = mid2;
    }

    return(mid);
}

//=============================================================================
};  // END of class ArrayPtrs

}; //namespace
//=============================================================================
//=============================================================================


#endif //__ArrayPtrs_h__
