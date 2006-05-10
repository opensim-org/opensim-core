#ifndef _Set_h_
#define _Set_h_
// Set.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */




// INCLUDES
#include "rdTools.h"
#include "Object.h"
#include "ArrayPtrs.h"
#include "PropertyObjArray.h"



//=============================================================================
//=============================================================================
/**
 * A class for holding a set of pointers to objects.  It is derived from
 * class Object and is implemented as a wrapper around template
 * ArrayPtrs<T>.  It is implemented around an array of pointers, rather
 * than values, so that it can make use of any virtual methods associated
 * with class T.
 *
 * @see ArrayPtrs
 * @author Frank C. Anderson
 */
namespace OpenSim { 

template<class T> class Set : public Object
{
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// DATA
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
protected:
// PROPERTIES
/** Array of pointers to objects. */
PropertyObjArray _propObjects;

// REFERENCES
ArrayPtrs<T> &_objects;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// METHODS
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public:
//=============================================================================
// DESTRUCTOR & CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
virtual ~Set()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Set() :
	Object(),
	_objects((ArrayPtrs<T>&)_propObjects.getValueObjArray())
{
	setType("Set");
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct from file.
 *
 * @param aFileName Name of the file.
 */
Set(const std::string &aFileName) :
	Object(aFileName),
	_objects((ArrayPtrs<T>&)_propObjects.getValueObjArray())
{
	setType("Set");
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Construct from a DOM element.
 *
 * @param aElement DOM element.
 */
Set(DOMElement *aElement) :
	Object(aElement),
	_objects((ArrayPtrs<T>&)_propObjects.getValueObjArray())
{
	setType("Set");
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSet Set to be copied.
 */
Set(const Set<T> &aSet) :
	Object(aSet),
	_objects((ArrayPtrs<T>&)_propObjects.getValueObjArray())
{
	setNull();
	_objects = aSet._objects;
}
//_____________________________________________________________________________
/**
 * Copy.
 */
Object*
copy() const
{
	Set<T> *retObj = new Set<T>();
	*retObj = *this;

	return(retObj);
}


private:
//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void
setNull()
{
	setupProperties();
	_objects.setSize(0);
}
//_____________________________________________________________________________
/**
 * Setup serialized member variables.
 */
void
setupProperties()
{
	_propObjects.setName("objects");
	_propertySet.append(	&_propObjects );
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
 * Assign this set to another set.
 * This operator makes a complete copy of the specified set; all member
 * variables and objects in the set are copied.  Because all objects are
 * copied, this set takes ownership of the newly allocated objects (i.e.,
 * _memoryOwner is set to true. So, the result is two independent,
 * identical sets, with the possible exception of the _memoryOwner flag.
 *
 * @param aSet Set to be copied.
 * @return Reference to this set.
 */
#ifndef SWIG
Set<T>& operator=(const Set<T> &aSet)
{	
	Object::operator=(aSet);
	_objects = aSet._objects;

	return(*this);
}

//-----------------------------------------------------------------------------
// BRACKETS ([])
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a pointer to the set object at a specified index.
 *
 * This operator is intended for accessing set objects with as little
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
	return( _objects[aIndex] );
}

//-----------------------------------------------------------------------------
// OUTPUT (<<)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Implementation of the output operator.
 * The output for a set looks like the following:\n\n
 *
 * Set[size] = T[0] T[1] T[2] ... T[size-1].
 *
 * @param aOut Output stream.
 * @param aSet Set to be output.
 * @return Reference to the output stream.
 */
friend std::ostream& operator<<(std::ostream &aOut,const Set<T> &aSet)
{
	aOut << "Set[" << aSet.getSize() <<"] =";

	int i;
	T* obj;
	for(i=0;i<aSet.getSize();i++)  {
		aOut << " ";
		obj = aSet[i];
		if(obj==NULL) {
			aOut << "NULL";
		} else {
			aOut << *obj;
		}
	}

	return(aOut);
}
#endif // SWIG


//=============================================================================
// MEMORY OWNERSHIP
//=============================================================================
//_____________________________________________________________________________
/**
 * Set whether or not this set owns the memory pointed to by the pointers
 * it holds.
 *
 * @param aTrueFalse If true, all the memory associated with each of the
 * pointers in this array are deleted upon the array's destruction.  If
 * false, deletes are not issued for each of the pointers.
 */
void setMemoryOwner(bool aTrueFalse)
{
	_objects.setMemoryOwner(aTrueFalse);
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
	return(_objects.getMemoryOwner());
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
	return( _objects.computeNewCapacity(aMinCapacity,rNewCapacity) );
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
	return( _objects.ensureCapacity(aCapacity) );
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
	_objects.trim();
}
//_____________________________________________________________________________
/**
 * Get the capacity of this storage instance.
 */
int getCapacity() const
{
	return( _objects.getCapacity() );
}

//-----------------------------------------------------------------------------
// CAPACITY INCREMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the amount by which the capacity is increased when the capacity of
 * of the array in exceeded.
 * If the specified increment is negative or this method
 * is called with no argument, the capacity is set to double whenever
 * the capacity is exceeded.
 *
 * @param aIncrement Desired capacity increment.
 */
void setCapacityIncrement(int aIncrement)
{
	_objects.setCapacityIncrement(aIncrement);
}
//_____________________________________________________________________________
/**
 * Get the amount by which the capacity is increased.
 */
int getCapacityIncrement() const
{
	return( _objects.getCapacityIncrement() );
}

//=============================================================================
// STORAGE OPERATIONS
//=============================================================================
//-----------------------------------------------------------------------------
// SIZE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the size of the array.  This method can be used only to decrease
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
virtual bool setSize(int aSize)
{
	return( _objects.setSize(aSize) );
}
//_____________________________________________________________________________
/**
 * Get the size of the array.
 *
 * @return Size of the array.
 */
virtual int getSize() const
{
	return( _objects.getSize() );
}

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
virtual int getIndex(const T *aObject,int aStartIndex=0) const
{
	return( _objects.getIndex(aObject,aStartIndex) );
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
virtual int getIndex(const std::string &aName,int aStartIndex=0) const
{
	return( _objects.getIndex(aName,aStartIndex) );
}

//-----------------------------------------------------------------------------
// APPEND
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Append to the array.  A copy is NOT made of the specified object.
 *
 * @param aObject Object to be appended.
 * @return True if the append was successful, false otherwise.
 */
virtual bool append(T *aObject)
{
	return( _objects.append(aObject) );
}
//_____________________________________________________________________________
/**
 * Append an array of objects.  Copies of the objects are NOT made
 *
 * @param aArray Array of objects to be appended.
 * @return True if the append was successful, false otherwise.
 */
virtual bool append(ArrayPtrs<T> &aArray)
{
	return( _objects.append(aArray) );
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
virtual bool insert(int aIndex,T *aObject)
{
	return( _objects.insert(aIndex,aObject) );
}

//-----------------------------------------------------------------------------
// REMOVE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Remove an object from the array at a specified index.
 * If this set is set as the memory owner, the object is deleted when it
 * is removed.
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
virtual bool remove(int aIndex)
{
	return( _objects.remove(aIndex) );
}
//_____________________________________________________________________________
/**
 * Remove an object from the array by specifying its address.
 * The object is deleted when it is removed.
 *
 * This method is relatively computationally costly since many of the array
 * elements may need to be shifted.
 *
 * @param aObject Pointer to the object to be removed.  If an object with the
 * specified address is not found, no action is taken.
 * @return True if the removal was successful, false otherwise.
 */
virtual bool remove(const T* aObject)
{
	return( _objects.remove(aObject) );
}

//-----------------------------------------------------------------------------
// SET AND GET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the object at a specified index.  A copy of the object is NOT made.
 *
 * @param aIndex Index of the array element to be set.  aIndex must be
 * greater than zero and less than or equal to the size of the array.  Note
 * that if aIndex is equal to the size of the array, the set is equivalent
 * to an append.
 * @param aObject Object to be set.
 * @return True if the set was successful, false otherwise.
 */
virtual bool set(int aIndex,T *aObject)
{
	return( _objects.set(aIndex,aObject) );
}
//_____________________________________________________________________________
/**
 * Get the value at a specified array index.
 *
 * If the index is negative or passed the end of the array, an exception
 * is thrown.
 *
 * For faster execution, the array elements can be accessed through the
 * overloaded operator[], which does no bounds checking.
 *
 * @param aIndex Index of the desired array element.
 * @return Reference to the array element.
 * @throws Exception if (aIndex<0)||(aIndex>=_size) or if the pointer
 * at aIndex is NULL.
 * @see operator[].
 */
virtual T* get(int aIndex) const
{
	return( _objects.get(aIndex) );
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
T* get(const std::string &aName)
{
	return( _objects.get(aName) );
}
//_____________________________________________________________________________
/**
 * Get the last value in the array.
 *
 * @return Last value in the array.
 * @throws Exception if the array is empty.
 */
virtual T* getLast() const
{
	return( _objects.getLast() );
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
 * approprite element is identified), so the performance of this method
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
int searchBinary(const T &aObject,bool aFindFirst=false,
					  int aLo=-1,int aHi=-1) const
{
	return( _objects.searchBinary(aObject,aFindFirst,aLo,aHi) );
}


//=============================================================================
};	// END class Set

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Set_h__
