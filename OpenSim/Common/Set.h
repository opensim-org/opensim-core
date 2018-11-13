#ifndef OPENSIM_SET_H_
#define OPENSIM_SET_H_
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  Set.h                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2018 Stanford University and the Authors                *
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

// INCLUDES
#include <iostream>
#include "osimCommonDLL.h"
#include "Object.h"
#include "ArrayPtrs.h"
#include "ObjectGroup.h"
#include "PropertyObjArray.h"

namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of pointers to objects.  It is derived from
 * base class C and is implemented as a wrapper around template class
 * ArrayPtrs<T>.  
 *
 * @see ArrayPtrs
 * @author Frank C. Anderson
 */
template<class T, class C=Object> class Set : public C {
OpenSim_DECLARE_CONCRETE_OBJECT_T(Set, T, C);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// DATA
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
protected:
// PROPERTIES
/** Array of pointers to objects. */
PropertyObjArray<T> _propObjects;
/** Array of pointers to object groups. */
PropertyObjArray<ObjectGroup> _propObjectGroups;

// REFERENCES
ArrayPtrs<T> &_objects;
ArrayPtrs<ObjectGroup> &_objectGroups;

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
Set() : Super(),
    _objects((ArrayPtrs<T>&)_propObjects.getValueObjArray()),
    _objectGroups((ArrayPtrs<ObjectGroup>&)_propObjectGroups.getValueObjArray())
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Construct from file.
 *
 * @param aFileName             Name of the file.
 * @param aUpdateFromXMLNode    Whether to update from XML.
 */
Set(const std::string &aFileName, bool aUpdateFromXMLNode = true) SWIG_DECLARE_EXCEPTION
    : Super(aFileName),
    _objects((ArrayPtrs<T>&)_propObjects.getValueObjArray()),
    _objectGroups((ArrayPtrs<ObjectGroup>&)_propObjectGroups.getValueObjArray())
{
    setNull();
    if (aUpdateFromXMLNode)
        this->updateFromXMLDocument();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSet Set to be copied.
 */
Set(const Set<T,C> &aSet) : Super(aSet),
    _objects((ArrayPtrs<T>&)_propObjects.getValueObjArray()),
    _objectGroups((ArrayPtrs<ObjectGroup>&)_propObjectGroups.getValueObjArray())
{
    setNull();
    _objects = aSet._objects;
    _objectGroups = aSet._objectGroups;
}


private:
//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * %Set all member variables to NULL values.
 */
void
setNull()
{
    setupProperties();
    _objects.setSize(0);
    _objectGroups.setSize(0);
}
//_____________________________________________________________________________
/**
 * Setup serialized member variables.
 */
void
setupProperties()
{
    _propObjects.setName("objects");
    this->_propertySet.append(    &_propObjects );

    _propObjectGroups.setName("groups");
    this->_propertySet.append(    &_propObjectGroups );
}

public:
//_____________________________________________________________________________
/**
 * Setup groups (match group member names to set members).
 */
void
setupGroups()
{
    int i;
    for (i=0; i<_objectGroups.getSize(); i++) {
        _objectGroups.get(i)->setupGroup((ArrayPtrs<Object>&)_objects);
    }
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
 * @param set The Set to be copied.
 * @return Reference to this set.
 */
#ifndef SWIG
Set<T,C>& operator=(const Set<T,C> &set)
{   
    Super::operator=(set);
    _objects = set._objects;
    _objectGroups = set._objectGroups;

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
T& operator[](int aIndex) const
{
    return( *_objects[aIndex] );
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
friend std::ostream& operator<<(std::ostream &aOut,const Set<T, C> &aSet)
{
    aOut << "Set[" << aSet.getSize() <<"] =";

    for(int i=0;i<aSet.getSize();i++)  {
        aOut << " " << aSet[i];
    }

    return(aOut);
}

#endif

//=============================================================================
// MEMORY OWNERSHIP
//=============================================================================
//_____________________________________________________________________________
/**
 * %Set whether or not this Set owns the memory pointed to by the pointers
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

#ifndef SWIG
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
#endif // SWIG
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
int getSize() const
{
    return( _objects.getSize() );
}

//-----------------------------------------------------------------------------
// INDEX
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the index of an object.
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
//_____________________________________________________________________________
/**
 * Get names of groups containing a given object 
 */
void getGroupNamesContaining(const std::string &aObjectName, Array<std::string> &rGroupNames) const
{
    rGroupNames.setSize(0);
    for(int i=0; i<_objectGroups.getSize(); i++)
        if(_objectGroups[i]->contains(aObjectName))
            rGroupNames.append(_objectGroups[i]->getName());
}
//-----------------------------------------------------------------------------
// APPEND
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Append to the array, and adopt passed in pointer.  A copy is NOT made of the specified object.  If
 * getMemoryOwner() is true, this Set takes over ownership of the object and
 * deletes it when the Set itself is deleted.
 *
 * @param aObject Object to be appended.
 * @return True if the append was successful, false otherwise.
 */
virtual bool adoptAndAppend(T *aObject)
{
    return( _objects.append(aObject) );
}

//_____________________________________________________________________________
/**
 * cloneAndAppend creates a clone of the passed in object and appends the clone to the array.  
 * The original object is unaffected and is not associated with the Set. The clone is created 
 * using the method clone() available to OpenSim::Object
 *
 * @param aObject Object whose clone is to be appended.
 * @return True if the append was successful, false otherwise.
 */
virtual bool cloneAndAppend(const T& aObject)
{
    return adoptAndAppend(aObject.clone());
}
//-----------------------------------------------------------------------------
// INSERT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Insert an object into the array at a specified index.  A copy of the
 * specified object is NOT made.  If getMemoryOwner() is true, this Set takes
 * over ownership of the object and deletes it when the Set itself is deleted.
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
#ifndef SWIG
//_____________________________________________________________________________
/**
 * Insert an object into the array at a specified index.  A copy is made of the
 * object and added to the Set.  The original object is unaffected.
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
virtual bool insert(int aIndex, const T& aObject)
{
    return insert(aIndex, aObject.clone());
}
#endif
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
    // remove the object from all of the groups
    int i;
    for (i=0; i<_objectGroups.getSize(); i++)
        _objectGroups.get(i)->remove(_objects.get(aIndex));

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
    // remove the object from all of the groups
    int i;
    for (i=0; i<_objectGroups.getSize(); i++)
        _objectGroups.get(i)->remove(aObject);

    return( _objects.remove(aObject) );
}

virtual void clearAndDestroy()
{
    _objects.clearAndDestroy();
    _objectGroups.clearAndDestroy();
}

//-----------------------------------------------------------------------------
// SET AND GET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * %Set the object at a specified index.  A copy of the object is NOT made.
 * If getMemoryOwner() is true, this Set takes over ownership of the object and
 * deletes it when the Set itself is deleted.
 *
 * @param aIndex Index of the array element to be set.  aIndex must be
 * greater than zero and less than or equal to the size of the array.  Note
 * that if aIndex is equal to the size of the array, the set is equivalent
 * to an append.
 * @param aObject Object to be set.
 * @param preserveGroups If true, the new object will be added to the groups
 * that the object it replaces belonged to
 * @return True if the set was successful, false otherwise.
 */
virtual bool set(int aIndex, T *aObject, bool preserveGroups = false)
{
    if (!preserveGroups)
        return( _objects.set(aIndex,aObject) );
    if (aObject != NULL && aIndex >= 0 && aIndex < _objects.getSize())
    {
        for (int i = 0; i < _objectGroups.getSize(); i++)
            _objectGroups.get(i)->replace(_objects.get(aIndex), aObject);
        _objects.remove(aIndex);
        return _objects.insert(aIndex, aObject);
    }
    return false;
}
#ifndef SWIG
//_____________________________________________________________________________
/**
 * %Set the object at a specified index.  A copy is made of the
 * object and added to the Set.  The original object is unaffected.
 *
 * @param aIndex Index of the array element to be set.  aIndex must be
 * greater than zero and less than or equal to the size of the array.  Note
 * that if aIndex is equal to the size of the array, the set is equivalent
 * to an append.
 * @param aObject Object to be set.
 * @param preserveGroups If true, the new object will be added to the groups
 * that the object it replaces belonged to
 * @return True if the set was successful, false otherwise.
 */
virtual bool set(int aIndex, const T& aObject, bool preserveGroups = false)
{
    return set(aIndex, aObject.clone(), preserveGroups);
}
#endif
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
virtual T& get(int aIndex) const
{
    return( *_objects.get(aIndex) );
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
T& get(const std::string &aName)
{
    return( *_objects.get(aName) );
}
#ifndef SWIG
const T& get(const std::string &aName) const
{
    return( *_objects.get(aName) );
}
#endif
//_____________________________________________________________________________
/**
 * Get whether this Set contains any object with the specified name.
 *
 * @param aName Name of the desired object.
 * @return true if the object exists
 */
bool contains(const std::string &aName) const
{
    return( _objects.getIndex(aName) != -1 );
}//_____________________________________________________________________________
/**
 * Get names of objects in the set.
 *
 * @param rNames Array of names.  The names are appended to rNames, so it
 * is permissible to send in an non-empty array; the names in the set
 * will simply be appended to the array sent in.
 */
virtual void getNames(OpenSim::Array<std::string> &rNames ) const
{
    for(int i=0;i<_objects.getSize();i++) {
        T *obj = _objects[i];
        if(obj==NULL) {
            rNames.append("NULL");
        } else {
            rNames.append(obj->getName());
        }
    }
}
#ifndef SWIG
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
 * appropriate element is identified), so the performance of this method
 * is approximately ln(n), where n is the size of the array.
 *
 * @param aObject Value to which the array elements are compared.
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
#endif
//=============================================================================
// GROUPS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the number of groups.
 */
int getNumGroups() const
{
    return _objectGroups.getSize();
}

//_____________________________________________________________________________
/**
 * Add an empty group to the set.
 */
void addGroup(const std::string& aGroupName)
{
    _objectGroups.append(new ObjectGroup(aGroupName));
}

//_____________________________________________________________________________
/**
 * Remove a group from the set. Elements are not removed.
 */
void removeGroup(const std::string& aGroupName)
{
    _objectGroups.remove(_objectGroups.get(aGroupName));
}

//_____________________________________________________________________________
/**
 * Rename a group.
 * 
 */
void renameGroup(const std::string& oldGroupName, const std::string& newGroupName)
{
    ObjectGroup* grp=_objectGroups.get(oldGroupName);
    //TODO This should check for duplicates and throw an exception if duplicate
    grp->setName(newGroupName);
}

//_____________________________________________________________________________
/**
 * Add an object to a group.
 */
void addObjectToGroup(const std::string& aGroupName, const std::string& aObjectName)
{
    ObjectGroup* group = _objectGroups.get(aGroupName);
    Object* object = _objects.get(aObjectName);
    if (group && object)
        group->add(object);
}

//_____________________________________________________________________________
/**
 * Get names of all groups
 */
void getGroupNames(Array<std::string> &rGroupNames) const
{
    rGroupNames.setSize(0);
    for(int i=0; i<_objectGroups.getSize(); i++)
        rGroupNames.append(_objectGroups[i]->getName());
}

//_____________________________________________________________________________
/**
 * Get a group by name.
 */
const ObjectGroup* getGroup(const std::string& aGroupName) const
{
    return _objectGroups.get(aGroupName);
}
//_____________________________________________________________________________
/**
 * Get a group by index.
 */
const ObjectGroup* getGroup(int aIndex) const
{
    return _objectGroups.get(aIndex);
}

//=============================================================================
};  // END class Set

}; //namespace
//=============================================================================
//=============================================================================

#endif  // OPENSIM_SET_H_
