#ifndef _NamedValueArray_h_
#define _NamedValueArray_h_
// NamedValueArray.h
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

/*  
 * Author:  
 */


#include <vector>
#include "rdTools.h"
#include "Exception.h"


#ifdef WIN32
#pragma warning( disable : 4251 )
#endif


//=============================================================================
//=============================================================================
/**
* A class for maintaining a list of primitive data-type named quantities. 
* This class allows for searching the list by index (fast) or by name (linear search/slow).
* vector as opposed as a hashmap is used as underlying datastructure since interfacing
* to SDFast dynamics routines is entirely based on using indices so dereferncing using index
* is assumed to be the predominant way to access entries in the list.
*
* @version 1.0
* @author Ayman Habib
* 
*/
namespace OpenSim { 

template <class T> class NamedValueArray
{

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// MEMBER VARIABLES
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
private:
	std::vector<T> _values;
	std::vector<std::string> _names;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// METHODS
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public:
	//=============================================================================
	// CONTRUCTION
	//=============================================================================
	//____________________________________________________________________________
	/**
	 * Destructor.
	 */
	virtual ~NamedValueArray()
	{
	};
	//____________________________________________________________________________
	/**
	 * Default constructor
	 */
	NamedValueArray() {};
	//____________________________________________________________________________
	/**
	 * Constructor that allocates aSize entries
	 */
	NamedValueArray(int aSize) 
	{
		_values.resize(aSize);
		_names.resize(aSize);
	};

	//=============================================================================
	// Operations
	//=============================================================================
	//_____________________________________________________________________________
	/**
	 * Check if NamedValueArray is empty
	 */
	bool empty() const
	{
		return _values.empty();
	}
	//_____________________________________________________________________________
	/**
	 * Clear
	 */
	void clear()
	{
		_values.clear();
		_names.clear();
	}
	//____________________________________________________________________________
	/**
	 * Append to the end of the NamedValueArray
	 */
	void append(const T& aValue, const std::string& aName) 
	{ 
		_values.push_back(aValue); 
		_names.push_back(aName);
	};

	//_____________________________________________________________________________
	/**
	 * Get number of values in NamedValueArray
	 */
	int getSize() const
	{
		return _values.size();
	};

	//_____________________________________________________________________________
	/**
	 * Set number of values in NamedValueArray, allocates space
	 */
	void setSize(const int newSize)
	{
		_values.resize(newSize);
		_names.resize(newSize);
	}

	//_____________________________________________________________________________
	/**
	 * Get objects that live in NamedValueArray as a raw array of values
	 * Memory is managed by the caller. Proper size need be allocated
	 * based on getSize.
	 */
	void getValues(T* aValuesArray) const
	{
		int idx = 0;
		for(typename std::vector<T>::const_iterator iter = _values.begin(); 
								 iter != _values.end(); 
								 iter++, idx++){
			aValuesArray[idx] = _values[idx];
		}
	};
	//_____________________________________________________________________________
	/**
	 * Get objects that live in NamedValueArray as a raw array of values
	 * This function (ab)uses the internal representation of stl vectors
	 * The pointer returned is valid only as long as no reallocation of 
	 * the vector due to resizing is performed.
	 *
	 * returns raw pointer to first entry in the vector of values (C-style)
	 */
	T* get()
	{
		return &(_values[0]);
	};

	/**
	 * set values from raw values, 
	 * size is passed in. Names are not affected 
	 * if size is not passed in it's assumed preset and obtained from getSize()
	 */
	void setValues(const T *aValuesArray, int aSize=-1)
	{
		int newSize = (aSize==-1)? getSize():aSize;
		if (getSize() != newSize)
			setSize(newSize);
		for(int idx=0; idx < newSize; idx++){
			_values[idx] = aValuesArray[idx];
		}
	};
	//_____________________________________________________________________________
	/**
	 * set values from raw values, names are not modified
	 * size is assumed to be preset 
	 */

	void setValueNames(std::string* & aValuesArray)
	{
		int idx = 0;
		for(typename std::vector<T>::iterator iter = _values.begin(); 
								 iter != _values.end(); 
								 iter++, idx++){
			_names[idx] = aValuesArray[idx];
		}
	};
	//_____________________________________________________________________________
	/**
	 * Get value corresponding to index idx.
	 * Only reason for returning a pointer rather than a reference is to handle
	 * error conditions where idx is out of range (return 0).
	 * May need to return a reference instead and throw an exception on error
	 */
	T* getByIndex(int idx) const
	{
		if (idx <0 || idx > getSize()-1){
			// This should throw an exception 
			return ((T *)0);
		}
		// index is in range
		return ((T *)&(_values[idx]));
	};

	//_____________________________________________________________________________
	/**
	 * Get value corresponding to name passed in
	 * Only reason for returning a pointer rather than a reference is to handle
	 * error conditions where idx is out of range (return 0).
	 * May need to return a reference instead and throw an exception on error
	 */
	T* getByName(const std::string &aName)const
	{
		T *returnValue=0;

		typename std::vector<T>::const_iterator valIterator = _values.begin();
		for(std::vector<std::string>::const_iterator iter = _names.begin(); 
								 iter != _names.end() && valIterator != _values.end(); 
								 iter++, valIterator++){
			if ((*iter) == aName){
				returnValue = (T*) (&(* valIterator));
				break;
			}
		}
		return returnValue;
	};

	//_____________________________________________________________________________
	/**
	 * Set name associated with value at index idx
	 */
	void setNameAt(int idx, const std::string &aName) 
	{
		_names.at(idx) = aName;
	};

	//_____________________________________________________________________________
	/**
	 * Get name associated with value at index idx
	 */
	const char *getNameAt(int idx) const
	{
		return _names[idx].c_str();
	};

	//_____________________________________________________________________________
	/**
	 * Get index for object associated with name aName
	 */
	int getIndex(const std::string &aName) const
	{
		int returnIndex=-1;
		int idx=0;

		for(std::vector<std::string>::const_iterator iter = _names.begin();
			iter != _names.end();
			iter++, idx++){
				if ((*iter)== aName){
					returnIndex  = idx;
					break;
				}
			}

		return returnIndex;
	};
	//_____________________________________________________________________________
	/**
	 * Indexing with aIndex, const object
	 * std::vector throws exception if out of bounds, 
	 * @todo we can handle it more gracefully here
	 */
	const T& operator[](int aIndex) const
	{
		return(_values[aIndex]);
	}
	//_____________________________________________________________________________
	/**
	 * Indexing by name, const object
	 * If name is not found this will index out of bounds and throw an exception
	 */
	const T& operator[](const std::string& aName) const
	{
		int aIndex = getIndex(aName);
		return (_values[aIndex]);

	}

	//_____________________________________________________________________________
	/**
	 * Indexing with aIndex, non-const object
	 */
	T& operator[](int aIndex)
	{
		return(_values[aIndex]);
	}
	//_____________________________________________________________________________
	/**
	 * Indexing by name, non-const object
	 * If name is not found this will index out of bounds and throw an exception
	 */
	T& operator[](const std::string& aName)
	{
		int aIndex = getIndex(aName);
		return (_values[aIndex]);
	}

	//=============================================================================
};	// END of class NamedValueArray

}; //namespace
//=============================================================================

#endif //__ObjectVector_h__
