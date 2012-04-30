#ifndef _PropertyDblVec_h_
#define _PropertyDblVec_h_
// PropertyDblVec.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2010, Stanford University. All rights reserved. 
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

#ifdef WIN32
#pragma warning( disable : 4251 )
#endif

#include "osimCommonDLL.h"
#include <string>
#include "Property_Deprecated.h"
#include "SimTKcommon.h"

//=============================================================================
//=============================================================================
/**
 * Class PropertyDblVec_ extends class Property.  It consists of a small
 * vector of doubles (i.e., SimTK::Vec<M>) and the methods for accessing
 * and modifying this Vec<M>.
 *
 * @version 1.0
 * @author Ayman Habib, Ajay Seth
 */
namespace OpenSim { 

template<int M> class PropertyDblVec_ : public Property_Deprecated
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Vector of doubles. */
	SimTK::Vec<M> _vec;

	/** Array representation for serialization */
	mutable Array<double> _array;

	char _typeAsString[7];

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	/** Default constructor */
	PropertyDblVec_() : Property_Deprecated(DblVec, "DblVec_PropertyName")
		{ sprintf(_typeAsString, "DblVec%i", M);
		  setAllowableListSize(M);	  
		};
	/** Construct from name and value */
	PropertyDblVec_(const std::string &aName, const SimTK::Vec<M>& aVec)
    :   Property_Deprecated(DblVec, aName) 
		{ sprintf(_typeAsString, "DblVec%i", M);  setValue(aVec);
		  setAllowableListSize(M);

		};
	/** Construct from name and value as an Array<double> */
	PropertyDblVec_(const std::string &aName, const Array<double> &anArray)
    :   Property_Deprecated(DblVec, aName)
		{ sprintf(_typeAsString, "DblVec%i", M);  setValue(anArray);
		  setAllowableListSize(M);
		};
	/** Copy constructor */
	PropertyDblVec_(const PropertyDblVec_ &aProperty)
    :   Property_Deprecated(aProperty)
		{ setValue(aProperty._vec);};

	/* Return a copy of this property */
	/*virtual*/ PropertyDblVec_* clone() const {
		PropertyDblVec_* prop = new PropertyDblVec_<M>(*this);
		return prop;
	};

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	/** Assign this property to another */
	PropertyDblVec_& operator=(const PropertyDblVec_ &aProperty){
		Property_Deprecated::operator=(aProperty);
		_vec = aProperty._vec;
		return(*this);
	};

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	/** Get the type of this property as a string. */
	virtual std::string getTypeName() const OVERRIDE_11 {
		return "double";
	};

	// VALUE
	/** set value of property from an equivalently sized Vec */
	virtual void setValue(const SimTK::Vec<M> &aVec) { 
		assert(aVec.size()==M); _vec=aVec; 
	};
	/** set value of this property from an array of doubles of equal or greater length */
	virtual void setValue(const Array<double> &anArray){
		assert(anArray.getSize() >= M);
		for(int i=0;i<M; i++)
			_vec[i] = anArray[i];
		_array = anArray;
	};
	/** get writable reference to the value */
	virtual SimTK::Vec<M>& getValueDblVec() {return _vec; };
	/** get const (read-only) reference to the value */
	virtual const SimTK::Vec<M>& getValueDblVec() const {return _vec; };
	/** set value from double array */
	virtual void setValue(int aSize, const double aArray[]){ // to be used by the serialization code
		assert(aSize == M);
		setValue(SimTK::Vec<M>::getAs(aArray));
	};
#ifndef SWIG
	/** get value as double array */
	virtual const Array<double>& getValueDblArray() const {
		_array.setSize(M);
		for(int i=0;i<M; i++)
			_array[i] = _vec[i];
		return _array;
	};
#endif
	/** Nonconst version of accessor for use by GUI.
	 * Beware: changes to returned array will not change _vec member as it should
	 * both _vec and _array should point to same memory to fix this issue
	 */
	virtual Array<double>& getValueDblArray() {
		_array.setSize(M);
		for(int i=0;i<M; i++)
			_array[i] = _vec[i];
		return _array;
	};
	/** Get a constant String represeting the value of this property. */
	virtual std::string toString() const {
		std::string str = "(";
		char dbl[256];
			for(int i=0; i < _vec.size(); i++){
				sprintf(dbl, "%g", _vec[i]);
				str += (i>0?" ":"") + std::string(dbl);
			}
		str += ")";
		return str;
	};
	// SIZE
	virtual int getArraySize() const { return M; }

    virtual bool isArrayProperty() const {return true;}

    virtual int getNumValues() const OVERRIDE_11 {return M;}
//=============================================================================
};	// END of class PropertyDblVec_

}; //OpenSim namespace
//=============================================================================
//=============================================================================

#endif //__PropertyDblVec__h__
