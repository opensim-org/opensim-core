#ifndef _Property_h_
#define _Property_h_
// Property.h
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
#include <string>
#include "Array.h"
#include "ArrayPtrs.h"

namespace OpenSim { 

class Object;

#ifdef SWIG
	#ifdef RDTOOLS_API
		#undef RDTOOLS_API
		#define RDTOOLS_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * A property consists of a type, name, and a value or an array of values.
 *
 * Class Property is an abstract base class that provides the functionality
 * common to all property types.
 *
 * At the time of the first formulation of the property classes, the only
 * property types that were envisioned are for a few fundamental data types
 * and for Object:\n\n
 * \tbool\n
 * \tint\n
 * \tfloat\n
 * \tdouble\n
 * \tstring\n
 * \tObject\n
 * \tarray of bools\n
 * \tarray of ints\n
 * \tarray of floats\n
 * \tarray of doubles\n
 * \tarray of strings\n
 * \tarray of Objects\n\n
 *
 * As additional property types are needed, they may be added however.
 *
 * @todo Make default constructors for all derived classes
 * @version 1.0
 * @author Frank C. Anderson
 */
#ifdef WIN32
#pragma warning( disable : 4290 )	// VC++ non-ANSI Exception handling
#endif

class RDTOOLS_API Property  
{

//=============================================================================
// DATA
//=============================================================================
public:
	/** Enumeration of recognized types. */
	enum PropertyType
	{
		None=0,Bool, Int, Flt, Dbl, Str, Obj,
		BoolArray, IntArray, FltArray, DblArray, StrArray, ObjArray
	};

private:
	/** Type of the property. */
	PropertyType _type;
	/** Name of the property. */
	std::string _name;
	/** Flag indicating whether or not this property uses some
	default property for initializing its value. */
	bool _useDefault;
protected:
	/** Comment to be associated with property, shown for default objects only
	for efficiency. */
	std::string _comment;
	/** String representation of property */
	std::string _valueString;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Property();
	Property(PropertyType aType,const std::string &aName);
	Property(const Property &aProperty);
	virtual Property* copy() const=0;
	virtual ~Property() {};
	void setNull();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Property& operator=(const Property &aProperty);
	virtual bool operator==(const Property &aProperty) const;
	virtual bool operator<(const Property &aProperty) const;
	friend std::ostream& operator<<(std::ostream &aOut,const Property &aProperty) {
		aOut << aProperty.getTypeAsString() << " " << aProperty.getName();
		return(aOut);
	};
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:

	// TYPE
	void setType(PropertyType aType);
	PropertyType getType() const;
	virtual const char* getTypeAsString() const;

	// NAME
	void setName(const std::string &aName);
	const std::string& getName() const;

	// Comment
	void setComment(const std::string &aComment) { _comment = aComment; };
	const std::string& getComment() const { return _comment; };
	// VALUE
	// Textual representation
	virtual const std::string &toString()=0;
	// Bool
	virtual void setValue(bool aValue);
	virtual bool& getValueBool();
#ifndef SWIG
	virtual const bool& getValueBool() const;
#endif
	// Int
	virtual void setValue(int aValue);
	virtual int& getValueInt();
#ifndef SWIG
	virtual const int& getValueInt() const;
#endif
	// Dbl
	virtual void setValue(double aValue);
	virtual double& getValueDbl();
#ifndef SWIG
	virtual const double& getValueDbl() const;
#endif
	// Str
	virtual void setValue(const std::string &aValue);
	virtual std::string& getValueStr();
#ifndef SWIG
	virtual const std::string& getValueStr() const;
#endif
	// Obj
	virtual void setValue(const Object &aValue);
	virtual Object& getValueObj();
#ifndef SWIG
	virtual const Object& getValueObj() const;
#endif
	// Bool Array
	virtual void setValue(int aSize,const bool aArray[]);
	virtual void setValue(const Array<bool> &aArray);
	virtual Array<bool>& getValueBoolArray();
#ifndef SWIG
	virtual const Array<bool>& getValueBoolArray() const;
#endif
	// Int Array
	virtual void setValue(int aSize,const int aArray[]);
	virtual void setValue(const Array<int> &aArray);
	virtual Array<int>& getValueIntArray();
#ifndef SWIG
	virtual const Array<int>& getValueIntArray() const;
#endif
	// Dbl Array
	virtual void setValue(int aSize,const double aArray[]);
	virtual void setValue(const Array<double> &aArray);
	virtual Array<double>& getValueDblArray();
#ifndef SWIG
	virtual const Array<double>& getValueDblArray() const;
#endif
	// Str Array
	virtual void setValue(int aSize,const std::string aArray[]);
	virtual void setValue(const Array<std::string> &aArray);
	virtual Array<std::string>& getValueStrArray();
#ifndef SWIG
	virtual const Array<std::string>& getValueStrArray() const;
#endif
	// Obj Array
	virtual void setValue(int aSize,Object **aArray);
	virtual void setValue(const ArrayPtrs<Object> &aArray);
	virtual ArrayPtrs<Object>& getValueObjArray();
#ifndef SWIG
	virtual const ArrayPtrs<Object>& getValueObjArray() const;
#endif
	// USE DEFAULT
	void setUseDefault(bool aTrueFalse);
	bool getUseDefault() const;

//=============================================================================
};	// END of class Property

}; //namespace
//=============================================================================
//=============================================================================

#endif //__Property_h__
