// Property.cpp
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


//============================================================================
// INCLUDES
//============================================================================
#include "Property.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Property::
Property()
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
Property::
Property(PropertyType aType,const string &aName)
{
	setNull();
	_type = aType;
	_name = aName;
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aProperty Property to be copied.
 */
Property::Property(const Property &aProperty)
{
	setNull();
	*this = aProperty;
	_comment = aProperty.getComment();
}
//_____________________________________________________________________________
/**
 * Construct and return a copy of this property.
 *
 * The property is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this property.
 *
Property* Property::copy() const
{
	Property *property = new Property(*this);
	return(property);
}
*/

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member variables to their null values.
 */
void Property::
setNull()
{
	_type = Property::None;
	_name = "unknown";
	_useDefault = false;
}



//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this property to another.
 *
 * @param aProperty Property to which to assign this property.
 * @return Reference to this property.
 */
Property& Property::
operator=(const Property &aProperty)
{
	setType(aProperty.getType());
	setName(aProperty.getName());
	setUseDefault(aProperty.getUseDefault());
	_comment = aProperty.getComment();
	return(*this);
}

//-----------------------------------------------------------------------------
// EQUALITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine if two properties are equal.
 *
 * Two properties are equal if their names are the same; the types do not
 * need to be the same.
 *
 * @param aProperty Property for which to make the equality test.
 * @return True if the specified property and this property are equal, false
 * otherwise.
 */
bool Property::
operator==(const Property &aProperty) const
{
	if(_name != aProperty.getName()) return(false);
	return(true);
}

//-----------------------------------------------------------------------------
// LESS THAN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine if this property is less than another.
 *
 * This property is less than another if the name of this string is less
 * than the name of the other property.
 *
 * @param aProperty Property for which to make the less than test.
 * @return True if this property is less than the other, false otherwise.
 */
bool Property::
operator<(const Property &aProperty) const
{
	return(_name < aProperty.getName());
}

//-----------------------------------------------------------------------------
// OUTPUT (<<)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Implementation of the output operator.
 * The output consists of the type and name:\n\n
 *
 * @param aOut Output stream.
 * @param aProperty Property to be output.
 * @return Reference to the output stream.
ostream& operator<<(ostream &aOut,const Property &aProperty)
{
	aOut << aProperty.getTypeAsString() << " " << aProperty.getName();
	return(aOut);
}
 */


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// TYPE AS STRING
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the type of this property as a string.
 *
 * @return Type of the property.
 */
const char* Property::
getTypeAsString() const
{
	return("None");
}

//-----------------------------------------------------------------------------
// TYPE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the type of this property.
 *
 * @param aType Type of this object represented as a string.  In most all
 * cases, the type should be the name of the class.
 */
void Property::
setType(PropertyType aType)
{
	_type = aType;
}
//_____________________________________________________________________________
/**
 * Get the type of this property.
 *
 * @return Type of the property.
 */
Property::PropertyType Property::
getType() const
{
	return(_type);
}

//-----------------------------------------------------------------------------
// NAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the name of this property.
 *
 * @param aName Name to which to set this property.
 */
void Property::
setName(const string &aName)
{
	_name = aName;
}
//_____________________________________________________________________________
/**
 * Get the name of this property.
 *
 * @retrun Name of this property
 */
const string& Property::
getName() const
{
	return(_name);
}

//-----------------------------------------------------------------------------
// VALUE BOOL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyInt::setValue(), which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aValue Value to which this property is to be assigned.
 * @see PropertyInt
 */
void Property::
setValue(bool aValue)
{
	string msg;
	msg = "Property.SetValue(bool): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 * @throws Exception When the property is not an PropertyInt.
 * @see PropertyInt
 */
bool& Property::
getValueBool()
{
	string msg;
	msg = "Property.getValueBool(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Constant reference to the value of this property.
 * @throws Exception When the property is not an PropertyInt.
 * @see PropertyInt
 */
const bool& Property::
getValueBool() const
{
	string msg;
	msg = "Property.getValueBool(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}

//-----------------------------------------------------------------------------
// VALUE INT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyInt::setValue(), which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aValue Value to which this property is to be assigned.
 * @see PropertyInt
 */
void Property::
setValue(int aValue)
{
	string msg;
	msg = "Property.SetValue(int): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 * @throws Exception When the property is not an PropertyInt.
 * @see PropertyInt
 */
int& Property::
getValueInt()
{
	string msg;
	msg = "Property.getValueInt(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Constant reference to the value of this property.
 * @throws Exception When the property is not an PropertyInt.
 * @see PropertyInt
 */
const int& Property::
getValueInt() const
{
	string msg;
	msg = "Property.getValueInt(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}

//-----------------------------------------------------------------------------
// VALUE DOUBLE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyDbl::setValue(), which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aValue Value to which this property is to be assigned.
 * @see PropertyDbl
 */
void Property::
setValue(double aValue)
{
	string msg;
	msg = "Property.setValue(double): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 * @throws Exception When the property is not an PropertyDbl.
 * @see PropertyDbl
 */
double& Property::
getValueDbl()
{
	string msg;
	msg = "Property.getValueDbl(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Const reference to the value of this property.
 * @throws Exception When the property is not an PropertyDbl.
 * @see PropertyDbl
 */
const double& Property::
getValueDbl() const
{
	string msg;
	msg = "Property.getValueDbl(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}

//-----------------------------------------------------------------------------
// VALUE STRING
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyStr::setValue(), which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aValue Value to which this property is to be assigned.
 * @see PropertyStr
 */
void Property::
setValue(const string &aValue)
{
	string msg;
	msg = "Property.setValue(string): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 * @throws Exception When the property is not an PropertyInt.
 * @see PropertyStr
 */
string& Property::
getValueStr()
{
	string msg;
	msg = "Property.getValueStr(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Constant reference to the value of this property.
 * @throws Exception When the property is not an PropertyInt.
 * @see PropertyStr
 */
const string& Property::
getValueStr() const
{
	string msg;
	msg = "Property.getValueStr(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}

//-----------------------------------------------------------------------------
// VALUE OBJECT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 * @throws Exception When the property is not an PropertyObj.
 * @see PropertyObj
 */
Object& Property::
getValueObj()
{
	string msg;
	msg = "Property.getValueObj(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Constant reference to the value of this property.
 * @throws Exception When the property is not an PropertyObj.
 * @see PropertyObj
 */
const Object& Property::
getValueObj() const
{
	string msg;
	msg = "Property.getValueObj(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}

//-----------------------------------------------------------------------------
// VALUE BOOL ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyBoolArray::setValue(), which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aSize Size of the array.
 * @param aArray Array to which this property is to be assigned.
 * @throws Exception When the property is not an PropertyBoolArray.
 * @see PropertyBoolArray
 */
void Property::
setValue(int aSize,const bool aArray[])
{
	string msg;
	msg = "Property.setValue(int,bool[]): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyBoolArray::setValue, which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aArray Array to which this property is to be assigned.
 * @throws Exception When the property is not an PropertyBoolArray.
 * @see PropertyBoolArray
 */
void Property::
setValue(const Array<bool> &aArray)
{
	string msg;
	msg = "Property.getValue(const Array<bool>&): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 * @throws Exception When the property is not an PropertyBoolArray.
 * @see PropertyBoolArray
 */
Array<bool>& Property::
getValueBoolArray()
{
	string msg;
	msg = "Property.getValueBoolArray(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Constant reference to the value of this property.
 * @throws Exception When the property is not an PropertyBoolArray.
 * @see PropertyBoolArray
 */
const Array<bool>& Property::
getValueBoolArray() const
{
	string msg;
	msg = "Property.getValueBoolArray(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}

//-----------------------------------------------------------------------------
// VALUE INT ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyIntArray::setValue(), which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aSize Size of the array.
 * @param aArray Array to which this property is to be assigned.
 * @throws Exception When the property is not an PropertyIntArray.
 * @see PropertyIntArray
 */
void Property::
setValue(int aSize,const int aArray[])
{
	string msg;
	msg = "Property.setValue(int,int[]): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyIntArray::setValue, which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aArray Array to which this property is to be assigned.
 * @throws Exception When the property is not an PropertyIntArray.
 * @see PropertyIntArray
 */
void Property::
setValue(const Array<int> &aArray)
{
	string msg;
	msg = "Property.getValue(const Array<int>&): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 * @throws Exception When the property is not an PropertyIntArray.
 * @see PropertyIntArray
 */
Array<int>& Property::
getValueIntArray()
{
	string msg;
	msg = "Property.getValueIntArray(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Constant reference to the value of this property.
 * @throws Exception When the property is not an PropertyIntArray.
 * @see PropertyIntArray
 */
const Array<int>& Property::
getValueIntArray() const
{
	string msg;
	msg = "Property.getValueIntArray(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}

//-----------------------------------------------------------------------------
// VALUE DOUBLE ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyDblArray::setValue(), which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aSize Size of the array.
 * @param aArray Array to which this property is to be assigned.
 * @throws Exception When the property is not an PropertyDblArray.
 * @see PropertyDblArray
 */
void Property::
setValue(int aSize,const double aArray[])
{
	string msg;
	msg = "Property.setValue(int,double[]): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyDblArray::setValue, which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aArray Array to which this property is to be assigned.
 * @throws Exception When the property is not an PropertyDblArray.
 * @see PropertyDblArray
 */
void Property::
setValue(const Array<double> &aArray)
{
	string msg;
	msg = "Property.getValue(const Array<double>&): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 * @throws Exception When the property is not an PropertyDblArray.
 * @see PropertyDblArray
 */
Array<double>& Property::
getValueDblArray()
{
	string msg;
	msg = "Property.getValueDblArray(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Constant reference to the value of this property.
 * @throws Exception When the property is not an PropertyDblArray.
 * @see PropertyDblArray
 */
const Array<double>& Property::
getValueDblArray() const
{
	string msg;
	msg = "Property.getValueDblArray(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}

//-----------------------------------------------------------------------------
// VALUE STRING ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyStrArray::setValue(), which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aSize Size of the array.
 * @param aArray Array to which this property is to be assigned.
 * @throws Exception When the property is not an PropertyStrArray.
 * @see PropertyStrArray
 */
void Property::
setValue(int aSize,const string aArray[])
{
	string msg;
	msg = "Property.setValue(int,string[]): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyStrArray::setValue, which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aArray Array to which this property is to be assigned.
 * @throws Exception When the property is not an PropertyStrArray.
 * @see PropertyStrArray
 */
void Property::
setValue(const Array<string> &aArray)
{
	string msg;
	msg = "Property.getValue(const Array<string>&): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 * @throws Exception When the property is not an PropertyStrArray.
 * @see PropertyStrArray
 */
Array<string>& Property::
getValueStrArray()
{
	string msg;
	msg = "Property.getValueStrArray(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Constant reference to the value of this property.
 * @throws Exception When the property is not an PropertyStrArray.
 * @see PropertyStrArray
 */
const Array<string>& Property::
getValueStrArray() const
{
	string msg;
	msg = "Property.getValueStrArray(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}

//-----------------------------------------------------------------------------
// VALUE RDOBJECT ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyObjArray::setValue(), which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aSize Size of the array.
 * @param aArray Array to which this property is to be assigned.
 * @throws Exception When the property is not an PropertyObjArray.
 * @see PropertyObjArray
 */
void Property::
setValue(int aSize,Object **aArray)
{
	string msg;
	msg = "Property.setValue(int,Object**): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * This method does nothing; it is just a place holder for
 * PropertyObjArray::setValue, which is the only intended meaningful
 * implementation of this method.
 *
 * This method has been implemented in Property, rather than made pure
 * virtual, so that all classes derived from Property will not have to
 * implement this method.
 *
 * @param aArray Array to which this property is to be assigned.
 * @throws Exception When the property is not an PropertyObjArray.
 * @see PropertyObjArray
 */
void Property::
setValue(const ArrayPtrs<Object> &aArray)
{
	string msg;
	msg = "Property.getValue(const ArrayPtrs<Object>&):";
	msg += "Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 * @throws Exception When the property is not an PropertyObjArray.
 * @see PropertyObjArray
 */
ArrayPtrs<Object>& Property::
getValueObjArray()
{
	string msg;
	msg = "Property.getValueObjArray(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Constant reference to the value of this property.
 * @throws Exception When the property is not an PropertyObjArray.
 * @see PropertyObjArray
 */
const ArrayPtrs<Object>& Property::
getValueObjArray() const
{
	string msg;
	msg = "Property.getValueObjArray(): Property type mismatch. ";
	msg += "This property is of type ";
	msg += getTypeAsString();
	msg += ".";

	throw Exception(msg,__FILE__,__LINE__);
}

//-----------------------------------------------------------------------------
// USE DEFAULT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not this property uses a "default" property for its value.
 *
 * True means that this property should have the same value as
 * some default property.  False means that the value of this property is
 * independent of the value of any other property.
 *
 * @param aTrueFalse True means this property uses another property for its
 * value.  False means this property does not use another property for its
 * value.
 */
void Property::
setUseDefault(bool aTrueFalse)
{
	_useDefault = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not this property uses a "default" property for its value.
 *
 * True means that this property should have the same value as
 * some default property.  False means that the value of this property is
 * independent of the value of any other property.
 *
 * @return True if this property uses another property for its value; false if
 * this property does not use another property for its value.
 */
bool Property::
getUseDefault() const
{
	return(_useDefault);
}



