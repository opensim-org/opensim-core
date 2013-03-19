#ifndef _PropertyBoolArray_h_
#define _PropertyBoolArray_h_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  PropertyBoolArray.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "Property_Deprecated.h"
#ifdef WIN32
#pragma warning( disable : 4251 )
#endif

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * Class PropertyBoolArray extends class Property.  It consists of an
 * array of booleans (i.e., Array<bool>) and the methods for accessing
 * and modifying this array.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API PropertyBoolArray : public Property_Deprecated
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Array of booleans. */
	Array<bool> _array;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyBoolArray();
	PropertyBoolArray(const std::string &aName,
		const Array<bool> &aArray);
	PropertyBoolArray(const std::string &aName,
		int aSize,const bool aArray[]);
	PropertyBoolArray(const PropertyBoolArray &aProperty);
	
    bool isArrayProperty() const OVERRIDE_11 {return true;}
    
    PropertyBoolArray* clone() const OVERRIDE_11 ;

    int getNumValues() const OVERRIDE_11 {return getArraySize();}

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	PropertyBoolArray& operator=(const PropertyBoolArray &aProperty);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// TYPE
	virtual std::string getTypeName() const OVERRIDE_11;
	// VALUE
	virtual void setValue(const Array<bool> &aArray);
	virtual void setValue(int aSize,const bool aArray[]);
	virtual Array<bool>& getValueBoolArray();
	virtual const Array<bool>& getValueBoolArray() const;
	// SIZE
	virtual int getArraySize() const { return _array.getSize(); }
	// VALUE as String
	virtual std::string toString() const;
   virtual void clearValues() OVERRIDE_11 { _array.setSize(0); }

//=============================================================================
};	// END of class PropertyBoolArray

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyBoolArray_h__
