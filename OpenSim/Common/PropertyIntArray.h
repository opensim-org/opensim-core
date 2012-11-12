#ifndef _PropertyIntArray_h_
#define _PropertyIntArray_h_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PropertyIntArray.h                        *
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

//=============================================================================
//=============================================================================
/**
 * Class PropertyIntArray extends class Property.  It consists of an
 * array of integers (i.e., Array<int>) and the methods for accessing
 * and modifying this array.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API PropertyIntArray : public Property_Deprecated
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Array of integers. */
	Array<int> _array;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyIntArray();
	PropertyIntArray(const std::string &aName,
		const Array<int> &aArray);
	PropertyIntArray(const std::string &aName,
		int aSize,const int aArray[]);
	PropertyIntArray(const PropertyIntArray &aProperty);

    bool isArrayProperty() const OVERRIDE_11 {return true;}

	PropertyIntArray* clone() const OVERRIDE_11;

    int getNumValues() const OVERRIDE_11 {return getArraySize();}

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	PropertyIntArray& operator=(const PropertyIntArray &aProperty);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// TYPE
	virtual std::string getTypeName() const OVERRIDE_11;
	// VALUE
	virtual void setValue(const Array<int> &aArray);
	virtual void setValue(int aSize,const int aArray[]);
	virtual Array<int>& getValueIntArray();
	virtual const Array<int>& getValueIntArray() const;
	// SIZE
	virtual int getArraySize() const { return _array.getSize(); }
	// VALUE as String
	virtual std::string toString() const;
    virtual void clearValues() OVERRIDE_11 {
        _array.setSize(0);
    }

//=============================================================================
};	// END of class PropertyIntArray

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyIntArray_h__
