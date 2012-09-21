#ifndef _PropertyInt_h_
#define _PropertyInt_h_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PropertyInt.h                           *
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


//=============================================================================
//=============================================================================
/**
 * Class PropertyInt extends class Property.  It consists of an integer
 * value and the methods for accessing and modifying this value.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API PropertyInt : public Property_Deprecated
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Value. */
	int _value;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyInt();
	PropertyInt(const std::string &aName,int aValue);
	PropertyInt(const PropertyInt &aProperty);
	/*virtual*/ PropertyInt* clone() const;

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	PropertyInt& operator=(const PropertyInt &aProperty);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// TYPE
	virtual std::string getTypeName() const OVERRIDE_11;
	// VALUE
	virtual void setValue(int aValue);
	virtual int& getValueInt();
	virtual const int& getValueInt() const;
	// VALUE as String
	virtual std::string toString() const;

//=============================================================================
};	// END of class PropertyInt

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyInt_h__
