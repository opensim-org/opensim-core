#ifndef _PropertyObj_h_
#define _PropertyObj_h_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PropertyObj.h                           *
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
#include "Object.h"
#include "Property_Deprecated.h"


//=============================================================================
//=============================================================================
/**
 * Class PropertyObj extends class Property.  It consists of a pointer to
 * an object and the methods for accessing and modifying this object.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API PropertyObj : public Property_Deprecated
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Value. */
	Object *_value;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyObj();
	PropertyObj(const std::string &aName,const Object &aValue);
	PropertyObj(const PropertyObj &aProperty);

	PropertyObj* clone() const OVERRIDE_11;
	virtual ~PropertyObj();

    virtual bool isObjectProperty() const OVERRIDE_11 {return true;}
    virtual bool isAcceptableObjectTag
        (const std::string& objectTypeTag) const OVERRIDE_11 {return true;}
    virtual const Object& getValueAsObject(int index=-1) const OVERRIDE_11
    {   assert(index <= 0); return getValueObj(); }
    virtual Object& updValueAsObject(int index=-1) OVERRIDE_11
    {   assert(index <= 0); return getValueObj(); }
    virtual void setValueAsObject(const Object& obj, int index=-1) OVERRIDE_11
    {   assert(index <= 0); delete _value; _value=obj.clone(); }

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	PropertyObj& operator=(const PropertyObj &aProperty);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	virtual bool isValidObject(const Object *obj) const { return true; } // TODO: make this class templated and do type checking
	// TYPE
	virtual std::string getTypeName() const OVERRIDE_11 
    {   return "Object"; }
	// VALUE
	// Got rid of setValue(Obj) since it would be dangerous to do so given that users of
	// PropertyObj would continue to hold a reference to the (deleted) object - Eran.
	virtual Object& getValueObj();
	virtual const Object& getValueObj() const;
	// VALUE as String
	virtual std::string toString() const;

//=============================================================================
};	// END of class PropertyObj

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyObj_h__
