#ifndef _PropertyObj_h_
#define _PropertyObj_h_
// PropertyObj.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
