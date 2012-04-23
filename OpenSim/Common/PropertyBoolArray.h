#ifndef _PropertyBoolArray_h_
#define _PropertyBoolArray_h_
// PropertyBoolArray.h
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

//=============================================================================
};	// END of class PropertyBoolArray

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyBoolArray_h__
