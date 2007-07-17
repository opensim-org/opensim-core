#ifndef _PropertyStr_h_
#define _PropertyStr_h_
// PropertyStr.h
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
#include "osimCommonDLL.h"
#include <string>
#include "Property.h"


//=============================================================================
//=============================================================================
/**
 * Class PropertyInt extends class Property.  It consists of a string
 * value and the methods for accessing and modifying this value.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API PropertyStr : public Property
{
	static const std::string DefaultValue;

//=============================================================================
// DATA
//=============================================================================
private:
	/** Value. */
	std::string _value;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyStr();
	PropertyStr(const std::string &aName,
		const std::string &aValue);
	PropertyStr(const PropertyStr &aProperty);
	virtual Property* copy() const;
	virtual ~PropertyStr() { };

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	PropertyStr& operator=(const PropertyStr &aProperty);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// TYPE
	virtual const char* getTypeAsString() const;
	// VALUE
	virtual void setValue(const std::string &aValue);
#ifndef SWIG
	virtual std::string& getValueStr();
#endif
	virtual const std::string& getValueStr() const;
	// VALUE as String
	virtual const std::string &toString();

	// Special method to reset the value
	void clearValue() { _value = DefaultValue; setUseDefault(true); }
	static const std::string& getDefaultStr() { return DefaultValue; }

//=============================================================================
};	// END of class PropertyStr

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyStr_h__
