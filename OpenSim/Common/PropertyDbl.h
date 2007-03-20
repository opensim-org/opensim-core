#ifndef _PropertyDbl_h_
#define _PropertyDbl_h_
// PropertyDbl.h
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
 * Class PropertyDbl extends class Property.  It consists of a double
 * value and the methods for accessing and modifying this value.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API PropertyDbl : public Property
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Value. */
	double _value;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyDbl();
	PropertyDbl(const std::string &aName,double aValue);
	PropertyDbl(const PropertyDbl &aProperty);
	virtual Property* copy() const;
	virtual ~PropertyDbl() { };

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	PropertyDbl& operator=(const PropertyDbl &aProperty);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// TYPE
	virtual const char* getTypeAsString() const;
	// VALUE
	virtual void setValue(double aValue);
	virtual double& getValueDbl();
	virtual const double& getValueDbl() const;
	// VALUE as String
	virtual const std::string &toString();

//=============================================================================
};	// END of class PropertyDbl

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyDbl_h__
