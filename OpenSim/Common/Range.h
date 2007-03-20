#ifndef _Range_h_
#define _Range_h_
// Range.h
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

/*  
 * Author:  
 */
#include "osimCommonDLL.h"
#include "Object.h"
#include "PropertyDbl.h"

//=============================================================================
/*
 * A Class representing a range for a parameter or a generalized coordinate
 *
 * @author Ayman Habib
 * @version 1.0
 */
namespace OpenSim { 

class OSIMCOMMON_API Range : public Object
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Min value */
	PropertyDbl		_propMin;
	/** Max value */
	PropertyDbl		_propMax;

	// REFERENCES
	double&				_min;
	double&				_max;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Range();
	Range(const Range &aRange);
	virtual ~Range(void);
	virtual Object* copy() const;

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	Range& operator=(const Range &aRange);
#endif	
private:
	void setNull();
	void setupProperties();

public:
	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
	const double getMin() const;
	void setMin(const double aMin);

	const double getMax() const;
	void setMax(const double aMax);


};

}; //namespace
#endif
