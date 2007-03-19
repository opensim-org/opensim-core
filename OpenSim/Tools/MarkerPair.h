#ifndef _MarkerPair_h_
#define _MarkerPair_h_

// MarkerPair.h
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


// INCLUDE
#include <iostream>
#include <math.h>
#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStrArray.h>

//=============================================================================
//=============================================================================
/**
 * A class for holding the names of a pair of markers (for making
 * measurements on a model).
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class OSIMTOOLS_API MarkerPair : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyStrArray _markerNamesProp;
	Array<std::string>& _markerNames;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MarkerPair();
	MarkerPair(const MarkerPair &aMarkerPair);
	virtual ~MarkerPair();
	virtual Object* copy() const;

#ifndef SWIG
	MarkerPair& operator=(const MarkerPair &aMarkerPair);
#endif
	void copyData(const MarkerPair &aMarkerPair);

	void getMarkerNames(std::string& aName1, std::string& aName2) const;

	void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class MarkerPair

}; //namespace
//=============================================================================
//=============================================================================

#endif // __MarkerPair_h__


