#ifndef _SimmGenericModelParams_h_
#define _SimmGenericModelParams_h_

// SimmGenericModelParams.h
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/XMLDocument.h>
#include "SimmModel.h"
#include "SimmMarker.h"

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing
 * a generic musculoskeletal model.
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class RDSIMULATION_API SimmGenericModelParams : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyStr _fileNameProp;
	std::string &_fileName;

	PropertyObjArray _markerSetProp;
	ArrayPtrs<SimmMarker> &_markerSet;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmGenericModelParams();
	SimmGenericModelParams(DOMElement *aElement);
	SimmGenericModelParams(const SimmGenericModelParams &aGenericModelParams);
	virtual ~SimmGenericModelParams();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmGenericModelParams& operator=(const SimmGenericModelParams &aGenericModelParams);
#endif
	void copyData(const SimmGenericModelParams &aGenericModelParams);

	SimmModel* processModel();

	/* Register types to be used when reading a SimmGenericModelParams object from xml file. */
	static void registerTypes();

	void peteTest() const;
	/** 
	 * Manually add a marker to the markerSet 
	 */
	void addMarker (SimmMarker *aSimmMarker) 
	{
		_markerSet.append(aSimmMarker);
	};

	/**
	 * Get file name for generic model
	 */
	const std::string& getModelFileName() const
	{
		return _fileName;
	};
protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmGenericModelParams

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmGenericModelParams_h__


