#ifndef __SimmGenericModelMaker_h__
#define __SimmGenericModelMaker_h__

// SimmGenericModelMaker.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
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
#include <OpenSim/Applications/Workflow/workflowDLL.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyObj.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Simulation/SIMM/MarkerSet.h>

#ifdef SWIG
	#ifdef workflow_API
		#undef workflow_API
		#define workflow_API
	#endif
#endif

namespace OpenSim {

class AbstractModel;

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing
 * a generic musculoskeletal model.
 *
 * @author Peter Loan
 * @version 1.0
 */
class workflow_API SimmGenericModelMaker : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyStr _fileNameProp;
	std::string &_fileName;

	PropertyObj _markerSetProp;
	MarkerSet &_markerSet;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmGenericModelMaker();
	SimmGenericModelMaker(DOMElement *aElement);
	SimmGenericModelMaker(const SimmGenericModelMaker &aGenericModelMaker);
	virtual ~SimmGenericModelMaker();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmGenericModelMaker& operator=(const SimmGenericModelMaker &aGenericModelMaker);
#endif
	void copyData(const SimmGenericModelMaker &aGenericModelMaker);

	AbstractModel* processModel(const std::string& aPathToSubject="");

	/* Register types to be used when reading a SimmGenericModelMaker object from xml file. */
	static void registerTypes();

	void peteTest() const;

	/**
	 * Get file name for generic model
	 */
	const std::string& getModelFileName() const
	{
		return _fileName;
	}

	// Set model file name
	void setModelFileName(const std::string& aFileName)
	{
		_fileName = aFileName;
		_fileNameProp.setUseDefault(false);
	}

	MarkerSet& getMarkerSet()
	{
		return _markerSet;
	}

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmGenericModelMaker
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmGenericModelMaker_h__


