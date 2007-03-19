#ifndef __GenericModelMaker_h__
#define __GenericModelMaker_h__

// GenericModelMaker.h
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
#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
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
class OSIMTOOLS_API GenericModelMaker : public Object  
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
	GenericModelMaker();
	GenericModelMaker(const GenericModelMaker &aGenericModelMaker);
	virtual ~GenericModelMaker();
	virtual Object* copy() const;

#ifndef SWIG
	GenericModelMaker& operator=(const GenericModelMaker &aGenericModelMaker);
#endif
	void copyData(const GenericModelMaker &aGenericModelMaker);

	AbstractModel* processModel(const std::string& aPathToSubject="");

	/* Register types to be used when reading a GenericModelMaker object from xml file. */
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
};	// END of class GenericModelMaker
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __GenericModelMaker_h__


