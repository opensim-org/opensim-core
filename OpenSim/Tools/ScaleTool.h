#ifndef __ScaleTool_h__
#define __ScaleTool_h__

// ScaleTool.h
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
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "GenericModelMaker.h"
#include "ModelScaler.h"
#include "MarkerPlacer.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how to scale a model
 * to fit a subject, place markers on it, and do IK on one or more motion
 * trials.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMTOOLS_API ScaleTool : public Object  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _massProp;
	double &_mass;

	PropertyDbl _heightProp;
	double &_height;

	PropertyDbl _ageProp;
	double &_age;

	PropertyStr _notesProp;
	std::string &_notes;

	PropertyObj _genericModelMakerProp;
	GenericModelMaker &_genericModelMaker;

	PropertyObj _modelScalerProp;
	ModelScaler &_modelScaler;

	PropertyObj _markerPlacerProp;
	MarkerPlacer &_markerPlacer;

	/** All files in workflow are specified relative to
	 * where the subject file is. Need to keep track of that in case absolute
	 * path is needed later
	 */
	std::string	 _pathToSubject;	

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ScaleTool();
	ScaleTool(const std::string &aFileName);
	ScaleTool(const ScaleTool &aSubject);
	virtual ~ScaleTool();
	virtual Object* copy() const;

#ifndef SWIG
	ScaleTool& operator=(const ScaleTool &aSubject);
#endif
	void copyData(const ScaleTool &aSubject);

	Model* createModel();
	/* Query the subject for different parameters */
	GenericModelMaker& getGenericModelMaker()
	{
		return _genericModelMaker;
	}

	ModelScaler& getModelScaler()
	{
		return _modelScaler;
	}

	MarkerPlacer& getMarkerPlacer()
	{
		return _markerPlacer;
	}

	bool isDefaultGenericModelMaker() { return _genericModelMakerProp.getUseDefault(); }
	bool isDefaultModelScaler() { return _modelScalerProp.getUseDefault(); }
	bool isDefaultMarkerPlacer() { return _markerPlacerProp.getUseDefault(); }

	/* Register types to be used when reading a ScaleTool object from xml file. */
	static void registerTypes();

	/** Accessor methods to obtain model attributes */
	double getSubjectMass() { return _mass; }
	double getSubjectAge() { return _age; }
	double getSubjectHeight() { return _height; }
	void setSubjectMass(double mass) { _mass = mass; }
	void setSubjectAge(double age) { _age = age; }
	void setSubjectHeight(double height) { _height = height; }
	/**
	 * Accessor methods to set and get path to Subject. This is needed
	 * since all file names referred to in the subject file are relative
	 * to subject file
	 */
	const std::string& getPathToSubject()
	{
		return _pathToSubject;
	}
	void setPathToSubject(const std::string& aPath)
	{
		_pathToSubject=aPath;
	}
	//std::string getParentDirectory(const std::string& fileName);

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class ScaleTool
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ScaleTool_h__
