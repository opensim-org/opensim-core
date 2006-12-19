#ifndef __SimmSubject_h__
#define __SimmSubject_h__

// SimmSubject.h
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
#include <OpenSim/Tools/PropertyObj.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Subject/SimmGenericModelMaker.h>
#include <OpenSim/Subject/SimmModelScaler.h>
#include <OpenSim/Subject/SimmMarkerPlacer.h>

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
class workflow_API SimmSubject : public Object  
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
	SimmGenericModelMaker &_genericModelMaker;

	PropertyObj _modelScalerProp;
	SimmModelScaler &_modelScaler;

	PropertyObj _markerPlacerProp;
	SimmMarkerPlacer &_markerPlacer;

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
	SimmSubject();
	SimmSubject(const std::string &aFileName);
	SimmSubject(DOMElement *aElement);
	SimmSubject(const SimmSubject &aSubject);
	virtual ~SimmSubject();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmSubject& operator=(const SimmSubject &aSubject);
#endif
	void copyData(const SimmSubject &aSubject);

	bool processModel();
	AbstractModel* createModel();
	/* Query the subject for different parameters */
	SimmGenericModelMaker& getGenericModelMaker()
	{
		return _genericModelMaker;
	}

	SimmModelScaler& getModelScaler()
	{
		return _modelScaler;
	}

	SimmMarkerPlacer& getMarkerPlacer()
	{
		return _markerPlacer;
	}

	double getMass() const { return _mass; }

	bool isDefaultGenericModelMaker() { return _genericModelMakerProp.getUseDefault(); }
	bool isDefaultModelScaler() { return _modelScalerProp.getUseDefault(); }
	bool isDefaultMarkerPlacer() { return _markerPlacerProp.getUseDefault(); }
	/* Register types to be used when reading a SimmSubject object from xml file. */
	static void registerTypes();

	/** Accessor methods to obtain model attributes */
	const double& getSubjectMass() { return _mass; }
	const double& getSubjectAge() { return _age; }
	const double& getSubjectHeight() { return _height; }
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

	void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmSubject
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmSubject_h__
