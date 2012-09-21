#ifndef __ScaleTool_h__
#define __ScaleTool_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  ScaleTool.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


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
class OSIMTOOLS_API ScaleTool : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(ScaleTool, Object);

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
	ScaleTool(const std::string &aFileName) SWIG_DECLARE_EXCEPTION;
	ScaleTool(const ScaleTool &aSubject);
	virtual ~ScaleTool();

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

	bool isDefaultGenericModelMaker() { return _genericModelMakerProp.getValueIsDefault(); }
	bool isDefaultModelScaler() { return _modelScalerProp.getValueIsDefault(); }
	bool isDefaultMarkerPlacer() { return _markerPlacerProp.getValueIsDefault(); }

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
	
	void setPrintResultFiles(bool aToWrite) { 
		_modelScaler.setPrintResultFiles(aToWrite);
		_markerPlacer.setPrintResultFiles(aToWrite);
	}

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
