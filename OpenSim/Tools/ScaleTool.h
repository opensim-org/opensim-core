#ifndef __ScaleTool_h__
#define __ScaleTool_h__

// ScaleTool.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
	ScaleTool(const std::string &aFileName) SWIG_DECLARE_EXCEPTION;
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
