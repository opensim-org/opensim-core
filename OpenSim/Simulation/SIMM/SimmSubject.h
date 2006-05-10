#ifndef _SimmSubject_h_
#define _SimmSubject_h_

// SimmSubject.h
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
#include <OpenSim/Tools/PropertyObj.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmGenericModelParams.h>
#include <OpenSim/Simulation/SIMM/SimmScalingParams.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerPlacementParams.h>
#include <OpenSim/Simulation/SIMM/SimmIKParams.h>

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
namespace OpenSim { 

class RDSIMULATION_API SimmSubject : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:
	// SimmModel* _model;

protected:

	PropertyDbl _massProp;
	double &_mass;

	PropertyDbl _heightProp;
	double &_height;

	PropertyDbl _ageProp;
	double &_age;

	PropertyStr _notesProp;
	std::string &_notes;

	PropertyObj _genericModelParamsProp;
	SimmGenericModelParams &_genericModelParams;

	PropertyObj _scalingParamsProp;
	SimmScalingParams &_scalingParams;

	PropertyObj _markerPlacementParamsProp;
	SimmMarkerPlacementParams &_markerPlacementParams;

	PropertyObj _IKParamsProp;
	SimmIKParams &_IKParams;

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
    void SimmSubject::copyData(const SimmSubject &aSubject);

	SimmModel* createModel();
	/* Query the subject for different parameters */
	SimmGenericModelParams& getGenericModelParams()
	{
		return _genericModelParams;
	}

	SimmScalingParams& getScalingParams()
	{
		return _scalingParams;
	}

	SimmMarkerPlacementParams& getMarkerPlacementParams()
	{
		return _markerPlacementParams;
	}

	SimmIKParams& getIKParams()
	{
		return _IKParams;
	}

	double getMass() const { return _mass; }

	bool isDefaultGenericModelParams() { return _genericModelParamsProp.getUseDefault(); }
	bool isDefaultScalingParams() { return _scalingParamsProp.getUseDefault(); }
	bool isDefaultMarkerPlacementParams() const;
	bool isDefaultIKParams() { return _IKParamsProp.getUseDefault(); }
	/* Register types to be used when reading a SimmSubject object from xml file. */
	static void registerTypes();

	void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
public:
	/** Default value for subject mass if not specified in file */
	static const double DefaultMass;
//=============================================================================
};	// END of class SimmSubject

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmSubject_h__


