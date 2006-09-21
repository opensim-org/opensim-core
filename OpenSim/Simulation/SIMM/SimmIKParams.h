#ifndef _SimmIKParams_h_
#define _SimmIKParams_h_

// SimmIKParams.h
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
#include <string>
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyObj.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyStrArray.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>

#include "SimmCoordinateSet.h"
#include "SimmMarkerSet.h"
#include "SimmIKTrialParamsSet.h"

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how to perform
 * a group of inverse kinematics trials on a model and a marker set.
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class RDSIMULATION_API SimmIKParams : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	// name of model file to use for IK (optional)
	PropertyStr _modelFileNameProp;
	std::string &_modelFileName;

	// marker set for updating markers in model before doing IK
	PropertyObj _markerSetProp;
	SimmMarkerSet &_markerSet;

	// coordinate set for updating coordinates in model before doing IK
	PropertyObj _coordinateSetProp;
	SimmCoordinateSet &_coordinateSet;

	// parameters for the set of IK trials to perform
	PropertyObj _IKTrialParamsSetProp;
	SimmIKTrialParamsSet &_IKTrialParamsSet;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmIKParams();
	SimmIKParams(DOMElement *aElement);
	SimmIKParams(const SimmIKParams &aIKParams);
	virtual ~SimmIKParams();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmIKParams& operator=(const SimmIKParams &aIKParams);
#endif
   void SimmIKParams::copyData(const SimmIKParams &aIKParams);

	//bool solveIK(IKSolverInterface *aSolver,SimmModel* aModel);
   SimmMarkerSet& getMarkerSet()
   {
	   return _markerSet;
   }

	int getNumIKTrials()
	{
		return _IKTrialParamsSet.getSize();
	}

	SimmIKTrialParams& getTrialParams(const int aIndex)
	{
		return (*(_IKTrialParamsSet.get(aIndex)));
	}

	SimmCoordinateSet &getCoordinateSet() const
	{
		return (_coordinateSet);
	}

	const std::string& getModelFileName() const
	{
		return _modelFileName;
	}
	/* Register types to be used when reading a SimmIKParams object from xml file. */
	static void registerTypes();

	void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmIKParams

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmIKParams_h__


