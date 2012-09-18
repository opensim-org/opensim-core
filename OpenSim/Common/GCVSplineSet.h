#ifndef _GCVSplineSet_h_
#define _GCVSplineSet_h_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  GCVSplineSet.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */



// INCLUDES
#include "osimCommonDLL.h"
#include "Object.h"
#include "FunctionSet.h"
#include "GCVSpline.h"
#include "Storage.h"


//=============================================================================
//=============================================================================
/**
 * A class for holding a set of generalized cross-validated splines.
 *
 * @see GCVSpline
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API GCVSplineSet : public FunctionSet {
OpenSim_DECLARE_CONCRETE_OBJECT(GCVSplineSet, FunctionSet);

//=============================================================================
// DATA
//=============================================================================
protected:

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	GCVSplineSet();
	GCVSplineSet(const char *aFileName);
	GCVSplineSet(int aDegree,const Storage *aStore,double aErrorVariance=0.0);
	virtual ~GCVSplineSet();

private:
	void setNull();
	void construct(int aDegree,const Storage *aStore,double aErrorVariance);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	GCVSpline* getGCVSpline(int aIndex) const;
	double getMinX() const;
	double getMaxX() const;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	Storage* constructStorage(int aDerivOrder,double aDX=-1);

//=============================================================================
};	// END class GCVSplineSet

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __GCVSplineSet_h__
