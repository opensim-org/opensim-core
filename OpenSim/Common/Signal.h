#ifndef _Signal_h_
#define _Signal_h_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Signal.h                             *
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


#include "osimCommonDLL.h"
#include "Array.h"



namespace OpenSim { 
//=============================================================================
//=============================================================================
/**
 * A class for signal processing.
 */
class OSIMCOMMON_API Signal
{
//=============================================================================
// DATA
//=============================================================================

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	//Signal();
	//virtual ~Signal();

	//--------------------------------------------------------------------------
	// FILTERS
	//--------------------------------------------------------------------------
	static int
		SmoothSpline(int aDegree,double aDeltaT,double aCutOffFrequency,
		int aN,double *aTimes,double *aSignal,double *rFilteredSignal);
	static int
		LowpassIIR(double aDeltaT,double aCutOffFrequency,
		int aN,double *aSignal,double *rFilteredSignal);
	static int
		LowpassFIR(int aOrder,double aDeltaT,double aCutoffFrequency,
		int aN,double *aSignal,double *rFilteredSignal);
	static int
		BandpassFIR(int aOrder,double aDeltaT,
		double aLowFrequency,double aHighFrequency,
		int aN,double *aSignal,double *aFilteredSignal);

	//--------------------------------------------------------------------------
	// PADDING
	//--------------------------------------------------------------------------
	static double*
		Pad(int aPad,int aN,const double aSignal[]);
	static void
		Pad(int aPad,OpenSim::Array<double> &aSignal);

	//--------------------------------------------------------------------------
	// POINT REDUCTION
	//--------------------------------------------------------------------------
	static int
		ReduceNumberOfPoints(double aDistance,
		Array<double> &rTime,Array<double> &rSignal);


	//--------------------------------------------------------------------------
	// CORE MATH
	//--------------------------------------------------------------------------
	static double sinc(double x);
	static double hamming(int k,int M);

//=============================================================================
};	// END class Signal

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Signal_h__
