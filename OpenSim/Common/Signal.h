#ifndef _Signal_h_
#define _Signal_h_
// Signal.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
