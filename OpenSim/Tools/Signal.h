#ifndef _Signal_h_
#define _Signal_h_
// Signal.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


#include "rdTools.h"
#include "Array.h"


//=============================================================================
//=============================================================================
/**
 * A class for signal processing.
 */
namespace OpenSim { 

class RDTOOLS_API Signal
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
