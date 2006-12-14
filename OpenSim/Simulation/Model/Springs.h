#ifndef _Springs_h_
#define _Springs_h_
// Springs.h
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

//INCLUDES
#include <OpenSim/Simulation/rdSimulationDLL.h>

#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * A class which implements a set of static functions which model
 * the stress-strain properties of a number of different types of springs.
 *
 */
namespace OpenSim { 

class RDSIMULATION_API Springs  
{

//=============================================================================
// DATA
//=============================================================================
private:

//=============================================================================
// METHODS
//=============================================================================
public:
	Springs();
	virtual ~Springs();

	//--------------------------------------------------------------------------
	// SPRINGS
	//--------------------------------------------------------------------------
	static double Damp(double kv,double v);
	static double Linear(double kx,double dx);
	static double Linear(double kx,double x0,double x);
	static double DampedLinear(double kv,double v,double kx,double x0,double x);
	static double ZeroForDampedLinear(double kv,double v,double kx,double x,
						double f);
	static double DisplacementOfDampedLinear(double kv,double v,double kx,double f);
	static double Quadratic(double kx,double dx);
	static double Quadratic(double kx,double x0,double x);
	static double DampedQuadratic(double kv,double v,double kx,double dx);
	static double DampedQuadratic(double kv,double v,double kx,double x0,double x);
	static double ExponentialBarrier(double v,double dx);
	static double ExponentialBarrier(double aG0,double aG1,double aG2,
						double aE0,double aE1,double aE2,
						double aKV,double aV,double aDX);
	static double ExponentialBarrierDX(double v,double dx);
	static double ExponentialBarrierDX(double aG0,double aG1,double aG2,
						double aE0,double aE1,double aE2,
						double aKV,double aV,double aDX);
	static double ExponentialBarrierDV(double v,double dx);
	static double ExponentialBarrierDV(double aG0,double aG1,double aG2,
						double aE0,double aE1,double aE2,
						double aKV,double aV,double aDX);
	static double oscillation(double delay,double t,double f);

//=============================================================================
};	// END class Springs

}; //namespace
//=============================================================================
//=============================================================================


#endif //__Springs_h__
