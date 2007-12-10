#ifndef _Springs_h_
#define _Springs_h_
// Springs.h
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

//INCLUDES
#include "osimActuatorsDLL.h"

#ifdef SWIG
	#ifdef OSIMACTUATORS_API
		#undef OSIMACTUATORS_API
		#define OSIMACTUATORS_API
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

class OSIMACTUATORS_API Springs  
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
