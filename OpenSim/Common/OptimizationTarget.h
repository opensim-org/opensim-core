#ifndef _OptimizationTarget_h_
#define _OptimizationTarget_h_
// OptimizationTarget.cpp
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "osimCommonDLL.h"
#include "Array.h"
#include <simmath/Optimizer.h>


namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * This class provides an interface specification for optimizing redundant
 * systems.  If a class represents a redundant system for which one would
 * like to find a set of optimal controls, the class should inherit from
 * this class and implement the virtual functions defined here.
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API OptimizationTarget : public SimTK::OptimizerSystem
{
//=============================================================================
// DATA
//=============================================================================
public:
	/** Smallest allowable perturbation size for computing derivatives. */
	static const double SMALLDX;
protected:
	/** Perturbation size for computing numerical derivatives. */
	Array<double> _dx;

//=============================================================================
// METHODS
//=============================================================================
public:
	OptimizationTarget(int aNX=0);

	// SET AND GET
	void setNumParameters(const int aNX); // OptimizerSystem function
	void setDX(double aVal);
	void setDX(int aIndex,double aVal);
	double getDX(int aIndex);
	double* getDXArray();

	// UTILITY
	void validatePerturbationSize(double &aSize);

	virtual bool prepareToOptimize(const SimTK::State& s, double *x) { return false; }
	virtual void printPerformance(double *x);

	static int
		CentralDifferencesConstraint(const OptimizationTarget *aTarget,
		double *dx,const SimTK::Vector &x,SimTK::Matrix &jacobian);
	static int
		CentralDifferences(const OptimizationTarget *aTarget,
		double *dx,const SimTK::Vector &x,SimTK::Vector &dpdx);
	static int
		ForwardDifferences(const OptimizationTarget *aTarget,
		double *dx,const SimTK::Vector &x,SimTK::Vector &dpdx);

};

}; //namespace

#endif // _OptimizationTarget_h_
