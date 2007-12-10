#ifndef _VectorFunctionForActuators_h_
#define _VectorFunctionForActuators_h_
// VectorFunctionForActuators.cpp
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

/*  
 * Author: Frank C. Anderson 
 */

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/VectorFunctionUncoupledNxN.h>
#include <OpenSim/Simulation/Integrator/IntegRKF.h>
#include "ModelIntegrand.h"

//extern template class RDTOOLS_API Array<double>;

//=============================================================================
//=============================================================================
/**
 * An abstract class for representing a vector function.
 *
 * A vector function is a relation between some number of independent variables 
 * and some number of dependent values such that for any particular set of
 * independent variables the correct number of dependent variables is returned.
 * Values of the function and its derivatives
 * are obtained by calling the evaluate() method.  The curve may or may not
 * be finite or diferentiable; the evaluate method returns values between
 * rdMath::MINUS_INFINITY and rdMath::PLUS_INFINITY, or it returns rdMath::NAN
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMSIMULATION_API VectorFunctionForActuators :
	public VectorFunctionUncoupledNxN
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Initial time for the integration. */
	double _ti;
	/** Final time for the integration. */
	double _tf;
	/** Target actuator forces. */
	Array<double> _f;
	/** Integrand. */
	ModelIntegrand *_integrand;
	/** Integrator. */
	IntegRKF *_integrator;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	VectorFunctionForActuators(ModelIntegrand *aIntegrand);
	VectorFunctionForActuators(const VectorFunctionForActuators &aFunction);
	virtual ~VectorFunctionForActuators();
	virtual Object* copy() const;
private:
	void setNull();
	void setEqual(const VectorFunctionForActuators &aVectorFunction);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	VectorFunctionForActuators&
		operator=(const VectorFunctionForActuators &aFunction);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
	void setInitialTime(double aTI);
	double getInitialTime() const;
	void setFinalTime(double aTF);
	double getFinalTime() const;
	void setTargetForces(const double *aF);
	void getTargetForces(double *rF) const;
	ModelIntegrand* getIntegrand();

	
	//--------------------------------------------------------------------------
	// EVALUATE
	//--------------------------------------------------------------------------
	virtual void evaluate(const double *aX,double *aY);
	virtual void evaluate(const Array<double> &aX,Array<double> &rY);
	virtual void evaluate(const Array<double> &aX,Array<double> &rY,
		const Array<int> &aDerivWRT);

//=============================================================================
};	// END class VectorFunctionForActuators

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __VectorFunctionForActuators_h__
