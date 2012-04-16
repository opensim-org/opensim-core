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

#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/VectorFunctionUncoupledNxN.h>
#include <OpenSim/Simulation/Model/CMCActuatorSubsystem.h>

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
 * -SimTK::Infinity and SimTK::Infinity, or it returns SimTK::NaN
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class VectorFunctionForActuators : public VectorFunctionUncoupledNxN {
OpenSim_DECLARE_CONCRETE_OBJECT(VectorFunctionForActuators, 
                                VectorFunctionUncoupledNxN);

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
	/** Actuator System  */
	SimTK::System* _CMCActuatorSystem;
	/** Actuator SubSystem  */
	CMCActuatorSubsystem* _CMCActuatorSubsystem;
	/** Integrator. */
	SimTK::Integrator* _integrator;
    /** Model */
    Model* _model;


//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	VectorFunctionForActuators(SimTK::System *aActuatorSystem, Model *model, CMCActuatorSubsystem* actSubsys);
	VectorFunctionForActuators(const VectorFunctionForActuators &aFunction);
	VectorFunctionForActuators();
	virtual ~VectorFunctionForActuators();

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
	CMCActuatorSubsystem* getCMCActSubsys();

	
	//--------------------------------------------------------------------------
	// EVALUATE
	//--------------------------------------------------------------------------

	virtual void calcValue( const double *aX, double *rF, int aSize) {
		std::cout << "Unimplemented evaluate method" << std::endl; 
//		exit(0);
	}
	virtual void calcValue( const Array<double> &aX, Array<double> &rF) {
		std::cout << "Unimplemented evaluate method" << std::endl; 
//		exit(0);
	}
	virtual void calcValue( const Array<double> &aX, Array<double> &rF, const Array<int> &aDerivWRT) {
		std::cout << "Unimplemented evaluate method" << std::endl; 
//		exit(0);
	}
	virtual void calcDerivative(const Array<double> &aX,Array<double> &rY,
		const Array<int> &aDerivWRT) {
		std::cout << "Unimplemented calcDerivative method" << std::endl; 
	}

	virtual void evaluate( const SimTK::State& s,  double *aX, double *rF);
	virtual void evaluate( const SimTK::State& s,  const OpenSim::Array<double> &aX, Array<double> &rF);
	virtual void evaluate( const SimTK::State& s,  Array<double> &rF, const Array<int> &aDerivWRT);
    virtual void evaluate(const double *rY){}
    virtual void evaluate(const Array<double> &rY){}
    virtual void evaluate(Array<double> &rY, const Array<int> &aDerivWRT){}


//=============================================================================
};	// END class VectorFunctionForActuators

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __VectorFunctionForActuators_h__
