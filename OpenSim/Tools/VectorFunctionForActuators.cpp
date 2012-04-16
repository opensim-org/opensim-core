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

// INCLUDES
#include "VectorFunctionForActuators.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Model/ControllerSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/CMCActuatorSubsystem.h>
#include "CMC.h"

#include "SimTKsimbody.h"

using namespace OpenSim;
using namespace std;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
VectorFunctionForActuators::~VectorFunctionForActuators()
{
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
VectorFunctionForActuators::
VectorFunctionForActuators(SimTK::System *aActuatorSystem, Model* model, CMCActuatorSubsystem* actSubsystem) :
	VectorFunctionUncoupledNxN(model->getActuators().getSize() ),
	_f(0.0)
{
	setNull();
    _model = model;
	_CMCActuatorSubsystem = actSubsystem;
	_CMCActuatorSystem = aActuatorSystem;
	_integrator = new SimTK::RungeKuttaMersonIntegrator(*aActuatorSystem);
    _integrator->setAccuracy( 5.0e-6 );
    _integrator->setMaximumStepSize(1.0e-3);

    // Don't project constraints while inside the controller
    _integrator->setProjectInterpolatedStates( false );
	_f.setSize(getNX());
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aVectorFunction Function to copy.
 */
VectorFunctionForActuators::
VectorFunctionForActuators(const VectorFunctionForActuators &aVectorFunction) :
	VectorFunctionUncoupledNxN(aVectorFunction),
	_f(0.0)
{
	setNull();

	// ASSIGN
	setEqual(aVectorFunction);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void VectorFunctionForActuators::
setNull()
{
	_ti = 0.0;
	_tf = 0.0;
	_CMCActuatorSystem    = NULL;
	_CMCActuatorSubsystem = NULL;
    _model             = NULL;
	_integrator        = NULL;
}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 */
void VectorFunctionForActuators::
setEqual(const VectorFunctionForActuators &aVectorFunction)
{
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
VectorFunctionForActuators& VectorFunctionForActuators::
operator=(const VectorFunctionForActuators &aVectorFunction)
{
	// BASE CLASS
	VectorFunctionUncoupledNxN::operator=(aVectorFunction);

	// DATA
	setEqual(aVectorFunction);

	return(*this);
}


//=============================================================================
// SET & GET
//=============================================================================
//-----------------------------------------------------------------------------
// INTIAL TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the initial time of the simulation.
 *
 * @param aTI Initial time.
 */
void VectorFunctionForActuators::
setInitialTime(double aTI)
{
	_ti = aTI;
}
//_____________________________________________________________________________
/**
 * Get the initial time of the simulation.
 *
 * @return Initial time.
 */
double VectorFunctionForActuators::
getInitialTime() const
{
	return(_ti);
}

//-----------------------------------------------------------------------------
// FINAL TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the final time of the simulation.
 *
 * @param aTF Final time.
 */
void VectorFunctionForActuators::
setFinalTime(double aTF)
{
	_tf = aTF;
}
//_____________________________________________________________________________
/**
 * Get the final time of the simulation.
 *
 * @return Final time.
 */
double VectorFunctionForActuators::
getFinalTime() const
{
	return(_tf);
}

//-----------------------------------------------------------------------------
// TARGET FORCES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the target actuator forces.
 *
 * @param aF Array of target forces.
 */
void VectorFunctionForActuators::
setTargetForces(const double *aF)
{
	int i,N=getNX();
	for(i=0;i<N;i++) _f[i] = aF[i];
}
//_____________________________________________________________________________
/**
 * Get the target actuator forces.
 *
 * @param rF Array of target forces.
 */
void VectorFunctionForActuators::
getTargetForces(double *rF) const
{
	int i,N=getNX();
	for(i=0;i<N;i++) rF[i] = _f[i];
}


//-----------------------------------------------------------------------------
// INTEGRAND
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the Actuator subsystem.
 *
 * @return Actuator Subsystem.
 */
CMCActuatorSubsystem* VectorFunctionForActuators::
getCMCActSubsys()
{
	return(_CMCActuatorSubsystem);
}



//=============================================================================
// EVALUATE
//=============================================================================
//_____________________________________________________________________________
/**
 * Evaluate the vector function.
 *
 * @param s SimTK::State.
 * @param aF Array of actuator force differences.
 */
void VectorFunctionForActuators::
evaluate( const SimTK::State& s, double *aX, double *rF) 
{
	int i;
	int N = getNX();

    CMC& controller=  dynamic_cast<CMC&>(_model->updControllerSet().get(_model->getControllerSet().getIndex("CMC" )));
    controller.updControlSet().setControlValues(_tf, aX);

    // create a Manager that will integrate just the actuator subsystem and use only the 
    // CMC controller

	Manager manager(*_model, *_integrator);
    manager.setInitialTime(_ti);
	manager.setFinalTime(_tf);
    manager.setSystem( _CMCActuatorSystem );
    // tell the mangager to not call the analayses or write to storage 
    // while the CMCSubsystem is being integrated.
	manager.setPerformAnalyses(false); 
	manager.setWriteToStorage(false); 
	SimTK::State& actSysState = _CMCActuatorSystem->updDefaultState();
	getCMCActSubsys()->updZ(actSysState) = _model->getForceSubsystem().getZ(s);

    actSysState.setTime(_ti);

	// Integration
    manager.integrate(actSysState, 0.000001);

    const Set<Actuator>& forceSet = _model->getActuators();
	// Vector function values
	int j = 0;
	for(i=0;i<N;i++) {
        Actuator& act = forceSet.get(i); 
	    rF[j] = act.getForce(getCMCActSubsys()->getCompleteState()) - _f[j];
        j++;
	}


}
//_____________________________________________________________________________
/**
 * Evaluate the vector function.
 *
 * @param s SimTK::State 
 * @param aF Array of actuator force differences.
 */
void VectorFunctionForActuators::
evaluate(const SimTK::State& s,  OpenSim::Array<double> &rF,
			const OpenSim::Array<int> &aDerivWRT)
{
	cout<<"\n\nVectorFunctionForActuators.evaluate:  ";
	cout<<"Unimplemented method\n\n";
}

//_____________________________________________________________________________
/**
 * Evaluate the vector function.
 *
 * @param aX Array of controls.
 * @param aF Array of actuator force differences.
 */
void VectorFunctionForActuators::
evaluate( const SimTK::State& s,  const OpenSim::Array<double> &aX, OpenSim::Array<double> &rF)
{
	evaluate( s, &aX[0],&rF[0]);
}
