/* -------------------------------------------------------------------------- *
 *                  OpenSim:  VectorFunctionForActuators.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
/*  
 * Author: Frank C. Anderson 
 */

// INCLUDES
#include "VectorFunctionForActuators.h"
#include <OpenSim/Simulation/Model/CMCActuatorSubsystem.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "CMC.h"


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
    VectorFunctionUncoupledNxN(model->getControllerSet().get("CMC" ).getActuatorSet().getSize() ),
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
// INITIAL TIME
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
evaluate(const SimTK::State& s, const double *aX, double *rF)
{
    int i;
    int N = getNX();

    CMC& controller=  dynamic_cast<CMC&>(_model->updControllerSet().get("CMC" ));
    controller.updControlSet().setControlValues(_tf, aX);

    // create a Manager that will integrate just the actuator subsystem and use only the 
    // CMC controller
    SimTK::State& actSysState = _CMCActuatorSystem->updDefaultState();
    getCMCActSubsys()->updZ(actSysState) = _model->getMultibodySystem()
                                            .getDefaultSubsystem().getZ(s);
    actSysState.setTime(_ti);

    SimTK::TimeStepper ts(*_CMCActuatorSystem, *_integrator);
    ts.initialize(actSysState);
    ts.stepTo(_tf);

    const Set<const Actuator>& forceSet = controller.getActuatorSet();
    // Vector function values
    int j = 0;
    for(i=0;i<N;i++) {
        auto act = dynamic_cast<const ScalarActuator*>(&forceSet[i]);
        rF[j] = act->getActuation(getCMCActSubsys()->getCompleteState()) - _f[j];
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
void VectorFunctionForActuators::evaluate(const SimTK::State& s,
        const OpenSim::Array<double>& aX, OpenSim::Array<double>& rF,
        const OpenSim::Array<int>& aDerivWRT) {
    log_warn("VectorFunctionForActuators::evaluate: Unimplemented method.");
}

//_____________________________________________________________________________
/**
 * Evaluate the vector function.
 *
 * @param aX Array of controls.
 * @param aF Array of actuator force differences.
 */
void VectorFunctionForActuators::evaluate(const SimTK::State& s,
        const OpenSim::Array<double>& aX, OpenSim::Array<double>& rF) {
    evaluate( s, &aX[0],&rF[0]);
}
