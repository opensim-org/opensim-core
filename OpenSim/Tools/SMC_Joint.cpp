/* -------------------------------------------------------------------------- *
 *                          OpenSim:  SMC_Joint.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "SMC_Joint.h"
#include <OpenSim/Common/Function.h>

using namespace std;
using namespace OpenSim;

//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SMC_Joint::~SMC_Joint()
{
}

//_____________________________________________________________________________
/**
 * Construct a task for a specified generalized coordinate.
 *
 * @param aQID ID of the generalized coordinate to be tracked.
 * @todo Instead of an integer id, the name of the coordinate
 * should be used.
 */
SMC_Joint::SMC_Joint(const string &aCoordinateName) :
    CMC_Joint(aCoordinateName),
    _s(_propS.getValueDbl())
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTask Joint task to be copied.
 */
SMC_Joint::SMC_Joint(const SMC_Joint &aTask) :
    CMC_Joint(aTask),
    _s(_propS.getValueDbl())
{
    setNull();
    copyData(aTask);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void SMC_Joint::
setNull()
{
    setupProperties();
    _s = 100.0;
}
//_____________________________________________________________________________
/**
 * Set up serialized member variables.
 */
void SMC_Joint::
setupProperties()
{
    _propS.setComment("Parameter for specifying the boundary"
        "of the surface error. The default for this parameter is 100.0."
        " Generally, this parameter can have a value in the range of 1.0 to 1000.0.");
    _propS.setName("surface_error_boundary");
    _propertySet.append(&_propS);
}


//_____________________________________________________________________________
/**
 * Copy only the member data of specified object.
 */
void SMC_Joint::
copyData(const SMC_Joint &aTask)
{
    _s = aTask._s;
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @param aTask Object to be copied.
 * @return  Reference to the altered object.
 */
SMC_Joint& SMC_Joint::
operator=(const SMC_Joint &aTask)
{
    // BASE CLASS
    CMC_Joint::operator =(aTask);

    // DATA
    copyData(aTask);

    return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the desired accelerations.
 * This method assumes that the states have been set for the model.
 *
 * @param aT Time at which the desired accelerations are to be computed in
 * real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void SMC_Joint::
computeDesiredAccelerations( const SimTK::State& state, double aT)
{
    _aDes = SimTK::NaN;

    // CHECK
    if(_model==NULL) return;
    if(_pTrk[0]==NULL) return;

    // COMPUTE ERRORS
    computeErrors(state, aT);

    // Term 1: Experimental Acceleration
    double a;
    if(_aTrk[0]==NULL) {
        std::vector<int> derivComponents(2);
        derivComponents[0]=0;
        derivComponents[1]=0;
        a = (_ka)[0]*_pTrk[0]->calcDerivative(derivComponents,SimTK::Vector(1,aT));
    } else {
        a = (_ka)[0]*_aTrk[0]->calcValue(SimTK::Vector(1,aT));
    }

    // Surface Error
    double s = -_vErr[0] -(_kv)[0]*_pErr[0];

    // Term 2: Velocity
    double v = (_kv)[0]*_vErr[0];

    // Term 3: Robust Term
    double r = (_kp)[0] * tanh(s/_s);

    // DESIRED ACCELERATION
    _aDes[0] = a + v - r;

    // PRINT
    //printf("SMC_Joint.computeDesiredAcceleration:\n");
    //printf("%s:  t=%lf aDes=%lf a=%lf vErr=%lf pErr=%lf\n",getName(),t,_aDes[0],
    //  _pTrk[0]->evaluate(2,t),_vErr[0],_pErr[0]);
}
