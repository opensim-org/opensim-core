/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PathActuator.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "PathActuator.h"

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
// default destructor, copy constructor, copy assignment

//_____________________________________________________________________________
/**
 * Default constructor.
 */
PathActuator::PathActuator()
{
    setNull();
    constructProperties();
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void PathActuator::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PathActuator::constructProperties()
{
    constructProperty_path(GeometryPath());
    constructProperty_optimal_force(1.0);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal force of the force.
 *
 * @param aOptimalForce Optimal force.
 */
void PathActuator::setOptimalForce(double aOptimalForce)
{
    set_optimal_force(aOptimalForce);
}

//_____________________________________________________________________________
/**
 * Get the optimal force of the force.
 *
 * @return Optimal force.
 */
double PathActuator::getOptimalForce() const
{
    return get_optimal_force();
}

//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the length of the path actuator. This is a convenience function that 
 * calls the underlying path object for its length.
 *
 * @return Current length of the actuator's path.
 */
double PathActuator::getLength(const SimTK::State& s) const
{
    return getPath().getLength(s);
}
//_____________________________________________________________________________
/**
 * Get the speed of actuator along its path.
 *
 * @return path lengthening speed.
 */
double PathActuator::getLengtheningSpeed(const SimTK::State& s) const
{
    return getPath().getLengtheningSpeed(s);
}
//_____________________________________________________________________________
/**
* Get the stress of the force. This would be the force or torque provided by
* this actuator divided by its optimal force.
* @return Stress.
*/
double PathActuator::getStress(const SimTK::State& s) const
{
    return fabs(getActuation(s)/get_optimal_force());
}


//_____________________________________________________________________________
/**
 * Add a Path point to the _path of the actuator. The new point is appended 
 * to the end of the current path
 *
 */
void PathActuator::addNewPathPoint(
         const std::string& proposedName,
         const PhysicalFrame& aBody,
         const SimTK::Vec3& aPositionOnBody) {
    // Create new PathPoint already appended to the PathPointSet for the path
    updGeometryPath().appendNewPathPoint(proposedName, aBody, aPositionOnBody);
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
double PathActuator::computeActuation(const SimTK::State& s) const
{
    // FORCE
    return( getControl(s) * get_optimal_force() );
}


//=============================================================================
// APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force along path wrapping over and connecting rigid bodies
 */
void PathActuator::computeForce(const SimTK::State& s,
                               SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                               SimTK::Vector& mobilityForces) const
{
    if(!_model) return;

    const auto &path = getPath();

    double force =0;
    if( isActuationOverridden(s) ) {
        force = computeOverrideActuation(s);
    } else {
        force = computeActuation(s);
    }

    // the force of this actuator used to compute power
    setActuation(s, force);

    path.addInEquivalentForces(s, force, bodyForces, mobilityForces);
}

/**
 * Compute the moment-arm of this muscle about a coordinate.
 */
double PathActuator::computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const
{
    return getPath().computeMomentArm(s, aCoord);
}

//------------------------------------------------------------------------------
//                            REALIZE DYNAMICS
//------------------------------------------------------------------------------
// See if anyone has an opinion about the path color and change it if so.
void PathActuator::extendRealizeDynamics(const SimTK::State& state) const
{
    Super::extendRealizeDynamics(state); // Mandatory first line

    // if this force is disabled OR it is being overridden (not computing dynamics)
    // then don't compute the color of the path.
    if (appliesForce(state) && !isActuationOverridden(state)){
        const SimTK::Vec3 color = computePathColor(state);
        if (!color.isNaN())
            getPath().setColor(state, color);
    }
}

//------------------------------------------------------------------------------
//                          COMPUTE PATH COLOR
//------------------------------------------------------------------------------
// This is the PathActuator base class implementation for choosing the path
// color. Derived classes like Muscle will override this with something
// meaningful.
// TODO: should the default attempt to use the actuation level to control
// colors? Not sure how to scale. Muscles could still override that with 
// activation level.
SimTK::Vec3 PathActuator::computePathColor(const SimTK::State& state) const {
    return SimTK::Vec3(SimTK::NaN);
}
