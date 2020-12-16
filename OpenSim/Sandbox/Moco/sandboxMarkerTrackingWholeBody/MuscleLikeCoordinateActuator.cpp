/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MuscleLikeCoordinateActuator.cpp                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Carmichael Ong                                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>

#include "MuscleLikeCoordinateActuator.h"

using namespace OpenSim;
using namespace std;

MuscleLikeCoordinateActuator::MuscleLikeCoordinateActuator(
        const std::string& coordinateName) {
    setNull();
    constructProperties();

    if (!coordinateName.empty())
        set_coordinate(coordinateName);
}

MuscleLikeCoordinateActuator::MuscleLikeCoordinateActuator(
        const std::string& coordinateName, 
        double optimalForce,
        Function* posForceVsCoordinateFunction,
        Function* negForceVsCoordinateFunction, 
        double qdot_max) {
    setNull();
    constructProperties();

    set_coordinate(coordinateName);
    set_optimal_force(optimalForce);
    set_pos_force_vs_coordinate_function(*posForceVsCoordinateFunction);
    set_neg_force_vs_coordinate_function(*negForceVsCoordinateFunction);
    set_qdot_max(qdot_max);
}

void MuscleLikeCoordinateActuator::setNull() {
    setAuthors("Carmichael Ong");
}

void MuscleLikeCoordinateActuator::constructProperties() {
    constructProperty_pos_force_vs_coordinate_function(Constant(0.0));
    constructProperty_neg_force_vs_coordinate_function(Constant(0.0));
    constructProperty_qdot_max(1.0);
}

void MuscleLikeCoordinateActuator::setPosForceVsCoordinateFunction(
        Function* posForceVsCoordinateFunction) {
    set_pos_force_vs_coordinate_function(*posForceVsCoordinateFunction);
}

const 
Function* MuscleLikeCoordinateActuator::getPosForceVsCoordinateFunction() {
    return &get_pos_force_vs_coordinate_function();
}

void MuscleLikeCoordinateActuator::setNegForceVsCoordinateFunction(
        Function* negForceVsCoordinateFunction) {
    set_neg_force_vs_coordinate_function(*negForceVsCoordinateFunction);
}

const 
Function* MuscleLikeCoordinateActuator::getNegForceVsCoordinateFunction() {
    return &get_neg_force_vs_coordinate_function();
}

void MuscleLikeCoordinateActuator::setMaxVelocity(double qdot_max) {
    set_qdot_max(qdot_max);
}

double MuscleLikeCoordinateActuator::getMaxVelocity() {
    return get_qdot_max();
}

double MuscleLikeCoordinateActuator::computeActuation( 
        const SimTK::State& s) const {
    return getControl(s) * getOptimalForce() *
           getForceVsCoordinateFunctionValue(s) * getVelocityMultiplier(s);
}

double MuscleLikeCoordinateActuator::getForceVsCoordinateFunctionValue( 
        const SimTK::State& s) const {
    double coordinateValue = getCoordinate()->getValue(s);
    if ( getControl(s) >= 0.0 ) {
        double thisValue = get_pos_force_vs_coordinate_function().calcValue(
                SimTK::Vector(1, coordinateValue));

        if (thisValue <= 0.0) {
            return 0.0;
        } else {
            return thisValue;
        }

    } else {
        double thisValue = get_neg_force_vs_coordinate_function().calcValue(
            SimTK::Vector(1, coordinateValue));

        if (thisValue <= 0.0) {
            return 0.0;
        } else {
            return thisValue;
        }
    }
}

double MuscleLikeCoordinateActuator::getVelocityMultiplier(
        const SimTK::State& s) const {
    double qdot = getSpeed(s);
    double qdot_max = get_qdot_max();

    if ( getControl(s) >= 0.0 ) {

        if ( qdot > qdot_max ) {
            return 0.0;

        } else if ( qdot < (-0.35678917232)*qdot_max ) {
            return 1.5;

        } else {
            return (1 - (1.01750751592)*atan(1.5*qdot/qdot_max));
        }
    } else {

        if ( qdot < -qdot_max) {
            return 0.0;

        } else if ( qdot > (0.35678917232)*qdot_max ) {
            return 1.5;

        } else {
            return (1 - (1.01750751592)*atan(1.5*qdot/(-qdot_max)));
        }
    }
}
