/* -------------------------------------------------------------------------- *
 * OpenSim Moco: PolynomialActuators.cpp                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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

#include "PolynomialActuators.h"

using namespace OpenSim;

//=============================================================================
//  POLYNOMIAL ACTUATORS
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy
// assignment operator.

// Default constructor.
PolynomialActuators::PolynomialActuators()
{
    constructProperties();
}

double PolynomialActuators::getLength(const SimTK::State& s) const
{
    int nc = getProperty_coordinate_list().size();
    double* coordinateValues = new double[nc]; // TODO, is this ok? Should I delete later?
    for (int i = 0; i < nc; ++i)
        coordinateValues[i] = coordinates[i]->getValue(s);
    SimTK::Vector x(nc,coordinateValues,true);

    return get_function().calcValue(x);

}

double PolynomialActuators::getLengtheningSpeed(const SimTK::State& s) const
{
    int nc = getProperty_coordinate_list().size();
    double* coordinateValues = new double[nc]; // TODO, is this ok? Should I delete later?
    for (int i = 0; i < nc; ++i)
        coordinateValues[i] = coordinates[i]->getValue(s);
    SimTK::Vector x(nc,coordinateValues,true);

    std::vector<int> derivComponents(1);
    double value = 0;
    for (int i = 0; i < nc; ++i) {
        derivComponents[0] = i;
        value += get_function().calcDerivative(derivComponents, x) *
                coordinates[i]->getSpeedValue(s);
    }
    return value;

}

// TODO
void PolynomialActuators::addInEquivalentForces(const SimTK::State& s,
    const double& tension,
    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
    SimTK::Vector& mobilityForces) const
{
    int nc = getProperty_coordinate_list().size();
    double* coordinateValues = new double[nc];
    for (int i = 0; i < nc; ++i)
        coordinateValues[i] = coordinates[i]->getValue(s);
    SimTK::Vector x(nc,coordinateValues,true);

    std::vector<int> derivComponents(1);
    double value = 0;
    for (int i = 0; i < nc; ++i) {
        derivComponents[0] = i;
        -get_function().calcDerivative(derivComponents, x); // this is the dM
    }

}

void PolynomialActuators::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model); // TODO would be good to understand this, not sure needed?

    int nc = getProperty_coordinate_list().size();
    for (int i = 0; i < nc; ++i) {
        coordinates[i] =
                model.getComponent<Coordinate>(get_coordinate_list(i));
    }

}

void PolynomialActuators::constructProperties() {

    constructProperty_function(MultivariatePolynomialFunction());
    constructProperty_coordinate_list();

}
