///* -------------------------------------------------------------------------- *
// * OpenSim Moco: PolynomialActuators.cpp                                 *
// * -------------------------------------------------------------------------- *
// * Copyright (c) 2017-19 Stanford University and the Authors                  *
// *                                                                            *
// * Author(s): Antoin Falisse                                                  *
// *                                                                            *
// * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
// * not use this file except in compliance with the License. You may obtain a  *
// * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
// *                                                                            *
// * Unless required by applicable law or agreed to in writing, software        *
// * distributed under the License is distributed on an "AS IS" BASIS,          *
// * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
// * See the License for the specific language governing permissions and        *
// * limitations under the License.                                             *
// * -------------------------------------------------------------------------- */
//
//#include "PolynomialActuators.h"
//
//using namespace OpenSim;
//
////=============================================================================
////  POLYNOMIAL ACTUATORS
////=============================================================================
//// Uses default (compiler-generated) destructor, copy constructor, copy
//// assignment operator.
//
//// Default constructor.
//PolynomialActuators::PolynomialActuators()
//{
//    constructProperties();
//}
//
//PolynomialActuators::PolynomialActuators(
//    const std::string& name,
//        const Coordinate& coordinate1, const Coordinate& coordinate2,
//        const Coordinate& coordinate3, const Coordinate& coordinate4)
//{
//    this->connectSocket_coordinate1(coordinate1);
//    this->connectSocket_coordinate2(coordinate2);
//    this->connectSocket_coordinate3(coordinate3);
//    this->connectSocket_coordinate4(coordinate4);
//
//    constructProperties();
//}
//
//const Coordinate& coordinate1Coord = getConnectee<Coordinate>("coordinate1");
//const Coordinate& coordinate2Coord = getConnectee<Coordinate>("coordinate2");
//const Coordinate& coordinate3Coord = getConnectee<Coordinate>("coordinate3");
//const Coordinate& coordinate4Coord = getConnectee<Coordinate>("coordinate4");
//
//
//double PolynomialActuators::getLength(const SimTK::State& s) const
//{
//    double coordinateValues[4] = {coordinate1Coord.getValue(s),
//        coordinate2Coord.getValue(s),coordinate3Coord.getValue(s).
//        coordinate4Coord.getValue(s)};
//    SimTK::Vector x(4,coordinateValues,true);
//    return get_function().calcValue(x);
//
//}
//
//double PolynomialActuators::getLengtheningSpeed(const SimTK::State& s) const
//{
//
//    double coordinateValues[4] = {coordinate1Coord.getValue(s),
//        coordinate2Coord.getValue(s),coordinate3Coord.getValue(s).
//        coordinate4Coord.getValue(s)};
//    SimTK::Vector x(4,coordinateValues,true);
//
//    std::vector<int> derivComponents1(1);
//    derivComponents1[0] = 0;
//    std::vector<int> derivComponents2(1);
//    derivComponents2[0] = 1;
//    std::vector<int> derivComponents3(1);
//    derivComponents3[0] = 2;
//    std::vector<int> derivComponents4(1);
//    derivComponents4[0] = 3;
//
//    return (get_function().calcDerivative(derivComponents1, x) *
//        coordinate1Coord.getSpeedValue(s) +
//        get_function().calcDerivative(derivComponents2, x) *
//        coordinate2Coord.getSpeedValue(s) +
//        get_function().calcDerivative(derivComponents3, x) *
//        coordinate3Coord.getSpeedValue(s) +
//        get_function().calcDerivative(derivComponents4, x) *
//        coordinate4Coord.getSpeedValue(s));
//
//}
//
//void PolynomialActuators::addInEquivalentForces(const SimTK::State& s,
//    const double& tension,
//    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
//    SimTK::Vector& mobilityForces) const
//{
//    double coordinateValues[4] = {coordinate1Coord.getValue(s),
//        coordinate2Coord.getValue(s),coordinate3Coord.getValue(s).
//        coordinate4Coord.getValue(s)};
//    SimTK::Vector x(4,coordinateValues,true);
//
//    std::vector<int> derivComponents1(1);
//    derivComponents1[0] = 0;
//    std::vector<int> derivComponents2(1);
//    derivComponents2[0] = 1;
//    std::vector<int> derivComponents3(1);
//    derivComponents3[0] = 2;
//    std::vector<int> derivComponents4(1);
//    derivComponents4[0] = 3;
//
//    double dM1 = -get_function().calcDerivative(derivComponents1, x);
//    double dM2 = -get_function().calcDerivative(derivComponents2, x);
//    double dM3 = -get_function().calcDerivative(derivComponents3, x);
//    double dM4 = -get_function().calcDerivative(derivComponents4, x);
//
//}
//
//void PolynomialActuators::constructProperties() {
//
//    constructProperty_function(MultivariatePolynomialFunction);
//    double coefficientDefaultValues[6] = {1,1,1,1,1,1};
//    constructProperty_coefficients(
//        SimTK::Vector(6,coefficientDefaultValues,true));
//    constructProperty_dimension(2);
//    constructProperty_order(2);
//
//}
