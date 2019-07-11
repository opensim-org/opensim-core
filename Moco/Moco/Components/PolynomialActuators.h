//#ifndef MOCO_POLYNOMIALACTUATORS_H
//#define MOCO_POLYNOMIALACTUATORS_H
///* -------------------------------------------------------------------------- *
// * OpenSim Moco: PolynomialActuators.h                                        *
// * -------------------------------------------------------------------------- *
// * Copyright (c) 2017-19 Stanford University and the Authors                  *
// *                                                                            *
// * Author(s): Antoine Falisse                                                 *
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
//#include "../MocoUtilities.h"
//#include "../osimMocoDLL.h"
//
//#include <OpenSim/Simulation/Model/GeometryPath.h>
//
//namespace OpenSim {
//
//class OSIMMOCO_API  PolynomialActuators : public GeometryPath {
//    OpenSim_DECLARE_CONCRETE_OBJECT(PolynomialActuators, GeometryPath);
//public:
////=============================================================================
//// PROPERTIES
////=============================================================================
//    OpenSim_DECLARE_PROPERTY(function, Function,
//            "The order of the polynomial approximation");
//    OpenSim_DECLARE_PROPERTY(dimension, int,
//            "The dimension of the polynomial approximation");
//    OpenSim_DECLARE_PROPERTY(order, int,
//            "The order of the polynomial approximation");
//    OpenSim_DECLARE_PROPERTY(coefficients, SimTK::Vector,
//            "The polynomial coefficients TODO specify order");
//
////=============================================================================
//// SOCKETS
////=============================================================================
//    // TODO max 4 dependent variables. Should all sockets be used?
//    OpenSim_DECLARE_SOCKET(coordinate1, Coordinate,
//        "The approximation is a function of this first coordinate's value.");
//    OpenSim_DECLARE_SOCKET(coordinate2, Coordinate,
//        "The approximation is a function of this second coordinate's value.");
//    OpenSim_DECLARE_SOCKET(coordinate3, Coordinate,
//        "The approximation is a function of this third coordinate's value.");
//    OpenSim_DECLARE_SOCKET(coordinate4, Coordinate,
//        "The approximation is a function of this fourth coordinate's value.");
//
////=============================================================================
//// METHODS
////=============================================================================
//    PolynomialActuators();
//    PolynomialActuators(const std::string& name,
//        const Coordinate& coordinate1, const Coordinate& coordinate2,
//        const Coordinate& coordinate3, const Coordinate& coordinate4);
//
//
//    bool hasCoordinate1() const;
//    bool hasCoordinate2() const;
//    bool hasCoordinate3() const;
//    bool hasCoordinate4() const;
//
//    const Coordinate& getCoordinate1() const;
//    const Coordinate& getCoordinate2() const;
//    const Coordinate& getCoordinate3() const;
//    const Coordinate& getCoordinate4() const;
//
//    void setCoordinate1(const Coordinate& coordinate);
//    void setCoordinate2(const Coordinate& coordinate);
//    void setCoordinate3(const Coordinate& coordinate);
//    void setCoordinate4(const Coordinate& coordinate);
//
//    // Length and Speed of actuator
//    double getLength(const SimTK::State& s) const;
//    double getLengtheningSpeed(const SimTK::State& s) const;
//
//    // TODO
//    /// Add in the equivalent body and generalized forces to be applied to the
//    /// multibody system resulting from a tension along the GeometryPath
//    /// @param state    state used to evaluate forces
//    /// @param[in]  tension      scalar (double) of the applied (+ve) tensile force
//    /// @param[in,out] bodyForces   Vector of SpatialVec's (torque, force) on bodies
//    /// @param[in,out] mobilityForces  Vector of generalized forces, one per mobility
//    void addInEquivalentForces(const SimTK::State& state,
//                               const double& tension,
//                               SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
//                               SimTK::Vector& mobilityForces) const;
//
//private:
//    void constructProperties();
//    void extendConnectToModel(Model& model) override;
//
////=============================================================================
//};  // END of class PolynomialActuators
////=============================================================================
////=============================================================================
//
//} // namespace OpenSim
//
//#endif // MOCO_POLYNOMIALACTUATORS_H
