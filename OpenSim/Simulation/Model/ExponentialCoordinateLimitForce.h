#ifndef OPENSIM_EXPONENTIAL_COORDINATE_LIMIT_FORCE_H_
#define OPENSIM_EXPONENTIAL_COORDINATE_LIMIT_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ExponentialCoordinateLimitForce.h               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

#include <OpenSim/Simulation/Model/ForceProducer.h>

namespace OpenSim {

/**
 * A class for modeling coordinate limit forces using exponential spring
 * functions. This class is based on the exponential functions used to model the
 * contributions from ligaments described in the following publication:
 *
 *     Anderson F.C. and Pandy M.G. (1999). A dynamics optimization
 *     solution for vertical jumping in three dimensions. Computer Methods
 *     in Biomechanics and Biomedical Engineering 2(3):201-231.
 *
 * The limit force in this class is defined by the following equation:
 *
 * f = αₗ exp(−βₗ(q − qₗ)) − αᵤ exp(βᵤ(q − qᵤ))
 *
 * For example, if the lower shape parameters are αₗ = 50, βₗ = 75, the upper
 * shape parameters are αᵤ = 20, βᵤ = 25, and the lower and upper coordinate
 * limits are qₗ = -0.2 and qᵤ = 0.3, then the limit force curve will take the
 * following shape:
 *
 * \image html exponential_coordinate_limit_force.png width=100%
 *
 * Note that, depending on the choice of shape parameters, the limit force may
 * start developing prior to reaching the coordinate limits.
 */
class OSIMSIMULATION_API ExponentialCoordinateLimitForce : public ForceProducer {
OpenSim_DECLARE_CONCRETE_OBJECT(ExponentialCoordinateLimitForce, ForceProducer);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(coordinate, std::string,
        "The name or full path of the coordinate to which the limit forces are "
        "applied.");
    OpenSim_DECLARE_PROPERTY(lower_limit, double,
        "The lower limit of the coordinate range of motion.");
    OpenSim_DECLARE_PROPERTY(upper_limit, double,
        "The upper limit of the coordinate range of motion.");
    OpenSim_DECLARE_PROPERTY(lower_shape_parameters, SimTK::Vec2,
        "Shape parameters for the exponential function that models the lower "
        "limit force.");
    OpenSim_DECLARE_PROPERTY(upper_shape_parameters, SimTK::Vec2,
        "Shape parameters for the exponential function that models the upper "
        "limit force.");

//=============================================================================
// METHODS
//=============================================================================
    /**
     * Default constructor.
     */
    ExponentialCoordinateLimitForce();

    /**
     * Convenience constructor.
     *
     * @param[in] coordinateNameOrPath The name or full path of the coordinate
     * to which the limit forces are applied.
     * @param[in] lowerLimit The lower limit of the coordinate range of motion.
     * @param[in] upperLimit The upper limit of the coordinate range of motion.
     * @param[in] shapeParametersLower The shape parameters for the exponential
     * function that models the lower limit force.
     * @param[in] shapeParametersUpper The shape parameters for the exponential
     * function that models the upper limit force.
     */
    ExponentialCoordinateLimitForce(
        const std::string& coordinateNameOrPath, double lowerLimit,
        double upperLimit, const SimTK::Vec2& shapeParametersLower,
        const SimTK::Vec2& shapeParametersUpper);

    // COMPUTATIONS
    double calcForce(const SimTK::State& s) const;
    double computePotentialEnergy(const SimTK::State& s) const override;

    // REPORTING
    Array<std::string> getRecordLabels() const override;
    Array<double> getRecordValues(const SimTK::State& state) const override;

protected:
    // MODEL COMPONENT INTERFACE
    void extendConnectToModel(Model& aModel) override;

private:
    // FORCE PRODUCER INTERFACE
    void implProduceForces(const SimTK::State&, ForceConsumer&) const override;

    // HELPERS
    void constructProperties();

    SimTK::ReferencePtr<const Coordinate> _coord;

};  // class ExponentialCoordinateLimitForce

} // namespace OpenSim

#endif // #ifndef OPENSIM_EXPONENTIAL_COORDINATE_FORCE_H_
