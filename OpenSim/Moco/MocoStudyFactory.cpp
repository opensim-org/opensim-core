/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoStudyFactory.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include "MocoStudyFactory.h"

#include "MocoProblem.h"
#include <SimTKcommon/Scalar.h>

#include <OpenSim/Actuators/ModelFactory.h>

using namespace OpenSim;

MocoStudy MocoStudyFactory::createLinearTangentSteeringStudy(
        double acceleration, double finalTime, double finalHeight) {

    class DirectionActuator : public ScalarActuator {
        OpenSim_DECLARE_CONCRETE_OBJECT(DirectionActuator, ScalarActuator);

    public:
        OpenSim_DECLARE_PROPERTY(
                acceleration, double, "Constant acceleration.");
        DirectionActuator() { constructProperty_acceleration(1.0); }
        double computeActuation(const SimTK::State&) const override {
            return SimTK::NaN;
        }
        void computeForce(const SimTK::State& state,
                SimTK::Vector_<SimTK::SpatialVec>&,
                SimTK::Vector& mobilityForces) const override {
            const double angle = getControl(state);
            const auto& coordX = getModel().getCoordinateSet().get(0);
            const auto& coordY = getModel().getCoordinateSet().get(1);
            applyGeneralizedForce(state, coordX,
                    get_acceleration() * cos(angle), mobilityForces);
            applyGeneralizedForce(state, coordY,
                    get_acceleration() * sin(angle), mobilityForces);
        }

        double getSpeed(const SimTK::State& state) const override
        {
            return SimTK::NaN;
        }
    };

    class LinearTangentFinalSpeed : public MocoGoal {
    public:
        OpenSim_DECLARE_CONCRETE_OBJECT(LinearTangentFinalSpeed, MocoGoal);
        void initializeOnModelImpl(const Model&) const override {
            setRequirements(0, 1);
        }
        void calcGoalImpl(
                const GoalInput& input, SimTK::Vector& cost) const override {
            cost[0] = -input.final_state.getU()[0];
        }
    };

    auto model = ModelFactory::createPlanarPointMass();
    model.set_gravity(SimTK::Vec3(0));
    model.updForceSet().clearAndDestroy();
    auto* actuator = new DirectionActuator();
    actuator->setName("actuator");
    actuator->set_acceleration(acceleration);
    model.addForce(actuator);
    model.finalizeConnections();

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, finalTime);
    problem.setStateInfo("/jointset/tx/tx/value", {0, 10}, 0);
    problem.setStateInfo(
            "/jointset/ty/ty/value", {0, finalHeight}, 0, finalHeight);
    problem.setStateInfo("/jointset/tx/tx/speed", {0, 10}, 0);
    problem.setStateInfo("/jointset/ty/ty/speed", {0, 10}, 0, 0);
    problem.setControlInfo(
            "/forceset/actuator", {-0.5 * SimTK::Pi, 0.5 * SimTK::Pi});

    problem.addGoal<LinearTangentFinalSpeed>();

    return study;
}
