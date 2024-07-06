/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
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

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Actuators/DeGrooteFregly2016Muscle.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Simulation/Model/Model.h>
using namespace OpenSim;

void createHangingMuscleModel() {
    Model model;
    model.setName("isometric_muscle");
    model.set_gravity(SimTK::Vec3(9.81, 0, 0));
    auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("height");
    model.addComponent(joint);

    auto* actu = new DeGrooteFregly2016Muscle();
    actu->setName("muscle");
    actu->set_max_isometric_force(10.0);
    actu->set_optimal_fiber_length(0.2);
    actu->set_tendon_slack_length(0.01);
    actu->set_tendon_strain_at_one_norm_force(0.10);
    actu->set_fiber_damping(0.01);
    actu->set_max_contraction_velocity(10);
    actu->set_pennation_angle_at_optimal(0);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addForce(actu);
    body->attachGeometry(new Sphere(0.05));
    
    auto* metabolicModel = new Bhargava2004SmoothedMuscleMetabolics();
    metabolicModel->setName("metabolic_cost");
    double musclemass = 1.123; 
    metabolicModel->addMuscle(actu->getName(), *actu, musclemass);
    model.addComponent(metabolicModel);
    model.finalizeConnections();
    model.print("hanging_muscle_met.osim");
    Model met_model = Model("hanging_muscle_met.osim"); 

    return;
}

int main() {
    Object::registerType(DeGrooteFregly2016Muscle());
    
    createHangingMuscleModel();
    Model met_model = Model("hanging_muscle_met.osim");
    met_model.initSystem();
    const auto bharg = met_model.getComponent<Bhargava2004SmoothedMuscleMetabolics>("/metabolic_cost");
    Bhargava2004SmoothedMuscleMetabolics_MuscleParameters mus = bharg.get_muscle_parameters(0);
    double testmass = mus.getMuscleMass();
    double expectedMass = 1.123; // Set the expected mass here
    
    if (testmass != expectedMass) {
        std::cerr << "Error: The muscle mass does not match the expected mass." << std::endl;
        return EXIT_FAILURE;
    }
    else { 
        std::cout << "Muscle mass matches expected mass." << std::endl;
        }

    return EXIT_SUCCESS;
}
