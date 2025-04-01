/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleSlidingMassWithBushing.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

/// Translate a point mass in one dimension in minimum time. This is a very
/// simple example that shows only the basics of Moco.
///
/// @verbatim
/// minimize   t_f
/// subject to xdot = v
///            vdot = F/m
///            x(0)   = 0
///            x(t_f) = 1
///            v(0)   = 0
///            v(t_f) = 0
/// w.r.t.     x   in [-5, 5]    position of mass
///            v   in [-50, 50]  speed of mass
///            F   in [-50, 50]  force applied to the mass
///            t_f in [0, 5]     final time
/// constants  m       mass
/// @endverbatim

#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Common/STOFileAdapter.h>

using namespace OpenSim;

std::unique_ptr<Model> createSlidingMassModel() {
    auto model = std::make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 2.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model->addComponent(actu);
    body->attachGeometry(new Sphere(0.05));
    model->finalizeConnections();
    model->print("sliding_mass.osim");
    
    return model;
}

std::unique_ptr<Model> createSlidingMassBushingModel() {
    // Create a new model with a bushing damper.
    auto bushingmodel = std::make_unique<Model>("sliding_mass.osim");
    // create and add a bushing damper for the motion.
    SliderJoint* joint = &bushingmodel->updComponent<SliderJoint>("slider");
    const PhysicalFrame& parentBody = joint->getParentFrame();
    const PhysicalFrame& childBody = joint->getChildFrame();
    // const PhysicalFrame& parentBody = bushingmodel->getComponent("slider").getParentFrame();
    // const PhysicalFrame& childBody = bushingmodel->getComponent("slider").getChildFrame();
    SimTK::Vec3 p1(0, 0, 0);
    SimTK::Vec3 o1(0, 0, 0);
    SimTK::Vec3 p2(0, 0, 0);
    SimTK::Vec3 o2(0, 0, 0);
    SimTK::Vec3 ls(5, 5, 5);
    SimTK::Vec3 rs(0, 0, 0);
    SimTK::Vec3 ld(10, 10, 10);
    SimTK::Vec3 rd(0, 0, 0);

    MocoBushingForce* mbf = new MocoBushingForce("mbf", parentBody, p1, o1, childBody, p2, o2, ls, rs, ld, rd);
    mbf->setName("bushing");
    bushingmodel->addForce(mbf);
    bushingmodel->finalizeConnections();
    bushingmodel->print("sliding_mass_bushing.osim");

    return bushingmodel;
}


std::unique_ptr<Model> createSlidingMassExpressionBushingModel() {
    // Create a new model with a bushing damper.
    auto bushingmodel = std::make_unique<Model>("sliding_mass.osim");
    // create and add a bushing damper for the motion.
    SliderJoint* joint = &bushingmodel->updComponent<SliderJoint>("slider");
    const PhysicalFrame& parentBody = joint->getParentFrame();
    const PhysicalFrame& childBody = joint->getChildFrame();
    // const PhysicalFrame& parentBody = bushingmodel->getComponent("slider").getParentFrame();
    // const PhysicalFrame& childBody = bushingmodel->getComponent("slider").getChildFrame();
    SimTK::Vec3 p1(0, 0, 0);
    SimTK::Vec3 o1(0, 0, 0);
    SimTK::Vec3 p2(0, 0, 0);
    SimTK::Vec3 o2(0, 0, 0);
    SimTK::Vec3 ls(5, 5, 5);
    SimTK::Vec3 rs(0, 0, 0);
    SimTK::Vec3 ld(10, 10, 10);
    SimTK::Vec3 rd(0, 0, 0);

    ExpressionBasedBushingForce* mbf = new ExpressionBasedBushingForce("mbf", parentBody, p1, o1, childBody, p2, o2, ls, rs, ld, rd);
    mbf->setName("bushing");
    bushingmodel->addForce(mbf);
    bushingmodel->finalizeConnections();
    bushingmodel->print("sliding_mass_expression.osim");

    return bushingmodel;
}


int main() {

    MocoStudy study;
    study.setName("sliding_mass");
    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = study.updProblem();
    // Model (dynamics).
    // -----------------
    problem.setModel(createSlidingMassModel());
    // Bounds.
    // -------
    // Initial time must be 0, final time can be within [0, 5].
    problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(0, 5));
    // Position must be within [-5, 5] throughout the motion.
    // Initial position must be 0, final position must be 1.
    problem.setStateInfo("/slider/position/value", MocoBounds(-5, 5),
                         MocoInitialBounds(0), MocoFinalBounds(1));
    // Speed must be within [-50, 50] throughout the motion.
    // Initial and final speed must be 0. Use compact syntax.
    problem.setStateInfo("/slider/position/speed", {-50, 50}, 0, 0);
    // Applied force must be between -50 and 50.
    problem.setControlInfo("/actuator", MocoBounds(-50, 50));
    // Cost.
    // -----
    problem.addGoal<MocoFinalTimeGoal>();
    // Configure the solver.
    // =====================
    MocoCasADiSolver& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(50);
    // Now that we've finished setting up the tool, print it to a file.
    study.print("sliding_mass.omoco");
    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve();
    solution.write("sliding_mass_solution.sto");
    // Visualize.
    // ==========
    study.visualize(solution);


    // ==============
    // Create a new model with a bushing damper.
    // ==============
    problem.setModel(createSlidingMassBushingModel());
    MocoSolution bushingSolution = study.solve();
    bushingSolution.write("sliding_mass_bushing_solution.sto");
    study.visualize(bushingSolution);

    
    // ==============
    // Create a new model with an expresssion based bushing force to show that the forces are the same.
    problem.setModel(createSlidingMassExpressionBushingModel());
    MocoSolution expressionSolution = study.solve();
    expressionSolution.write("sliding_mass_expression_solution.sto");
    study.visualize(expressionSolution);


    // ==============
    // Create a new model with a bushing damper and optimize the damping coefficient.
    // ==============
    problem.setModel(createSlidingMassBushingModel());
    // Add MocoParameter for x component of translational_damping.
    int propertyElt = 0; // y-position is the second element of the mass_center
    std::vector<std::string> componentPaths = {
        "/forceset/bushing"
    };
    MocoParameter mpbf("bushing_damping_x", componentPaths, "translational_damping",
            MocoBounds(0, 100), propertyElt);
    problem.addParameter(mpbf);
    // Solve the problem with optimized damping coefficients.
    MocoSolution bushingOptSolution = study.solve();
    bushingOptSolution.write("sliding_mass_bushing_opt_solution.sto");
    study.visualize(bushingOptSolution);


    // ==============
    // Explore the outputs of the base bushing force. 
    // create a force analysis to explore outputs. 
    const MocoTrajectory& mt = MocoTrajectory("sliding_mass_bushing_solution.sto");
    const std::vector<std::string>& analyze_output_names{".*forceset.*"};
    OpenSim::Model myModel = Model("sliding_mass_bushing.osim");
    std::cout << "Model loaded" << std::endl;
    TimeSeriesTable_<SimTK::Vec6> analyze_outputs = analyzeMocoTrajectory<SimTK::Vec6>(myModel, mt, analyze_output_names);
    STOFileAdapter::write(analyze_outputs.flatten(), "MocoBushingForce_analyze_outputs.sto");
    // ==============
    // create a force analysis to explore outputs of the expression based bushing force - for comparison.
    const MocoTrajectory& mt2 = MocoTrajectory("sliding_mass_expression_solution.sto");
    const std::vector<std::string>& analyze_output_names2{".*forceset.*"};
    OpenSim::Model myModel2 = Model("sliding_mass_expression.osim");
    std::cout << "Model loaded" << std::endl;
    TimeSeriesTable_<SimTK::Vec6> analyze_outputs2 = analyzeMocoTrajectory<SimTK::Vec6>(myModel2, mt2, analyze_output_names2);
    STOFileAdapter::write(analyze_outputs2.flatten(), "ExpressionBasedBushingForce_analyze_outputs.sto");

    //

    return EXIT_SUCCESS;
}
