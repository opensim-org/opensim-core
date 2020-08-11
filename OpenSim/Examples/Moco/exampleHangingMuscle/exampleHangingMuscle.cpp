/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleHangingMuscle.cpp                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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

/// This example includes a point mass hanging by a muscle (+x is downward),
/// and shows how to use MocoStudy with a model that includes a muscle.
/// Additionally, this example shows how to use OpenSim's Analyses with a
/// MocoSolution.
/// The trajectory optimization problem is to lift the point mass by a small
/// distance in minimum time.

#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Analyses/ProbeReporter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Tools/AnalyzeTool.h>

using namespace OpenSim;

Model createHangingMuscleModel(bool ignoreActivationDynamics,
        bool ignoreTendonCompliance) {
    Model model;
    model.setName("hanging_muscle");
    model.set_gravity(SimTK::Vec3(9.81, 0, 0));
    auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("height");
    model.addComponent(joint);

    // The point mass is supported by a muscle. The DeGrooteFregly2016Muscle
    // is the only muscle model in OpenSim that has been tested with Moco.
    auto* actu = new DeGrooteFregly2016Muscle();
    actu->setName("muscle");
    actu->set_max_isometric_force(30.0);
    actu->set_optimal_fiber_length(0.10);
    actu->set_tendon_slack_length(0.05);
    actu->set_tendon_strain_at_one_norm_force(0.10);
    actu->set_ignore_activation_dynamics(ignoreActivationDynamics);
    actu->set_ignore_tendon_compliance(ignoreTendonCompliance);
    actu->set_fiber_damping(0.01);
    // The DeGrooteFregly2016Muscle is the only muscle model in OpenSim that
    // can express its tendon compliance dynamics using an implicit
    // differential equation.
    actu->set_tendon_compliance_dynamics_mode("implicit");
    actu->set_max_contraction_velocity(10);
    actu->set_pennation_angle_at_optimal(0.10);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addForce(actu);

    // Add metabolics probes: one for the total metabolic rate,
    // and one for each term in the metabolics model.
    {
        auto* probe = new Umberger2010MuscleMetabolicsProbe();
        probe->setName("metabolics");
        probe->addMuscle("muscle", 0.5);
        model.addProbe(probe);
    }
    {
        auto* probe = new Umberger2010MuscleMetabolicsProbe();
        probe->setName("activation_maintenance_rate");
        probe->set_activation_maintenance_rate_on(true);
        probe->set_shortening_rate_on(false);
        probe->set_basal_rate_on(false);
        probe->set_mechanical_work_rate_on(false);
        probe->addMuscle("muscle", 0.5);
        model.addProbe(probe);
    }
    {
        auto* probe = new Umberger2010MuscleMetabolicsProbe();
        probe->setName("shortening_rate");
        probe->set_activation_maintenance_rate_on(false);
        probe->set_shortening_rate_on(true);
        probe->set_basal_rate_on(false);
        probe->set_mechanical_work_rate_on(false);
        probe->addMuscle("muscle", 0.5);
        model.addProbe(probe);
    }
    {
        auto* probe = new Umberger2010MuscleMetabolicsProbe();
        probe->setName("basal_rate");
        probe->set_activation_maintenance_rate_on(false);
        probe->set_shortening_rate_on(false);
        probe->set_basal_rate_on(true);
        probe->set_mechanical_work_rate_on(false);
        probe->addMuscle("muscle", 0.5);
        model.addProbe(probe);
    }
    {
        auto* probe = new Umberger2010MuscleMetabolicsProbe();
        probe->setName("mechanical_work_rate");
        probe->set_activation_maintenance_rate_on(false);
        probe->set_shortening_rate_on(false);
        probe->set_basal_rate_on(false);
        probe->set_mechanical_work_rate_on(true);
        probe->addMuscle("muscle", 0.5);
        model.addProbe(probe);
    }

    body->attachGeometry(new Sphere(0.05));

    model.finalizeConnections();

    return model;
}

int main() {

    const bool ignoreActivationDynamics = false;
    const bool ignoreTendonCompliance = false;
    Model model = createHangingMuscleModel(ignoreActivationDynamics,
            ignoreTendonCompliance);
    model.print("hanging_muscle.osim");

    MocoStudy study;
    MocoProblem& problem = study.updProblem();
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, {0.05, 1.0});
    problem.setStateInfo("/joint/height/value", {0.14, 0.16}, 0.15, 0.14);
    problem.setStateInfo("/joint/height/speed", {-1, 1}, 0, 0);
    problem.setControlInfo("/forceset/muscle", {0.01, 1});

    // Initial state constraints/costs.
    if (!ignoreActivationDynamics) {
        auto* initial_activation = problem.addGoal<MocoInitialActivationGoal>();
        initial_activation->setName("initial_activation");
    }
    if (!ignoreTendonCompliance) {
        auto* initial_equilibrium =
                problem.addGoal<MocoInitialVelocityEquilibriumDGFGoal>();
        initial_equilibrium->setName("initial_velocity_equilibrium");
        // The problem converges in fewer iterations when this goal is in cost
        // mode.
        initial_equilibrium->setMode("cost");
        initial_equilibrium->setWeight(0.001);
    }

    problem.addGoal<MocoFinalTimeGoal>();

    auto& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(50);
    solver.set_multibody_dynamics_mode("implicit");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);

    MocoSolution solution = study.solve();
    STOFileAdapter::write(solution.exportToStatesTable(),
            "exampleHangingMuscle_states.sto");
    STOFileAdapter::write(solution.exportToControlsTable(),
            "exampleHangingMuscle_controls.sto");

    // Conduct an analysis using MuscleAnalysis and ProbeReporter.
    {
        // Create an AnalyzeTool setup file.
        AnalyzeTool analyze;
        analyze.setName("analyze");
        analyze.setModelFilename("hanging_muscle.osim");
        analyze.setStatesFileName("exampleHangingMuscle_states.sto");
        analyze.updAnalysisSet().adoptAndAppend(new MuscleAnalysis());
        analyze.updAnalysisSet().adoptAndAppend(new ProbeReporter());
        analyze.updControllerSet().adoptAndAppend(
                new PrescribedController("exampleHangingMuscle_controls.sto"));
        analyze.print("exampleHangingMuscle_AnalyzeTool_setup.xml");
    }
    // Run the analysis.
    AnalyzeTool analyze("exampleHangingMuscle_AnalyzeTool_setup.xml");
    analyze.run();

    return EXIT_SUCCESS;
}
