# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleHangingMuscle.py                                      #
# -------------------------------------------------------------------------- #
# Copyright (c) 2020 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Christopher Dembia                                              #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License"); you may    #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #
# This example includes a point mass hanging by a muscle (+x is downward),
# and shows how to use MocoStudy with a model that includes a muscle.
# Additionally, this example shows how to use OpenSim's Analyses with a
# MocoSolution.
# The trajectory optimization problem is to lift the point mass by a small
# distance in minimum time.

import os
import opensim as osim

def createHangingMuscleModel(ignore_activation_dynamics,
                             ignore_tendon_compliance):
    model = osim.Model()
    model.setName("hanging_muscle")
    model.set_gravity(osim.Vec3(9.81, 0, 0))
    body = osim.Body("body", 0.5, osim.Vec3(0), osim.Inertia(1))
    model.addComponent(body)

    # Allows translation along x.
    joint = osim.SliderJoint("joint", model.getGround(), body)
    coord = joint.updCoordinate()
    coord.setName("height")
    model.addComponent(joint)

    # The point mass is supported by a muscle.
    # The DeGrooteFregly2016Muscle is the only muscle model in OpenSim that
    # has been tested with Moco.
    actu = osim.DeGrooteFregly2016Muscle()
    actu.setName("muscle")
    actu.set_max_isometric_force(30.0)
    actu.set_optimal_fiber_length(0.10)
    actu.set_tendon_slack_length(0.05)
    actu.set_tendon_strain_at_one_norm_force(0.10)
    actu.set_ignore_activation_dynamics(ignore_activation_dynamics)
    actu.set_ignore_tendon_compliance(ignore_tendon_compliance)
    actu.set_fiber_damping(0.01)
    # The DeGrooteFregly2016Muscle is the only muscle model in OpenSim that
    # can express its tendon compliance dynamics using an implicit
    # differential equation.
    actu.set_tendon_compliance_dynamics_mode("implicit")
    actu.set_max_contraction_velocity(10)
    actu.set_pennation_angle_at_optimal(0.10)
    actu.addNewPathPoint("origin", model.updGround(), osim.Vec3(0))
    actu.addNewPathPoint("insertion", body, osim.Vec3(0))
    model.addForce(actu)

    # Add metabolics probes: one for the total metabolic rate,
    # and one for each term in the metabolics model.
    probe = osim.Umberger2010MuscleMetabolicsProbe()
    probe.setName("metabolics")
    probe.addMuscle("muscle", 0.5)
    model.addProbe(probe)

    probe = osim.Umberger2010MuscleMetabolicsProbe()
    probe.setName("activation_maintenance_rate")
    probe.set_activation_maintenance_rate_on(True)
    probe.set_shortening_rate_on(False)
    probe.set_basal_rate_on(False)
    probe.set_mechanical_work_rate_on(False)
    probe.addMuscle("muscle", 0.5)
    model.addProbe(probe)

    probe = osim.Umberger2010MuscleMetabolicsProbe()
    probe.setName("shortening_rate")
    probe.set_activation_maintenance_rate_on(False)
    probe.set_shortening_rate_on(True)
    probe.set_basal_rate_on(False)
    probe.set_mechanical_work_rate_on(False)
    probe.addMuscle("muscle", 0.5)
    model.addProbe(probe);

    probe = osim.Umberger2010MuscleMetabolicsProbe()
    probe.setName("basal_rate")
    probe.set_activation_maintenance_rate_on(False)
    probe.set_shortening_rate_on(False)
    probe.set_basal_rate_on(True)
    probe.set_mechanical_work_rate_on(False)
    probe.addMuscle("muscle", 0.5)
    model.addProbe(probe)

    probe = osim.Umberger2010MuscleMetabolicsProbe()
    probe.setName("mechanical_work_rate")
    probe.set_activation_maintenance_rate_on(False)
    probe.set_shortening_rate_on(False)
    probe.set_basal_rate_on(False)
    probe.set_mechanical_work_rate_on(True)
    probe.addMuscle("muscle", 0.5)
    model.addProbe(probe)

    body.attachGeometry(osim.Sphere(0.05))

    model.finalizeConnections()

    return model

ignore_activation_dynamics = False
ignore_tendon_compliance = False
model = createHangingMuscleModel(ignore_activation_dynamics,
                                 ignore_tendon_compliance)
model.printToXML("hanging_muscle.osim")

study = osim.MocoStudy()
problem = study.updProblem()
problem.setModelAsCopy(model)
problem.setTimeBounds(0, [0.05, 1.0])
problem.setStateInfo("/joint/height/value", [0.14, 0.16], 0.15, 0.14)
problem.setStateInfo("/joint/height/speed", [-1, 1], 0, 0)
problem.setControlInfo("/forceset/muscle", [0.01, 1])

# Initial state constraints/costs.
if not ignore_activation_dynamics:
    initial_activation = osim.MocoInitialActivationGoal()
    problem.addGoal(initial_activation)
    initial_activation.setName("initial_activation")
if not ignore_tendon_compliance:
    initial_equilibrium = osim.MocoInitialVelocityEquilibriumDGFGoal()
    problem.addGoal(initial_equilibrium)
    initial_equilibrium.setName("initial_velocity_equilibrium")
    # The problem converges in fewer iterations when this goal is in cost mode.
    initial_equilibrium.setMode("cost")
    initial_equilibrium.setWeight(0.001)

problem.addGoal(osim.MocoFinalTimeGoal())

solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(25)
solver.set_multibody_dynamics_mode("implicit")
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-4)

solution = study.solve()
osim.STOFileAdapter.write(solution.exportToStatesTable(),
                      "exampleHangingMuscle_states.sto")
osim.STOFileAdapter.write(solution.exportToControlsTable(),
                      "exampleHangingMuscle_controls.sto")

# Conduct an analysis using MuscleAnalysis and ProbeReporter.
# Create an AnalyzeTool setup file.
analyze = osim.AnalyzeTool()
analyze.setName("analyze")
analyze.setModelFilename("hanging_muscle.osim")
analyze.setStatesFileName("exampleHangingMuscle_states.sto")
analyze.updAnalysisSet().cloneAndAppend(osim.MuscleAnalysis())
analyze.updAnalysisSet().cloneAndAppend(osim.ProbeReporter())
analyze.updControllerSet().cloneAndAppend(
    osim.PrescribedController("exampleHangingMuscle_controls.sto"))
analyze.printToXML("exampleHangingMuscle_AnalyzeTool_setup.xml")
# Run the analysis.
analyze = osim.AnalyzeTool("exampleHangingMuscle_AnalyzeTool_setup.xml")
analyze.run()

table_force = osim.TimeSeriesTable(
    "analyze_MuscleAnalysis_ActiveFiberForce.sto")
table_velocity = osim.TimeSeriesTable(
    "analyze_MuscleAnalysis_FiberVelocity.sto")
time = table_force.getIndependentColumn()
force = table_force.getDependentColumn("muscle").to_numpy()
velocity = table_velocity.getDependentColumn("muscle").to_numpy()

# Plot the terms of the metabolics model, and compare the metabolics model's
# mechanical work rate to the mechanical work rate computed using the
# MuscleAnalysis.
plot = False
# The following environment variable is set during automated testing.
if os.getenv('OPENSIM_USE_VISUALIZER') != '0':
    try:
        import pylab as pl
        plot = True
    except:
        print('Skipping plotting')

if plot:
    pl.plot(time, force * -velocity,
            label='active_fiber_force * fiber_velocity', lw=4)

    table_metabolics = osim.TimeSeriesTable("analyze_ProbeReporter_probes.sto")
    time = table_metabolics.getIndependentColumn()
    metabolics_total_rate = table_metabolics.getDependentColumn(
        "metabolics_TOTAL").to_numpy()
    pl.plot(time, metabolics_total_rate, label='total metabolic rate')

    mech_work_rate = table_metabolics.getDependentColumn(
        "mechanical_work_rate_TOTAL").to_numpy()
    pl.plot(time, mech_work_rate, label='mechanical work rate')

    activation_maintenance_rate = table_metabolics.getDependentColumn(
        "activation_maintenance_rate_TOTAL").to_numpy()
    pl.plot(time, activation_maintenance_rate,
            label='activation maintenance rate')

    shortening_rate = table_metabolics.getDependentColumn(
        "shortening_rate_TOTAL").to_numpy()
    pl.plot(time, shortening_rate, label='shortening rate')

    basal_rate = table_metabolics.getDependentColumn(
        "basal_rate_TOTAL").to_numpy()
    pl.plot(time, basal_rate, label='basal rate')
    pl.legend()
    pl.show()

