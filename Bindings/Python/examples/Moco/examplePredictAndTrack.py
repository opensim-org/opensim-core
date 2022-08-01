# -------------------------------------------------------------------------- #
# OpenSim Moco: examplePredictAndTrack.py                                    #
# -------------------------------------------------------------------------- #
# Copyright (c) 2018 Stanford University and the Authors                     #
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

import os
import math
import opensim as osim

"""
This file performs the following problems using a
double pendulum model:

    1. predict an optimal trajectory (and controls),
    2. track the states from the optimal trajectory, and
    3. track the marker trajectories from the optimal trajectory.

"""

visualize = True
# The following environment variable is set during automated testing.
if os.getenv('OPENSIM_USE_VISUALIZER') == '0':
    visualize = False

# Create a model of a double pendulum.
# ------------------------------------
def createDoublePendulumModel():
    model = osim.Model()
    model.setName("double_pendulum")

    # Create two links, each with a mass of 1 kg, center of mass at the body's
    # origin, and moments and products of inertia of zero.
    b0 = osim.Body("b0", 1, osim.Vec3(0), osim.Inertia(1))
    model.addBody(b0)
    b1 = osim.Body("b1", 1, osim.Vec3(0), osim.Inertia(1))
    model.addBody(b1)

    # Add markers to body origin locations.
    m0 = osim.Marker("m0", b0, osim.Vec3(0))
    m1 = osim.Marker("m1", b1, osim.Vec3(0))
    model.addMarker(m0)
    model.addMarker(m1)

    # Connect the bodies with pin joints. Assume each body is 1 m long.
    j0 = osim.PinJoint("j0", model.getGround(), osim.Vec3(0), osim.Vec3(0),
        b0, osim.Vec3(-1, 0, 0), osim.Vec3(0))
    q0 = j0.updCoordinate()
    q0.setName("q0")
    j1 = osim.PinJoint("j1",
        b0, osim.Vec3(0), osim.Vec3(0), b1, osim.Vec3(-1, 0, 0), osim.Vec3(0))
    q1 = j1.updCoordinate()
    q1.setName("q1")
    model.addJoint(j0)
    model.addJoint(j1)

    tau0 = osim.CoordinateActuator()
    tau0.setCoordinate(j0.updCoordinate())
    tau0.setName("tau0")
    tau0.setOptimalForce(1)
    model.addComponent(tau0)

    tau1 = osim.CoordinateActuator()
    tau1.setCoordinate(j1.updCoordinate())
    tau1.setName("tau1")
    tau1.setOptimalForce(1)
    model.addComponent(tau1)

    # Add display geometry.
    bodyGeometry = osim.Ellipsoid(0.5, 0.1, 0.1)
    transform = osim.Transform(osim.Vec3(-0.5, 0, 0))
    b0Center = osim.PhysicalOffsetFrame("b0_center", b0, transform)
    b0.addComponent(b0Center)
    b0Center.attachGeometry(bodyGeometry.clone())
    b1Center = osim.PhysicalOffsetFrame("b1_center", b1, transform)
    b1.addComponent(b1Center)
    b1Center.attachGeometry(bodyGeometry.clone())

    model.finalizeConnections()
    model.printToXML("double_pendulum.osim")
    return model

    
def solvePrediction():
    # Predict the optimal trajectory for a minimum time swing-up.
    # In the diagram below, + represents the origin, and ---o represents a link
    # in the double pendulum.
    #
    #                              o
    #                              |
    #                              o
    #                              |
    #         +---o---o            +
    #
    #       iniital pose      final pose
    #
    study = osim.MocoStudy()
    study.setName("double_pendulum_predict")

    problem = study.updProblem()

    # Model (dynamics).
    problem.setModel(createDoublePendulumModel())

    # Bounds.
    problem.setTimeBounds(0, [0, 5])
    # Arguments are name, [lower bound, upper bound],
    #                     initial [lower bound, upper bound],
    #                     final [lower bound, upper bound].
    problem.setStateInfo("/jointset/j0/q0/value", [-10, 10], 0)
    problem.setStateInfo("/jointset/j0/q0/speed", [-50, 50], 0, 0)
    problem.setStateInfo("/jointset/j1/q1/value", [-10, 10], 0)
    problem.setStateInfo("/jointset/j1/q1/speed", [-50, 50], 0, 0)
    problem.setControlInfo("/tau0", [-100, 100])
    problem.setControlInfo("/tau1", [-100, 100])

    # Cost: minimize final time and error from desired
    #       end effector position.
    ftCost = osim.MocoFinalTimeGoal()
    ftCost.setWeight(0.001)
    problem.addGoal(ftCost)

    finalCost = osim.MocoMarkerFinalGoal()
    finalCost.setName("final")
    finalCost.setWeight(1000.0)
    finalCost.setPointName("/markerset/m1")
    finalCost.setReferenceLocation(osim.Vec3(0, 2, 0))
    problem.addGoal(finalCost)


    # Configure the solver.
    solver = study.initTropterSolver()
    solver.set_num_mesh_intervals(100)
    solver.set_verbosity(2)
    solver.set_optim_solver("ipopt")

    guess = solver.createGuess()
    guess.setNumTimes(2)
    guess.setTime([0, 1])
    guess.setState("/jointset/j0/q0/value", [0, -math.pi])
    guess.setState("/jointset/j1/q1/value", [0, 2*math.pi])
    guess.setState("/jointset/j0/q0/speed", [0, 0])
    guess.setState("/jointset/j1/q1/speed", [0, 0])
    guess.setControl("/tau0", [0, 0])
    guess.setControl("/tau1", [0, 0])
    guess.resampleWithNumTimes(10)
    solver.setGuess(guess)

    # Save the problem to a setup file for reference.
    study.printToXML("examplePredictAndTrack_predict.omoco")

    # Solve the problem.
    solution = study.solve()
    solution.write("examplePredictAndTrack_predict_solution.sto")

    if visualize:
        study.visualize(solution)
    return solution
    

def computeMarkersReference(predictedSolution):
    model = createDoublePendulumModel()
    model.initSystem()
    states = predictedSolution.exportToStatesTable()

    statesTraj = osim.StatesTrajectory.createFromStatesTable(model, states)
    
    markerTrajectories = osim.TimeSeriesTableVec3()
    markerTrajectories.setColumnLabels(["/markerset/m0", "/markerset/m1"])

    for state in statesTraj:
        model.realizePosition(state)
        m0 = model.getComponent("markerset/m0")
        m1 = model.getComponent("markerset/m1")
        markerTrajectories.appendRow(state.getTime(),
            osim.RowVectorVec3([m0.getLocationInGround(state),
                                  m1.getLocationInGround(state)]))
                                  
    # Assign a weight to each marker.
    markerWeights = osim.SetMarkerWeights()
    markerWeights.cloneAndAppend(osim.MarkerWeight("/markerset/m0", 1))
    markerWeights.cloneAndAppend(osim.MarkerWeight("/markerset/m1", 5))
    
    return osim.MarkersReference(markerTrajectories, markerWeights)

    
def solveStateTracking(stateRef):
    # Predict the optimal trajectory for a minimum time swing-up.
    study = osim.MocoStudy()
    study.setName("double_pendulum_track")

    problem = study.updProblem()

    # Model (dynamics).
    problem.setModel(createDoublePendulumModel())

    # Bounds.
    # Arguments are name, [lower bound, upper bound],
    #                     initial [lower bound, upper bound],
    #                     final [lower bound, upper bound].
    finalTime = stateRef.getIndependentColumn()[-1]
    problem.setTimeBounds(0, finalTime)
    problem.setStateInfo("/jointset/j0/q0/value", [-10, 10], 0)
    problem.setStateInfo("/jointset/j0/q0/speed", [-50, 50], 0)
    problem.setStateInfo("/jointset/j1/q1/value", [-10, 10], 0)
    problem.setStateInfo("/jointset/j1/q1/speed", [-50, 50], 0)
    problem.setControlInfo("/tau0", [-150, 150])
    problem.setControlInfo("/tau1", [-150, 150])

    # Cost: track provided state data.
    stateTracking = osim.MocoStateTrackingGoal()
    stateTracking.setReference(osim.TableProcessor(stateRef))
    problem.addGoal(stateTracking)

    effort = osim.MocoControlGoal()
    effort.setName("effort")
    effort.setWeight(0.001)
    # TODO problem.addGoal(effort)

    # Configure the solver.
    solver = study.initTropterSolver()
    solver.set_num_mesh_intervals(50)
    solver.set_verbosity(2)
    solver.set_optim_solver("ipopt")
    solver.set_optim_jacobian_approximation("exact")
    solver.set_optim_hessian_approximation("exact")
    solver.set_exact_hessian_block_sparsity_mode("dense")

    # Save the problem to a setup file for reference.
    study.printToXML("examplePredictAndTrack_track_states.omoco")

    # Solve the problem.
    solution = study.solve()

    solution.write("examplePredictAndTrack_track_states_solution.sto")

    if visualize:
        study.visualize(solution)
    return solution

    
def solveMarkerTracking(markersRef, guess):
    # Predict the optimal trajectory for a minimum time swing-up.
    study = osim.MocoStudy()
    study.setName("double_pendulum_track")

    problem = study.updProblem()

    # Model (dynamics).
    problem.setModel(createDoublePendulumModel())

    # Bounds.
    # Arguments are name, [lower bound, upper bound],
    #                     initial [lower bound, upper bound],
    #                     final [lower bound, upper bound].
    finalTime = markersRef.getMarkerTable().getIndependentColumn()[-1]
    problem.setTimeBounds(0, finalTime)
    problem.setStateInfo("/jointset/j0/q0/value", [-10, 10], 0)
    problem.setStateInfo("/jointset/j0/q0/speed", [-50, 50], 0)
    problem.setStateInfo("/jointset/j1/q1/value", [-10, 10], 0)
    problem.setStateInfo("/jointset/j1/q1/speed", [-50, 50], 0)
    problem.setControlInfo("/tau0", [-100, 100])
    problem.setControlInfo("/tau1", [-100, 100])

    # Cost: track provided marker data.
    markerTracking = osim.MocoMarkerTrackingGoal()
    markerTracking.setMarkersReference(markersRef)
    problem.addGoal(markerTracking)
    
    effort = osim.MocoControlGoal()
    effort.setName("effort")
    effort.setWeight(0.0001)
    # problem.addGoal(effort)

    # Configure the solver.
    solver = study.initTropterSolver()
    solver.set_num_mesh_intervals(50)
    solver.set_verbosity(2)
    solver.set_optim_solver("ipopt")
    solver.set_optim_jacobian_approximation("exact")
    solver.set_optim_hessian_approximation("exact")
    solver.set_exact_hessian_block_sparsity_mode("dense")
    
    solver.setGuess(guess)

    # Save the problem to a setup file for reference.
    study.printToXML("examplePredictAndTrack_track_markers.omoco")

    # Solve the problem.
    solution = study.solve()

    solution.write("examplePredictAndTrack_track_markers_solution.sto")

    if visualize:
        study.visualize(solution)
    return solution
    

    
    
optimalTrajectory = solvePrediction()

markersRef = computeMarkersReference(optimalTrajectory)

trackedSolution = solveStateTracking(optimalTrajectory.exportToStatesTable())

trackedSolution2 = solveMarkerTracking(markersRef, trackedSolution)

















