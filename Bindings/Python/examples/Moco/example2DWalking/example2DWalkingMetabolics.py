# -*- coding: utf-8 -*-
"""
MOCO: WALKING 2D EXAMPLE - COORDINATE TRACKING, MINIMISE METABOLIC COST

@author: Prasanna Sritharan, June 2022

This is a Python implementation of an example optimal control
problem (2D walking) orginally created in C++ by Antoine Falisse
(see: example2DWalkingMetabolics.cpp) and adapted for Matlab by Brian Umberger
(see: example2DWalkingMetabolics.m).

This example features a tracking simulation of walking that includes
minimisation of the metabolic cost of transport computed using a smooth
approximation of the metabolic energy model of Bhargava et al (2004).

Set a coordinate tracking problem where the goal is to minimize the
difference between provided and simulated coordinate values and speeds
as well as to minimize an effort cost (squared controls) and a metabolic
cost (metabolic energy normalized by distance traveled and body mass;
the metabolics model is based on a smooth approximation of the
phenomenological model described by Bhargava et al. (2004)). The provided
data represents half a gait cycle. Endpoint constraints enforce periodicity
of the coordinate values (except for pelvis tx) and speeds, coordinate
actuator controls, and muscle activations.
"""


from math import pi
import opensim as osim



# %% SET THE MODEL AND METABOLICS ANALYSIS

# Get the OpenSim model
model = osim.Model("2D_gait.osim")

# Add a component to calculate metabolics
metabolics = osim.Bhargava2004SmoothedMuscleMetabolics()
metabolics.setName("metabolics")
metabolics.set_use_smoothing(True)

# Add muscles to metabolics component
leg = ["r", "l"]
musclist = ["hamstrings", "bifemsh", "glut_max", "iliopsoas", "rect_fem", 
            "vasti", "gastroc", "soleus", "tib_ant"]
for mc in [m + "_" + l for m in musclist for l in leg]:
    metabolics.addMuscle(mc, osim.Muscle.safeDownCast(model.getComponent(mc)))

# Add metabolics component to model
model.addComponent(metabolics)
model.finalizeConnections()



# %% DEFINE THE OPTIMAL CONTROL PROBLEM

# Create the tracking problem
track = osim.MocoTrack()
track.setName("gait_tracking_met_cost")

# Pass the model to MocoTrack
modelProcessor = osim.ModelProcessor(model)
track.setModel(modelProcessor)

# Reference data for coordinate tracking
tableProcessor = osim.TableProcessor("referenceCoordinates.sto")
tableProcessor.append(osim.TabOpLowPassFilter(6.0))
track.setStatesReference(tableProcessor)

# Remaining MocoTrack settings
track.set_states_global_tracking_weight(30)
track.set_allow_unused_references(True)
track.set_track_reference_position_derivatives(True)
track.set_apply_tracked_states_to_guess(True)
track.set_initial_time(0.0)
track.set_final_time(0.47008941)

# Call initialize() to get the internal MocoStudy. The MocoStudy is comprised
# of a MocoProblem and a MocoSolver. Get the MocoProblem to configure further.
study = track.initialize()
problem = study.updProblem()



# %% SET GOALS

# Symmetry:
# This goal allows us to simulate only one step with left-right symmetry that
# we can then double to create two steps, one on each leg (IFO>IFS>CFO>CFS).
# Note that this is not actually a full gait cycle (IFO>IFS>CFO>CFS>IFO).
symmetryGoal = osim.MocoPeriodicityGoal("symmetry_goal")
problem.addGoal(symmetryGoal)

# Enforce periodic symmetry
model = modelProcessor.process()
model.initSystem()
state_names = [model.getStateVariableNames().getitem(sn) for sn in range(model.getNumStateVariables())]
for sn in state_names:
    
    # Symmetric coordinate values and speeds (except for pelvis_tx value):
    # Here we constrain final coordinate values of one leg to match the initial
    # value of the other leg. Manually add pelvis_tx speed later.
    if "jointset" in sn:
        if "_r" in sn:
            symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, sn.replace("_r", "_l")))
        elif "_l" in sn:
            symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, sn.replace("_l", "_r")))
        elif "pelvis_tx" not in sn:
            symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn))

    # Symmetric muscle activations:
    # Here, we constrain final muscle activation values of one leg to match the
    # initial activation values of the other leg.
    elif "activation" in sn:
        if "_r" in sn:
            symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, sn.replace("_r", "_l"))) 
        elif "_l" in sn:
            symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, sn.replace("_l", "_r")))


# The pelvis_tx speed has periodic symmetry
symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair("/jointset/groundPelvis/pelvis_tx/speed"))   

# Lumbar control has periodic symmetry. The other controls don't need symmetry
# enforced as they are all muscle excitations. Their behaviour will be
# modulated by the periodicity imposed on their respective activations.
symmetryGoal.addControlPair(osim.MocoPeriodicityGoalPair('/lumbarAct'))


# Effort: 
# Get a reference to the MocoControlGoal that is added to a MocoTrack 
# problem by default and adjust the weight
effort = osim.MocoControlGoal().safeDownCast(problem.updGoal("control_effort"))
effort.setWeight(0.1)


# Metabolic cost:
# Total metabolic rate includes activation heat rate, maintenance heat rate,
# shortening heat rate, mechanical work rate, and basal metabolic rate.
met_weight = 0.1
metabolicsGoal = osim.MocoOutputGoal("metabolics", met_weight)
metabolicsGoal.setOutputPath("/metabolics|total_metabolic_rate")
metabolicsGoal.setDivideByDisplacement(True)
metabolicsGoal.setDivideByMass(True)
problem.addGoal(metabolicsGoal)



# %% SET BOUNDS
         
# Coordinate bounds as dict
coord_bounds = {}
coord_bounds["/jointset/groundPelvis/pelvis_tilt/value"] = [-20*pi/180, -10*pi/180]
coord_bounds["/jointset/groundPelvis/pelvis_tx/value"] = [0.0, 1.0]
coord_bounds["/jointset/groundPelvis/pelvis_ty/value"] = [0.75, 1.25]
coord_bounds["/jointset/hip_r/hip_flexion_r/value"] = [-10*pi/180, 60*pi/180]
coord_bounds["/jointset/hip_l/hip_flexion_l/value"] = [-10*pi/180, 60*pi/180]
coord_bounds["/jointset/knee_r/knee_angle_r/value"] = [-50*pi/180, 0]
coord_bounds["/jointset/knee_l/knee_angle_l/value"] = [-50*pi/180, 0]
coord_bounds["/jointset/ankle_r/ankle_angle_r/value"] = [-15*pi/180, 25*pi/180]
coord_bounds["/jointset/ankle_l/ankle_angle_l/value"] = [-15*pi/180, 25*pi/180]
coord_bounds["/jointset/lumbar/lumbar/value"] = [0, 20*pi/180]

# Set coordinate bounds
for bnd in coord_bounds:
    problem.setStateInfo(bnd, coord_bounds[bnd]);



# %% SOLVE

# Configure the solver
solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(50)
solver.set_verbosity(2)
solver.set_optim_solver("ipopt")
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-4)
solver.set_optim_max_iterations(10000)

# Solve the problem for a single step
solution = study.solve()


# Create two mirrored steps from the single step solution, see API 
# documentation for use of this utility function
two_steps_solution = osim.createPeriodicTrajectory(solution)
two_steps_solution.write("walk_2D_metabolics_two_steps_tracking_solution.sto")

# Also extract the predicted ground forces, see API documentation for use of
# this utility function
contact_r = ["contactHeel_r", "contactFront_r"]
contact_l = ["contactHeel_l", "contactFront_l"]
contact_forces_table = osim.createExternalLoadsTableForGait(model, two_steps_solution, contact_r, contact_l)
osim.STOFileAdapter().write(contact_forces_table, "walk_2D_metabolics_two_steps_tracking_ground_forces.sto")


# Determine the cost of transport from the metabolics cost term. Need to divide
# by the weight applied earlier.
COT = solution.getObjectiveTerm("metabolics") / met_weight
print("\nThe metabolic cost of transport is: %f" % COT)


# visualise
study.visualize(two_steps_solution)