# -*- coding: utf-8 -*-
"""
MOCO: WALKING 2D EXAMPLE - TRACKING & PREDICTION

@author: Prasanna Sritharan, June 2022

This is a Python implementation of an example optimal control
problem (2D walking) orginally created in C++ by Antoine Falisse
(see: example2DWalking.cpp) and adapted for Matlab by Brian Umberger
(see: example2DWalking.m).
"""

from math import pi
import opensim as osim




"""
gait_tracking():
Set up a coordinate tracking problem where the goal is to minimize the
difference between provided and simulated coordinate values and speeds (and
ground reaction forces), as well as to minimize an effort cost (squared
controls). The provided data represents one step. Endpoint constraints
enforce periodicity of the coordinate values (except for pelvis tx) and speeds,
coordinate actuator controls, and muscle activations.
"""
def gait_tracking():

    

    # **********************************
    # DEFINE THE OPTIMAL CONTROL PROBLEM
        
    # Create tracking problem
    track = osim.MocoTrack()
    track.setName("gait_tracking")
    
    # Construct a ModelProcessor and set it on the tool
    modelProcessor = osim.ModelProcessor("2D_gait.osim")
    track.setModel(modelProcessor)
    
    
    # Get the coordinates into a TableProcessor
    tableProcessor = osim.TableProcessor("referenceCoordinates.sto")
    tableProcessor.append(osim.TabOpLowPassFilter(6.0))
    
    # Set the coordinates as the reference states
    track.setStatesReference(tableProcessor)
    track.set_allow_unused_references(True)
    
    # Only coordinates are provided so derive to get speeds and track these too
    track.set_track_reference_position_derivatives(True)
    
    # use coordinates and speeds for initial guess
    track.set_apply_tracked_states_to_guess(True)
    
    # Define the global state tracking weight
    track.set_states_global_tracking_weight(1.0)
    
    
    # Set initial time, final time and mesh interval
    track.set_initial_time(0.0)
    track.set_final_time(0.47008941)
    
    # Call initialize() to receive a pre-configured MocoStudy object based on
    # the settings above. Use this to customize the problem beyond the
    # Mocotrack interface. 
    study = track.initialize()
    
    # Get a writable reference to the MocoProblem from the MocoStudy to perform
    # more customisation
    problem = study.updProblem()
    
    
    
    # **********************************
    # SET GOALS
        
    # Symmetry:
    # This goal allows us to simulate only one step with left-right symmetry
    # that we can then double to create two steps, one on each leg 
    # (IFO>IFS>CFO>CFS). Note that this is not actually a full gait cycle 
    # (IFO>IFS>CFO>CFS>IFO).
    symmetryGoal = osim.MocoPeriodicityGoal("symmetry_goal")
    problem.addGoal(symmetryGoal)
    
    # Enforce periodic symmetry
    model = modelProcessor.process()
    model.initSystem()
    state_names = [model.getStateVariableNames().getitem(sn) 
                       for sn in range(model.getNumStateVariables())]
    for sn in state_names:
        
        # Symmetric coordinate values and speeds (except for pelvis_tx value):
        # Here we constrain final coordinate values of one leg to match the 
        # initial value of the other leg. Manually add pelvis_tx speed later.
        if "jointset" in sn:
            if "_r" in sn:
                symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, 
                           sn.replace("_r", "_l")))
            elif "_l" in sn:
                symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, 
                           sn.replace("_l", "_r")))
            elif "pelvis_tx" not in sn:
                symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn))
    
        # Symmetric muscle activations:
        # Here, we constrain final muscle activation values of one leg to match
        # the initial activation values of the other leg.
        elif "activation" in sn:
            if "_r" in sn:
                symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, 
                           sn.replace("_r", "_l"))) 
            elif "_l" in sn:
                symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, 
                           sn.replace("_l", "_r")))
                   
    
    # The pelvis_tx speed has periodic symmetry
    symmetryGoal.addStatePair(
        osim.MocoPeriodicityGoalPair("/jointset/groundPelvis/pelvis_tx/speed"))                
    
    # Lumbar control has periodic symmetry. The other controls don't need 
    # symmetry enforced as they are all muscle excitations. Their behaviour
    # will be contstrained by the periodicity imposed on their respective 
    # activations.
    symmetryGoal.addControlPair(osim.MocoPeriodicityGoalPair('/lumbarAct'))
    
    
    # Effort: 
    # Get a reference to the MocoControlGoal that is added to a MocoTrack 
    # problem by default and adjust the weight
    effort = osim.MocoControlGoal.safeDownCast(
                    problem.updGoal("control_effort"))
    effort.setWeight(10.0)
    
    
    # Ground contact: 
    # Track the left and right vertical and fore-aft GRFs
    contact_r = ["contactHeel_r", "contactFront_r"]
    contact_l = ["contactHeel_l", "contactFront_l"]
    grf_tracking_weight = 1
    if grf_tracking_weight != 0:
        
        # Create a contact tracking goal
        contactTracking = osim.MocoContactTrackingGoal("contact", 
                                                       grf_tracking_weight)
        contactTracking.setExternalLoadsFile("referenceGRF.xml")
        
        # Add contact groups. The sum of all the contact forces in each group
        # should track the force data from a single ExternalForce
        contactTracking.addContactGroup(contact_r, "Right_GRF")
        contactTracking.addContactGroup(contact_l, "Left_GRF")
        
        # 2D walking problem so consider force errors in the sagittal plane
        contactTracking.setProjection("plane")
        contactTracking.setProjectionVector(osim.Vec3(0, 0, 1))
        
        # add the goal to the MocoProblem
        problem.addGoal(contactTracking)
     
        
    
    # **********************************
    # SET BOUNDS
             
    # Coordinate bounds as dict
    coord_bounds = {}
    coord_bounds["/jointset/groundPelvis/pelvis_tilt/value"] = [-20*pi/180, 
                                                                -10*pi/180]
    coord_bounds["/jointset/groundPelvis/pelvis_tx/value"] = [0.0, 1.0]
    coord_bounds["/jointset/groundPelvis/pelvis_ty/value"] = [0.75, 1.25]
    coord_bounds["/jointset/hip_r/hip_flexion_r/value"] = [-10*pi/180, 
                                                           60*pi/180]
    coord_bounds["/jointset/hip_l/hip_flexion_l/value"] = [-10*pi/180, 
                                                           60*pi/180]
    coord_bounds["/jointset/knee_r/knee_angle_r/value"] = [-50*pi/180, 0]
    coord_bounds["/jointset/knee_l/knee_angle_l/value"] = [-50*pi/180, 0]
    coord_bounds["/jointset/ankle_r/ankle_angle_r/value"] = [-15*pi/180, 
                                                             25*pi/180]
    coord_bounds["/jointset/ankle_l/ankle_angle_l/value"] = [-15*pi/180, 
                                                             25*pi/180]
    coord_bounds["/jointset/lumbar/lumbar/value"] = [0, 20*pi/180]
    
    # Set coordinate bounds
    for bnd in coord_bounds:
        problem.setStateInfo(bnd, coord_bounds[bnd]);
        
    
    
    # **********************************
    # SOLVE
        
    # The Solver is pre-configured, however, uncomment below to configure 
    # further if desired. This gets a writable reference to the Solver, for 
    # custom configuration.
    # solver = study.updSolver()
    # solver.set_num_mesh_intervals(50);
    # solver.set_verbosity(2);
    # solver.set_optim_solver("ipopt");
    # solver.set_optim_convergence_tolerance(1e-4);
    # solver.set_optim_constraint_tolerance(1e-4);
    # solver.set_optim_max_iterations(1000);
    
    
    # Solve the problem for a single step
    solution = study.solve()
    
    
    # Create two mirrored steps from the single step solution, see API 
    # documentation for use of this utility function
    two_steps_solution = osim.createPeriodicTrajectory(solution)
    two_steps_solution.write("walk_2D_two_steps_tracking_solution.sto")
    
    # Also extract the predicted ground forces, see API documentation for use 
    # of this utility function
    contact_forces_table = osim.createExternalLoadsTableForGait(model, 
                            two_steps_solution, contact_r, contact_l)
    osim.STOFileAdapter().write(contact_forces_table, 
                            "walk_2D_two_steps_tracking_ground_forces.sto")
    

    return study, solution, two_steps_solution




"""
gait_prediction(tracking_solution):
Set up a gait prediction problem where the goal is to minimize effort
(squared controls) divided by distance traveled while enforcing symmetry of
the walking cycle and a prescribed average gait speed through endpoint
constraints. The solution of the coordinate tracking problem is used as an 
initial guess for the prediction.
"""
def gait_prediction(tracking_solution):

    
    
    # **********************************
    # DEFINE THE OPTIMAL CONTROL PROBLEM
       
    # Create predcition problem
    study = osim.MocoStudy()
    study.setName("gait_prediciton")
    
    # Get a writable reference to the MocoProblem from the MocoStudy to
    # customise the problem settings
    problem = study.updProblem()
    
    # Construct a ModelProcessor and set it on the Problem
    modelProcessor = osim.ModelProcessor("2D_gait.osim")
    problem.setModelProcessor(modelProcessor)
    
    
    
    # **********************************
    # SET GOALS
       
    # Symmetry: This goal allows us to simulate only one step with left-right
    # symmetry that we can then double to create two steps, one on each leg
    # (IFO>IFS>CFO>CFS). Note that this is not actually a full gait cycle
    # (IFO>IFS>CFO>CFS>IFO)
    symmetryGoal = osim.MocoPeriodicityGoal("symmetry_goal")
    problem.addGoal(symmetryGoal)
    
    
    # Enforce periodic symmetry
    model = modelProcessor.process()
    model.initSystem()
    state_names = [model.getStateVariableNames().getitem(sn) 
                       for sn in range(model.getNumStateVariables())]
    for sn in state_names:
        
        # Symmetric coordinate values and speeds (except for pelvis_tx value):
        # Here we constrain final coordinate values of one leg to match the 
        # initial value of the other leg. Manually add pelvis_tx speed later.
        if "jointset" in sn:
            if "_r" in sn:
                symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, 
                           sn.replace("_r", "_l")))
            elif "_l" in sn:
                symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, 
                           sn.replace("_l", "_r")))
            elif "pelvis_tx" not in sn:
                symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn))
    
        # Symmetric muscle activations:
        # Here, we constrain final muscle activation values of one leg to match
        # the initial activation values of the other leg.
        elif "activation" in sn:
            if "_r" in sn:
                symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, 
                           sn.replace("_r", "_l"))) 
            elif "_l" in sn:
                symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn, 
                           sn.replace("_l", "_r")))     
    
    # The pelvis_tx speed has periodic symmetry
    symmetryGoal.addStatePair(
        osim.MocoPeriodicityGoalPair("/jointset/groundPelvis/pelvis_tx/speed"))                
    
    # Lumbar control has periodic symmetry. The other controls don't need 
    # symmetry enforces as they are all muscle excitations. Their behaviour
    # will be modulated by the periodicity imposed on their activations.
    symmetryGoal.addControlPair(osim.MocoPeriodicityGoalPair('/lumbarAct'))
    
    # Specify the desired average walk speed, add as a goal
    speedGoal = osim.MocoAverageSpeedGoal('avg_speed')
    speedGoal.set_desired_average_speed(1.2)
    problem.addGoal(speedGoal)
    
    # Minimise total "effort" over the distance, i.e. minimise sum of the 
    # absolute values of all controls raised to a given exponent, integrated 
    # over the phase. In a MocoTrack problem, this is automatically added, 
    # however, in this predictive problem, we have created a MocoStudy from 
    # scratch, so we need to add this goal manually to the Problem. The second 
    # input argument to the constructor is the weight applied to this goal. 
    # We also normalise the effort by the distance travelled.
    effortGoal = osim.MocoControlGoal('effort', 10)
    effortGoal.setExponent(3)
    effortGoal.setDivideByDisplacement(True)
    problem.addGoal(effortGoal)
    
    
    
    # **********************************
    # SET BOUNDS
        
    # set time bounds
    problem.setTimeBounds(0, [0.4, 0.6])
    
    
    # Coordinate bounds as dict
    coord_bounds = {}
    coord_bounds["/jointset/groundPelvis/pelvis_tilt/value"] = [-20*pi/180, 
                                                                -10*pi/180]
    coord_bounds["/jointset/groundPelvis/pelvis_tx/value"] = [0.0, 1.0]
    coord_bounds["/jointset/groundPelvis/pelvis_ty/value"] = [0.75, 1.25]
    coord_bounds["/jointset/hip_r/hip_flexion_r/value"] = [-10*pi/180, 
                                                           60*pi/180]
    coord_bounds["/jointset/hip_l/hip_flexion_l/value"] = [-10*pi/180, 
                                                           60*pi/180]
    coord_bounds["/jointset/knee_r/knee_angle_r/value"] = [-50*pi/180, 0]
    coord_bounds["/jointset/knee_l/knee_angle_l/value"] = [-50*pi/180, 0]
    coord_bounds["/jointset/ankle_r/ankle_angle_r/value"] = [-15*pi/180, 
                                                             25*pi/180]
    coord_bounds["/jointset/ankle_l/ankle_angle_l/value"] = [-15*pi/180, 
                                                             25*pi/180]
    coord_bounds["/jointset/lumbar/lumbar/value"] = [0, 20*pi/180]
    
    # Set coordinate bounds
    for bnd in coord_bounds:
        problem.setStateInfo(bnd, coord_bounds[bnd])
    
    
    
    # **********************************
    # SOLVE
        
    # Configure the solver. In the tracking problem, the solver was 
    # pre-configured, using MocoTrack, however, as we have created a MocoStudy 
    # from scratch, we need to initialise the MocoSolver and configure it.
    solver = study.initCasADiSolver()
    solver.set_num_mesh_intervals(50)
    solver.set_verbosity(2)
    solver.set_optim_solver("ipopt")
    solver.set_optim_convergence_tolerance(1e-4)
    solver.set_optim_constraint_tolerance(1e-4)
    solver.set_optim_max_iterations(1000)
    
    # Set the tracking solution for one step as the initial guess
    solver.setGuess(tracking_solution)
    
    
    # Solve the problem for a single step
    solution = study.solve()
    
    
    # Create two mirrored steps from the single step solution, see API 
    # documentation for use of this utility function
    two_steps_solution = osim.createPeriodicTrajectory(solution)
    two_steps_solution.write("walk_2D_two_steps_prediction_solution.sto")
    
    # Also extract the predicted ground forces, see API documentation for use 
    # of this utility function
    contact_r = ["contactHeel_r", "contactFront_r"]
    contact_l = ["contactHeel_l", "contactFront_l"]
    contact_forces_table = osim.createExternalLoadsTableForGait(model, 
                            two_steps_solution, contact_r, contact_l)
    osim.STOFileAdapter().write(contact_forces_table, 
                            "walk_2D_two_steps_prediction_ground_forces.sto")


    return study, solution, two_steps_solution



# %% TRACKING PROBLEM

# Solve the problem and visualise
tracking_study, tracking_solution, tracking_two_steps_solution = \
                                                            gait_tracking()
tracking_study.visualize(tracking_two_steps_solution)



# %% PREDICTION PROBLEM

# Solve the problem and visualise (uses tracking_solution as initial guess)
prediction_study, prediction_solution, prediction_two_steps_solution = \
                                            gait_prediction(tracking_solution)
prediction_study.visualize(prediction_two_steps_solution)


