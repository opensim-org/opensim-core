import os
import opensim

def getStepStatistics(results_folder, force_threshold = 10, minimum_step_time = 0.5):
	# find states file and model file
    for fname in os.listdir(results_folder):
        if fname.endswith('states.sto'):
            states_file = os.path.join(results_folder, fname)
    
    for fname in os.listdir(results_folder):
        if fname.endswith('.osim'):
            model_file = os.path.join(results_folder, fname)
	
	# get ground contact information from HuntCrossleyForce's
	model = opensim.Model(model_file)
	states = opensim.States(states_file)
	analyzeTool = opensim.AnalyzeTool()
	analyzeTool.setModel(model)
	analyzeTool.setStates(states)
    left_foot_forces = opensim.getForce('foot_l').getOutput() #<HuntCrossleyForce name="foot_l">
	right_foot_forces = opensim.getForce('foot_r').getOutput() #<HuntCrossleyForce name="foot_r">
	analyzeTool.add(leftFootForces)
	analyzeTool.add(rightFootForces)
    analyzeTool.run()
    
	# get initial contact times for each foot
    left_strike_times = opensim.getFootStrikeTimes(left_foot_forces, force_threshold, minimum_step_time)
	right_strike_times = opensim.getFootStrikeTimes(right_foot_forces, force_threshold, minimum_step_time)
	
	# get relevant walking statistics
	heel_l = opensim.getJoint('ankle_l').getJointCenterPoint()
	heel_r = opensim.getJoint('ankle_r').getJointCenterPoint()
	speed, step_length, cadence = opensim.getWalkingStatistics(left_strike_times, right_strike_times, heel_l, heel_r)
	