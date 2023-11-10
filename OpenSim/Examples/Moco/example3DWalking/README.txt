OpenSim Moco: example3DWalking
==============================

This folder contains examples for simulating walking with a model containing
80 lower-limb muscles.

Data and model sources:
- https://simtk.org/projects/full_body
- https://simtk.org/projects/model-high-flex
- https://simtk.org/projects/fbmodpassivecal

Model
-----
The model described in the file 'subject_walk_scaled.osim' is a modified
version of the Rajagopoal et al. 2016 musculoskeletal model. The passive forces
and muscle-tendon paths have been modified based on Uhlrich et al. 2022 and
Lai et al. 2017. The subtalar, mtp, and wrist coordinates have been replaced with
WeldJoints and residual actuators have been added to the pelvis. The model was
fit to the marker trajectories and ground reaction forces using AddBiomechanics.

Data
----
The coordinate and marker data included in the 'coordinates.sto' and
'marker_trajectories.trc' files also come from the Rajagopal et al. 2016
model distribution. The coordinates were computed using inverse kinematics
and modified via the Residual Reduction Algorithm (RRA).

The electromyography data in 'electromyography.sto' is based on the file
emg_walk_raw.anc from the Rajagopal et al. 2016 model distribution. The data
is bandpassed filtered (50 Hz to 500 Hz; 6th order Butterworth), rectified,
lowpass filtered (7.5 Hz; 4th order Butterworth),
and then filtered with a critically damped filter (15 Hz; 4th order).
Finally, each column is normalized so its maximum value is 1.0.

