OpenSim Moco example: 3-D walking
=================================

This folder contains examples for simulating walking with a model containing
80 lower-limb muscles.

Data and model source: https://simtk.org/projects/full_body

Model
-----
The model described in the file 'subject_walk_armless.osim' is a modified
version of the Rajagopoal et al. 2016 musculoskeletal model. The lumbar,
subtalar, and mtp coordinates have been replaced with WeldJoints and residual
actuators have been added to the pelvis (1 N-m for rotational coordinates and 10
N for translational coordinates). Finally, the arms and all associated
components have been removed for simplicity.

Data
----
The coordinate and marker data included in the 'coordinates.sto' and
'marker_trajectories.trc' files also come from the Rajagopal et al. 2016
model distribution. The coordinates were computed using inverse kinematics
and modified via the Residual Reduction Algorithm (RRA).
