
=============
MAIN PROGRAMS
=============

evaluateAndCompareSimulations.m
-------------------------------
Plots kinematics, kinetics, activations, and any other
quantities of interest from one or more simulations for
easy comparison between like quantities from simulations

Calls:
- plot_dataFromMotOrStoFiles.m
  . Single plot made by each call to this function
  . For gencoords, joint moments
- plot_multipleFiguresFromMotOrStoFiles.m
  . Multiple plots made by each call to this function,
    which avoids having to read an input file multiple
    times to make multiple plots from various columns
    of the same file (good for a large file like the
    states file)
  . For activations

make_plotsFromMotOrStoFiles.m
-----------------------------
Makes a single plot of whatever columns from whatever
storage/motion files.  So can plot similar quantities
from a single simulation, or same quantities from
multiple simulations against each other, or some
combination of these.

Calls:
- plot_dataFromMotOrStoFiles.m

make_plotsOfJointMoments.m
--------------------------
Makes a single plot of knee joint moments from two storage
files: before simulation (inverse dynamics after IK) and
after simulation (inverse dynamics after CMC).

Calls:
- plot_dataFromMotOrStoFiles.m

compute_netMuscleMoments.m
--------------------------
Creates the file
<resultsMuscleAnalysisDir>\<name>_NET_Muscle_Moments.sto,
containing the net muscle moments from the output of a
muscle analysis run.

=============================
KEY PLOTTING HELPER FUNCTIONS
=============================

plot_multipleFiguresFromMotOrStoFiles.m
---------------------------------------
Creates desired number of figures with the given
corresponding titles, and plots the curves from each
input file that have the given column labels (can have
different column labels for curves on the same plot).

Calls:
- read_motionFile.m

plot_dataFromMotOrStoFiles.m
----------------------------
Makes a single plot of the curves from the given columns
of the given storage/motion files.

Calls:
- get_timeAndDataColumns.m

===============
REFERENCE FILES
===============

muscleLabels.m: list of various muscle names and groups
