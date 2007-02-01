Contents:
~~~~~~~~~
Gait2392 contains a gait model with 23 degrees of freedom and 92 muscles.  The
setup files in this directory are tuned for more accurate results and a long
simulation time.

Gait2354 contains a gait model with 23 degrees of freedom and 54 muscles.  The
setup files in this directory are tuned for speed and a shorter simulation
time.  You should run through this example first to get familiar with SimTrack.

Running an example through SimTrack:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Enter the gait directory of your choice (Gait2354 or Gait2392; Gait2354 is
recommended for your first run through).  Then run the following commands in
order:

scale -S subject01_Setup_Scale.xml
	(Creates subject-specific model by scaling generic model)

makeSDFastModel -IM subject01.osim -OM subject01_sdfast.osim
	(Creates a dynamic SD/Fast-based model for dynamic simulation)

ik -S subject01_Setup_IK.xml
	(Computes inverse kinematics solution)

cmcgait -S subject01_Setup_RRA1.xml
	(Computes average residuals for residual reduction algorithm and generates a model with
	adjusted mass parameters)

cmcgait -S subject01_Setup_RRA2.xml
	(Computes adjusted kinematics which yield reduced residuals)

cmcgait -S subject01_Setup_CMC.xml
	(Generates muscle-actuated simulation)

forward -S subject01_Setup_Forward.xml
	(Uses CMC solution to drive a muscle-driven forward dynamics simulation)

perturb -S subject01_Setup_Perturb.xml
	(Computes induced accelerations by perturbing muscle forces)
