// main.cpp

/* Copyright (c)  2006 Stanford University
 * Use of the OpenSim software in source form is permitted provided that the following
 * conditions are met:
 *   1. The software is used only for non-commercial research and education. It may not
 *     be used in relation to any commercial activity.
 *   2. The software is not distributed or redistributed.  Software distribution is allowed 
 *     only through https://simtk.org/home/opensim.
 *   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
 *      presentations, or documents describing work in which OpenSim or derivatives are used.
 *   4. Credits to developers may not be removed from executables
 *     created from modifications of the source.
 *   5. Modifications of source code must retain the above copyright notice, this list of
 *     conditions and the following disclaimer. 
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 *  Below is an example of an OpenSim application that provides its own 
 *  main() routine.  This application is a forward simulation of falling block with 
 *  four LinearSetPoint contacts which model the interaction of the block striking
 *  the ground. The parameters for the block and the contact elements are specified
 *  in the block.osim data file.
 */

// Author:  Jack Middleton 

//==============================================================================
//==============================================================================
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ActuatorSet.h>
#include <OpenSim/Simulation/Model/ContactForceSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/Contact.h>
#include <OpenSim/Analyses/Kinematics.h>
#include "SimTKsimbody.h" 



using namespace OpenSim;
using namespace std;

//______________________________________________________________________________
/**
 * Run a simulation with a falling block acted on by contact elements
 */
int main()
{
	// STEP 1
	// Set output precision
	IO::SetPrecision(8);
	IO::SetDigitsPad(-1);

	// STEP 2
	// Create the model
	Model model("block.osim");
    model.setup();

	// STEP 3
	// Alter the initial states if desired.
	// Initial states would normally be specified in a file
	Array<double> yi(0.0,model.getNumStates());
	model.getInitialStates(&yi[0]);
	yi[1] = 0.25;	// Y Position of block center of mass (com)
	yi[7] = 1.0;	// X Velocity of block com
	yi[9] = 0.0;	// Z Velocity of block com
	model.setInitialStates(&yi[0]);

	// STEP 4
	// Add analyses to the model.
	// This analysis will gather information during a simulation
	// without altering the simulation.
	// stepInterval specifies how frequently analyses will record info,
	// every four integration steps in this case.
	int stepInterval = 4;
	// Kinematics
	Kinematics *kin = new Kinematics(&model);
	kin->setStepInterval(stepInterval);
	model.addAnalysis(kin);


	// STEP 5
	// print out some info about the model
	model.printDetailedInfo(cout);

	// STEP 6
	// Construct the integrand and the manager.
	// The model integrand is what is integrated during the simulation.
	// The manager takes care of a variety of low-level initializations.
	ModelIntegrand *integrand = new ModelIntegrand(&model);
	Manager manager(integrand);


	// STEP 7
	// Specify the initial and final times of the simulation.
	double ti=0.0;
    double tf=2.0;
	manager.setInitialTime(ti);
	manager.setFinalTime(tf);

	// STEP 8
	// Set up the numerical integrator.
	int maxSteps = 20000;
	IntegRKF *integ = manager.getIntegrator();
	integ->setMaximumNumberOfSteps(maxSteps);
    integ->setUseConstantDT( true );
    integ->setDT( 1.0e-2 );
	integ->setTolerance(1.0e-7);
	integ->setFineTolerance(5.0e-9);

	// STEP 9
	// Integrate
	cout<<"\n\nIntegrating from "<<ti<<" to "<<tf<<endl;
	manager.integrate();

	// STEP 10
	// Print the analysis results.
	model.getAnalysisSet()->printResults("fallingBlock","./");

	return 0;
}
