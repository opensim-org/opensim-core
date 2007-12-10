// ModelTestSuite.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "rdModelTestSuiteDLL.h"
#include <iostream>
#include <string>
//#include <OpenSim/Tools/Math.h>
#include <OpenSim/Simulation/Model/ActuatorSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ContactForceSet.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>
#include "rdModelTestSuite.h"





using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
ModelTestSuite::
ModelTestSuite()
{

}


//=============================================================================
// TEST
//=============================================================================
//__________________________________________________________________________
/**
 * Run all tests on a model.
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
Test(Model *aModel)
{
	if(aModel==NULL) {
		printf("ModelTestSuite.Test: ERROR- NULL model pointer.\n");
		return(false);
	}


	if ( (_outFPT=fopen("modelTestSuite.txt","w")) == NULL )
	{
		printf("ModelTestSuite.Test: ERROR- Cannot open output file.\n");
		return(false);
	}


	fprintf(_outFPT,"-------------------------- "
		"ModelTestSuite Results ---------------------------\n\n");

	// TESTS
	bool status;
	status = TestNumbers(aModel);
	status = TestNames(aModel);
	status = TestStates(aModel);
	status = TestPseudoStates(aModel);
	status = TestGravity(aModel);
	status = TestBodies(aModel);
	status = TestKinematics(aModel);
	status = TestLoads(aModel);
	status = TestDerivatives(aModel);
	status = TestMassMatrix(aModel);
	status = TestJacobian(aModel);
	status = TestOrientationUtilities(aModel);

	status = TestContacts(aModel);
	//status = TestActuation(aModel);


	fprintf(_outFPT,"\n\n----------------------- "
		"End of ModelTestSuite Results -----------------------");

	fclose(_outFPT);

	return(true);
}


//=============================================================================
// NUMBERS
//=============================================================================
//__________________________________________________________________________
/**
 * Test the various numbers of things in a model.
 *		Model::getNumControls()
 *		Model::getNumCoordinates()
 *		Model::getNumSpeeds()
 *		Model::getNumStates()
 *		Model::getNumBodies()
 *		Model::getNumJoints()
 *		Model::getNumActuators()
 *		Model::getNP()
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestNumbers(Model *aModel)
{
	if(aModel==NULL) {
		fprintf(_outFPT,
			"ModelTestSuite.TestNumbers: FAILED- NULL model pointer.\n");
		return(false);
	}

	// PRINT NUMBERS
	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestNumbers:\n");

	fprintf(_outFPT,"Controls    = %d\n",aModel->getNumControls());
	fprintf(_outFPT,"Coordinates = %d\n",aModel->getNumCoordinates());
	fprintf(_outFPT,"Speeds      = %d\n",aModel->getNumSpeeds());
	fprintf(_outFPT,"States      = %d\n",aModel->getNumStates());
	fprintf(_outFPT,"Bodies      = %d\n",aModel->getNumBodies());
	fprintf(_outFPT,"Joints      = %d\n",aModel->getNumJoints());
	fprintf(_outFPT,"Actuators   = %d\n",aModel->getNumActuators());
	fprintf(_outFPT,"Contacts      = %d\n",aModel->getNumContacts());

	fprintf(_outFPT,"\nModelTestSuite.TestNumbers: PASSED.\n"
		"========================================"
		"=======================================\n\n");
	return(true);
}


//=============================================================================
// NAMES
//=============================================================================
//__________________________________________________________________________
/**
 * Test the names of things in a model.
 *		Model::getName()
 *		Model::getBodyName()
 *		Model::getCoordinateName()
 *		Model::getSpeedName()
 *		Model::getActuatorName()
 *		Model::getControlName()
 *		Model::getStateName()
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestNames(Model *aModel)
{
	Array<string> allStateNames("");
	if(aModel==NULL) {
		fprintf(_outFPT,
			"ModelTestSuite.TestNames: FAILED- NULL model pointer.\n");
		return(false);
	}

	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestNames:\n");

	fprintf(_outFPT,"\nModel Name = '%s'",aModel->getName().c_str());

	int i;

	fprintf(_outFPT,"\n\n Body          Name\n------------------------------");
	for (i=0;i<aModel->getNumBodies();i++)
		fprintf(_outFPT,"\n%4d           '%s'",i,aModel->getDynamicsEngine().getBodySet()->get(i)->getName().c_str());
		//fprintf(_outFPT,"\n%4d           '%s'",i,aModel->getBodyName(i).c_str());
	fprintf(_outFPT,"\n------------------------------\n");


	fprintf(_outFPT,"\n\nCoordinate     Name\n------------------------------");
	for (i=0;i<aModel->getNumCoordinates();i++)
		fprintf(_outFPT,"\n%4d           '%s'",i,aModel->getDynamicsEngine().getCoordinateSet()->get(i)->getName().c_str());
	fprintf(_outFPT,"\n------------------------------\n");

	fprintf(_outFPT,"\n\nSpeed          Name\n------------------------------");
	for (i=0;i<aModel->getNumSpeeds();i++)
		fprintf(_outFPT,"\n%4d           '%s'",i,aModel->getDynamicsEngine().getSpeedSet()->get(i)->getName().c_str());
	fprintf(_outFPT,"\n------------------------------\n");

	fprintf(_outFPT,"\n\nActuator       Name\n------------------------------");
	for (i=0;i<aModel->getNumActuators();i++)
		fprintf(_outFPT,"\n%4d           '%s'",i,aModel->getActuatorSet()->get(i)->getName().c_str());
	fprintf(_outFPT,"\n------------------------------\n");

	fprintf(_outFPT,"\n\nControl        Name\n------------------------------");
	for (i=0;i<aModel->getNumControls();i++)
		fprintf(_outFPT,"\n%4d           '%s'",i,aModel->getControlName(i).c_str());
	fprintf(_outFPT,"\n------------------------------\n");

	fprintf(_outFPT,"\n\nState          Name\n------------------------------");
	
	aModel->getStateNames(allStateNames);
	for (i=0;i<aModel->getNumStates();i++)
		fprintf(_outFPT,"\n%4d           '%s'",i,allStateNames[i].c_str());
	fprintf(_outFPT,"\n------------------------------\n");

	fprintf(_outFPT,"\nModelTestSuite.TestNames: PASSED.\n"
		"========================================"
		"=======================================\n\n");


	return(true);
}


//=============================================================================
// STATES
//=============================================================================
//__________________________________________________________________________
/**
 * Test the methods for setting and getting the states of a model.
 *		Model::set()
 *		Model::setTime()
 *		Model::getTime()
 *		Model::setTimeNormConstant()
 *		Model::getTimeNormConstant()
 *		Model::setControls()
 *		Model::getControls()
 *		Model::setInitialStates()
 *		Model::getInitialStates()
 *		Model::setStates()
 *		Model::getStates()
 *		Model::setConfiguration()
 *		Model::getCoordinates()
 *		Model::getSpeeds()
 *		Model::extractConfiguration()
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestStates(Model *aModel)
{
	if(aModel==NULL) {
		fprintf(_outFPT,
			"ModelTestSuite.TestStates: FAILED- NULL model pointer.\n");
		return(false);
	}

	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestStates:\n");

	int i;
	Array<string> allStateNames("");

	// Set states, controls, and time.
	double *yi = new double[aModel->getNumStates()];

	for (i=0;i<aModel->getNumStates();i++)
		yi[i] = (double)i;

	double *xi = new double[aModel->getNumControls()];

	for (i=0;i<aModel->getNumControls();i++)
		xi[i] = -(double)i;

	double T = 12.3;

	aModel->set(T,xi,yi);



	// Get the values.
	double *rY = new double[aModel->getNumStates()];
	double *rX = new double[aModel->getNumControls()];

	// Get the states and the controls.
	aModel->getStates(rY);
	aModel->getControls(rX);

	fprintf(_outFPT,"\nSet Time, States, and Controls -----\n");

	fprintf(_outFPT,"\nTime = %.4lf seconds",aModel->getTime());

	fprintf(_outFPT,"\n\nState      Value      Name\n"
		"-------------------------------------");

	aModel->getStateNames(allStateNames);
	for (i=0;i<aModel->getNumStates();i++)
		fprintf(_outFPT,"\n%4d   %12.4e  '%s'",
		i,rY[i], allStateNames[i].c_str());

	fprintf(_outFPT,"\n-------------------------------------\n");

	fprintf(_outFPT,"\n\nControl    Value      Name\n"
		"-------------------------------------");

	for (i=0;i<aModel->getNumControls();i++)
		fprintf(_outFPT,"\n%4d   %12.4e  '%s'",
		i,rX[i],aModel->getControlName(i).c_str());

	fprintf(_outFPT,"\n-------------------------------------\n");


	fprintf(_outFPT,"\nReset Time, States, and Controls -----\n");

	T += 10.0;

	aModel->setTime(T);
	fprintf(_outFPT,"\nTime = %.4lf seconds",aModel->getTime());

	double Tnorm = 5.0;
	aModel->setTimeNormConstant(Tnorm);
	fprintf(_outFPT,"\nTime normalization constant = %.4lf",
		aModel->getTimeNormConstant());


	for (i=0;i<aModel->getNumStates();i++)
		yi[i] = (double)(i+10);

	aModel->setStates(yi);
	aModel->getStates(rY);

	fprintf(_outFPT,"\n\nState      Value      Name\n"
		"-------------------------------------");

	for (i=0;i<aModel->getNumStates();i++)
		fprintf(_outFPT,"\n%4d   %12.4e  '%s'",
		i,rY[i],allStateNames[i].c_str());

	fprintf(_outFPT,"\n-------------------------------------\n");


	for (i=0;i<aModel->getNumControls();i++)
		xi[i] = -(double)(i+10);

	aModel->setControls(xi);
	aModel->getControls(rX);

	fprintf(_outFPT,"\n\nControl    Value      Name\n"
		"-------------------------------------");

	for (i=0;i<aModel->getNumControls();i++)
		fprintf(_outFPT,"\n%4d   %12.4e  '%s'",
		i,rX[i],aModel->getControlName(i).c_str());

	fprintf(_outFPT,"\n-------------------------------------\n");




	fprintf(_outFPT,"\n\nConfiguration tests -----");

	Array<double> rQ(0.0,aModel->getNumCoordinates());
	
	fprintf(_outFPT,"\n\nCoordinate       Value\n"
		                "-------------------------");

	for (i=0;i<aModel->getNumCoordinates();i++){
		rQ[i] = aModel->getDynamicsEngine().getCoordinateSet()->get(i)->getValue();
		fprintf(_outFPT,"\n %4d        %12.4e",i,rQ[i]);
	}
	fprintf(_outFPT,"\n-------------------------\n");


	Array<double> rU(0.0,aModel->getNumSpeeds());

	fprintf(_outFPT,"\n\nSpeed      Value\n"
		"------------------");

	for (i=0;i<aModel->getNumSpeeds();i++){
		rU[i] = aModel->getDynamicsEngine().getSpeedSet()->get(i)->getValue();
		fprintf(_outFPT,"\n%3d   %12.4e",i,rU[i]);
	}
	fprintf(_outFPT,"\n------------------\n");


	fprintf(_outFPT,"\n\nExtracting configuration:");

	aModel->getDynamicsEngine().extractConfiguration(yi,&rQ[0],&rU[0]);

	fprintf(_outFPT,"\n\nCoordinate       Value\n"
		                "-------------------------");

	for (i=0;i<aModel->getNumCoordinates();i++)
		fprintf(_outFPT,"\n %4d        %12.4e",i,rQ[i]);

	fprintf(_outFPT,"\n-------------------------\n");

	fprintf(_outFPT,"\n\nSpeed      Value\n"
		"------------------");

	for (i=0;i<aModel->getNumSpeeds();i++)
		fprintf(_outFPT,"\n%3d   %12.4e",i,rU[i]);

	fprintf(_outFPT,"\n------------------\n");


	fprintf(_outFPT,"\n\nReset states:");

	for (i=0;i<aModel->getNumStates();i++)
		yi[i] = (double)(i+20);

	aModel->getDynamicsEngine().setConfiguration(yi);

	fprintf(_outFPT,"\n\nCoordinate       Value\n"
		                "-------------------------");

	for (i=0;i<aModel->getNumCoordinates();i++){
		rQ[i] = aModel->getDynamicsEngine().getCoordinateSet()->get(i)->getValue();
		fprintf(_outFPT,"\n %4d        %12.4e",i,rQ[i]);
	}
	fprintf(_outFPT,"\n-------------------------\n");


	fprintf(_outFPT,"\n\nSpeed      Value\n"
		"------------------");

	for (i=0;i<aModel->getNumSpeeds();i++){
		rU[i] = aModel->getDynamicsEngine().getSpeedSet()->get(i)->getValue();
		fprintf(_outFPT,"\n%3d   %12.4e",i,rU[i]);
	}
	fprintf(_outFPT,"\n------------------\n");


	fprintf(_outFPT,"\n\nReset coordinates and speeds:");

	for (i=0;i<aModel->getNumCoordinates();i++)
		rQ[i] = rQ[i] + 10.0;

	for (i=0;i<aModel->getNumSpeeds();i++)
		rU[i] = rU[i] + 20.0;

	aModel->getDynamicsEngine().setConfiguration(&rQ[0],&rU[0]);

	fprintf(_outFPT,"\n\nCoordinate       Value\n"
		                "-------------------------");

	for (i=0;i<aModel->getNumCoordinates();i++){
		rQ[i] = aModel->getDynamicsEngine().getCoordinateSet()->get(i)->getValue();
		fprintf(_outFPT,"\n %4d        %12.4e",i,rQ[i]);
	}
	fprintf(_outFPT,"\n-------------------------\n");

	fprintf(_outFPT,"\n\nSpeed      Value\n"
		"------------------");

	for (i=0;i<aModel->getNumSpeeds();i++){
		rU[i] = aModel->getDynamicsEngine().getSpeedSet()->get(i)->getValue();
		fprintf(_outFPT,"\n%3d   %12.4e",i,rU[i]);
	}
	fprintf(_outFPT,"\n------------------\n");



	fprintf(_outFPT,"\nModelTestSuite.TestStates: PASSED.\n"
		"========================================"
		"=======================================\n\n");


	delete yi;
	delete xi;

	delete rY;
	delete rX;


	return(true);
}


//=============================================================================
// PSEUDO-STATES
//=============================================================================
//__________________________________________________________________________
/**
 * Test the methods for setting and getting the pseudo-states of a model.
 *		Model::setInitialPseudoStates()
 *		Model::getInitialPseudoStates()
 *		Model::setPseudoStates()
 *		Model::getPseudoStates()
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestPseudoStates(Model *aModel)
{

	if(aModel==NULL) {
		fprintf(_outFPT,
			"ModelTestSuite.TestStates: FAILED- NULL model pointer.\n");
		return(false);
	}

	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestStates:\n");

	//----------------------
	// INITIAL PSEUDO-STATES
	//----------------------
	// CURRENT
	Array<double> ypi(0.0,aModel->getNumPseudoStates());
	aModel->getInitialPseudoStates(&ypi[0]);
	cout<<"ypi:  "<<ypi<<endl;

	
	//----------------------
	// PSEUDO-STATES
	//----------------------
	// CURRENT
	Array<double> yp(0.0,aModel->getNumPseudoStates());
	aModel->getPseudoStates(&yp[0]);
	cout<<"yp:  "<<yp<<endl;
	aModel->setPseudoStates(&ypi[0]);
	aModel->getPseudoStates(&yp[0]);
	cout<<"yp:  "<<yp<<endl;
	
	
	return(true);
}


//=============================================================================
// GRAVITY
//=============================================================================
//__________________________________________________________________________
/**
 * Test class rdGravity.
 *		Model::getGravity()
 *		Model::setGravity()
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestGravity(Model *aModel)
{
	if(aModel==NULL) {
		fprintf(_outFPT,
			"ModelTestSuite.TestGravity: FAILED- NULL model pointer.\n");
		return(false);
	}

	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestGravity:\n");


	// Get the states before we do anything with gravity.
	double *rY = new double[aModel->getNumStates()];

	aModel->getStates(rY);


	double rGrav[3],aGrav[3],aGravSave[3];

	fprintf(_outFPT,"\nGet gravity -----");

	aModel->getGravity(rGrav);

	aGravSave[0] = rGrav[0];
	aGravSave[1] = rGrav[1];
	aGravSave[2] = rGrav[2];

	fprintf(_outFPT,"\nGravity = %9.4lf  %9.4lf  %9.4lf",
		rGrav[0],rGrav[1],rGrav[2]);

	if (rGrav[0] < 0.0)
		aGrav[0] = rGrav[0] - 10.0;
	else
		aGrav[0] = rGrav[0] + 10.0;

	if (rGrav[1] < 0.0)
		aGrav[1] = rGrav[1] - 10.0;
	else
		aGrav[1] = rGrav[1] + 10.0;

	if (rGrav[2] < 0.0)
		aGrav[2] = rGrav[2] - 10.0;
	else
		aGrav[2] = rGrav[2] + 10.0;

	aModel->setGravity(aGrav);
	aModel->getGravity(rGrav);

	fprintf(_outFPT,"\n\nReset gravity -----");

	fprintf(_outFPT,"\nGravity = %9.4lf  %9.4lf  %9.4lf",
		rGrav[0],rGrav[1],rGrav[2]);

	fprintf(_outFPT,"\n\nReset gravity -----");

	aModel->setGravity(aGravSave);

	aModel->getGravity(rGrav);

	fprintf(_outFPT,"\nGravity = %9.4lf  %9.4lf  %9.4lf",
		rGrav[0],rGrav[1],rGrav[2]);

	// Reset the states.
	aModel->setStates(rY);

	delete rY;

	fprintf(_outFPT,"\n\nModelTestSuite.TestGravity: PASSED.\n"
		"========================================"
		"=======================================\n\n");


	return(true);
}


//=============================================================================
// BODIES
//=============================================================================
//__________________________________________________________________________
/**
 * Test class for bodies.
 *		Model::getGroundID()
 *		Model::getMass()
 *		Model::getInertiaBodyLocal()
 *		Model::getSystemInertia()
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestBodies(Model *aModel)
{
	if(aModel==NULL) {
		fprintf(_outFPT,
			"ModelTestSuite.TestBodies: FAILED- NULL model pointer.\n");
		return(false);
	}

	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestBodies:\n");

	fprintf(_outFPT,"\nGround ID = %d",aModel->getDynamicsEngine().getGroundBody());


	int i;


	fprintf(_outFPT,"\n\n Body      Mass\n"
		"------------------");

	for (i=0;i<aModel->getNumBodies();i++)
		fprintf(_outFPT,"\n%4d  %12.4e",i,aModel->getDynamicsEngine().getBodySet()->get(i)->getMass());

	fprintf(_outFPT,"\n------------------\n");

	double rI[3][3];

	for (i=0;i<aModel->getNumBodies();i++){

		aModel->getDynamicsEngine().getBodySet()->get(i)->getInertia(rI);

		fprintf(_outFPT,"\nInertia Matrix for Body: %d -----",i);

		fprintf(_outFPT,"\n         X              Y              Z"
			"\n---------------------------------------------");
		fprintf(_outFPT,"\nX  %12.4e   %12.4e   %12.4e",
			rI[0][0],rI[0][1],rI[0][2]);
		fprintf(_outFPT,"\nY  %12.4e   %12.4e   %12.4e",
			rI[1][0],rI[1][1],rI[1][2]);
		fprintf(_outFPT,"\nZ  %12.4e   %12.4e   %12.4e",
			rI[2][0],rI[2][1],rI[2][2]);

		fprintf(_outFPT,"\n---------------------------------------------\n");
	}


	double rM,rCOM[3];

	aModel->getDynamicsEngine().getSystemInertia(&rM,rCOM,rI);

	fprintf(_outFPT,"\n\nSystem Properties:\n");
	fprintf(_outFPT,"\nMass = %.3lf",rM);
	fprintf(_outFPT,"\n COM = %12.3e  %12.3e  %12.3e",rCOM[0],rCOM[1],rCOM[2]);


	fprintf(_outFPT,"\n\nInertia Tensor -----");

	fprintf(_outFPT,"\n         X              Y              Z"
		"\n---------------------------------------------");
	fprintf(_outFPT,"\nX  %12.4e   %12.4e   %12.4e",
		rI[0][0],rI[0][1],rI[0][2]);
	fprintf(_outFPT,"\nY  %12.4e   %12.4e   %12.4e",
		rI[1][0],rI[1][1],rI[1][2]);
	fprintf(_outFPT,"\nZ  %12.4e   %12.4e   %12.4e",
		rI[2][0],rI[2][1],rI[2][2]);

	fprintf(_outFPT,"\n---------------------------------------------\n");


	fprintf(_outFPT,"\n\nModelTestSuite.TestBodies: PASSED.\n"
		"========================================"
		"=======================================\n\n");


	return(true);
}


//=============================================================================
// KINEMATICS
//=============================================================================
//__________________________________________________________________________
/**
 * Test the methods for computing kinematic quantities for a model.
 *		Model::getPosition()
 *		Model::getVelocity()
 *		Model::getAcceleration()
 *		Model::getDirectionCosines()
 *		Model::getAngularVelocity()
 *		Model::getAngularVelocityBodyLocal()
 *		Model::getAngularAcceleration()
 *		Model::getAngularAccelerationBodyLocal()
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestKinematics(Model *aModel)
{
	if(aModel==NULL) {
		fprintf(_outFPT,
			"ModelTestSuite.TestKinematics: FAILED- NULL model pointer.\n");
		return(false);
	}

	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestKinematics:\n");

	int i;
	double aPoint[] = { 0.0, 0.0, 0.0 };

	fprintf(_outFPT,"\n\n Body                 Position"
					"                               Velocity\n"
"---------------------------------------------------------"
"----------------------");

	// CONFIGURATION
	Array<double> q(0.0);  q.setSize(aModel->getNumCoordinates());
	Array<double> u(0.0);  u.setSize(aModel->getNumSpeeds());
	aModel->getDynamicsEngine().setConfiguration(&q[0],&u[0]);

	BodySet *bodies = aModel->getDynamicsEngine().getBodySet();

	for (i=0;i<aModel->getNumBodies();i++)
	{
		double rPos[3],rVel[3];
		aModel->getDynamicsEngine().getPosition(*(*bodies)[i],aPoint,rPos);
		aModel->getDynamicsEngine().getVelocity(*(*bodies)[i],aPoint,rVel);

		fprintf(_outFPT,"\n%4d   %11.3e %11.3e %11.3e %11.3e %11.3e %11.3e",
			i,rPos[0],rPos[1],rPos[2],rVel[0],rVel[1],rVel[2]);
	}
	fprintf(_outFPT,"\n-----------------------------------------------------"
		"--------------------------\n");


	fprintf(_outFPT,"\n\n Body               Acceleration\n"
		"---------------------------------------------");


	int nq = aModel->getNumCoordinates();
	int nu = aModel->getNumSpeeds();

	double *dqdt = new double[nq];
	double *dudt = new double[nu];

	aModel->getDynamicsEngine().computeDerivatives(dqdt,dudt);

	delete dqdt;
	delete dudt;

	for (i=0;i<aModel->getNumBodies();i++)
	{
		double rAcc[3];
		aModel->getDynamicsEngine().getAcceleration(*(*bodies)[i],aPoint,rAcc);

		fprintf(_outFPT,"\n%4d   %12.4e %12.4e %12.4e",
			i,rAcc[0],rAcc[1],rAcc[2]);
	}
	fprintf(_outFPT,"\n---------------------------------------------\n");


	fprintf(_outFPT,"\n\n Body       Direction Cosines (Array)\n"
		"--------------------------------------");

	for (i=0;i<aModel->getNumBodies();i++)
	{
		double rDirCosArray[3][3];
		aModel->getDynamicsEngine().getDirectionCosines(*(*bodies)[i],rDirCosArray);

		fprintf(_outFPT,"\n%4d   %9.6lf  %9.6lf  %9.6lf",
			i,rDirCosArray[0][0],rDirCosArray[0][1],rDirCosArray[0][2]);
		fprintf(_outFPT,"\n       %9.6lf  %9.6lf  %9.6lf",
			rDirCosArray[1][0],rDirCosArray[1][1],rDirCosArray[1][2]);
		fprintf(_outFPT,"\n       %9.6lf  %9.6lf  %9.6lf",
			rDirCosArray[2][0],rDirCosArray[2][1],rDirCosArray[2][2]);

		if (i < (aModel->getNumBodies()-1) ) fprintf(_outFPT,"\n");
	}
	fprintf(_outFPT,"\n--------------------------------------\n");


	fprintf(_outFPT,"\n\n Body       Direction Cosines (Vector)\n"
		"--------------------------------------");

	double *rDirCosVector;
	rDirCosVector = new double[9];

	for (i=0;i<aModel->getNumBodies();i++)
	{
		aModel->getDynamicsEngine().getDirectionCosines(*(*bodies)[i],rDirCosVector);

		fprintf(_outFPT,"\n%4d   %9.6lf  %9.6lf  %9.6lf",
			i,rDirCosVector[0],rDirCosVector[1],rDirCosVector[2]);
		fprintf(_outFPT,"\n       %9.6lf  %9.6lf  %9.6lf",
			rDirCosVector[3],rDirCosVector[4],rDirCosVector[5]);
		fprintf(_outFPT,"\n       %9.6lf  %9.6lf  %9.6lf",
			rDirCosVector[6],rDirCosVector[7],rDirCosVector[8]);

		if (i < (aModel->getNumBodies()-1) ) fprintf(_outFPT,"\n");
	}
	fprintf(_outFPT,"\n--------------------------------------\n");

	delete rDirCosVector;


	fprintf(_outFPT,"\n\n Body            Angular Velocity (Global)        "
		"Resultant\n"
		"-------------------------------------------------------------");

	for (i=0;i<aModel->getNumBodies();i++)
	{
		double rAngVel[3],totalAngVel;
		aModel->getDynamicsEngine().getAngularVelocity(*(*bodies)[i],rAngVel);

		totalAngVel = sqrt(rAngVel[0]*rAngVel[0]+rAngVel[1]*rAngVel[2]+
			rAngVel[2]*rAngVel[2]);

		fprintf(_outFPT,"\n%4d   %12.4e %12.4e %12.4e   %12.4e",
			i,rAngVel[0],rAngVel[1],rAngVel[2],totalAngVel);
	}
	fprintf(_outFPT,"\n------------------------------"
		"-------------------------------\n");


	fprintf(_outFPT,"\n\n Body            Angular Velocity (Local)         "
		"Resultant\n"
		"-------------------------------------------------------------");

	for (i=0;i<aModel->getNumBodies();i++)
	{
		double rAngVel[3],totalAngVel;
		aModel->getDynamicsEngine().getAngularVelocityBodyLocal(*(*bodies)[i],rAngVel);

		totalAngVel = sqrt(rAngVel[0]*rAngVel[0]+rAngVel[1]*rAngVel[2]+
			rAngVel[2]*rAngVel[2]);

		fprintf(_outFPT,"\n%4d   %12.4e %12.4e %12.4e   %12.4e",
			i,rAngVel[0],rAngVel[1],rAngVel[2],totalAngVel);
	}
	fprintf(_outFPT,"\n------------------------------"
		"-------------------------------\n");


	fprintf(_outFPT,"\n\n Body        Angular Acceleration (Global)        "
		"Resultant\n"
		"------------------------------------------------------------");

	for (i=0;i<aModel->getNumBodies();i++)
	{
		double rAngAcc[3],totalAngAcc;
		aModel->getDynamicsEngine().getAngularAcceleration(*(*bodies)[i],rAngAcc);

		totalAngAcc = sqrt(rAngAcc[0]*rAngAcc[0]+rAngAcc[1]*rAngAcc[2]+
			rAngAcc[2]*rAngAcc[2]);

		fprintf(_outFPT,"\n%4d   %12.4e %12.4e %12.4e   %12.4e",
			i,rAngAcc[0],rAngAcc[1],rAngAcc[2],totalAngAcc);
	}
	fprintf(_outFPT,"\n------------------------------"
		"------------------------------\n");


	fprintf(_outFPT,"\n\n Body        Angular Acceleration (Local)         "
		"Resultant\n"
		"------------------------------------------------------------");

	for (i=0;i<aModel->getNumBodies();i++)
	{
		double rAngAcc[3],totalAngAcc;
		aModel->getDynamicsEngine().getAngularAccelerationBodyLocal(*(*bodies)[i],rAngAcc);

		totalAngAcc = sqrt(rAngAcc[0]*rAngAcc[0]+rAngAcc[1]*rAngAcc[2]+
			rAngAcc[2]*rAngAcc[2]);

		fprintf(_outFPT,"\n%4d   %12.4e %12.4e %12.4e   %12.4e",
			i,rAngAcc[0],rAngAcc[1],rAngAcc[2],totalAngAcc);
	}
	fprintf(_outFPT,"\n------------------------------"
		"------------------------------\n");



	fprintf(_outFPT,"\nModelTestSuite.TestKinematics: PASSED.\n"
		"========================================"
		"=======================================\n\n");

	return(true);
}



//=============================================================================
// LOADS
//=============================================================================
//__________________________________________________________________________
/**
 * Test class Loads.
 *		Model::applyForce()
 *		Model::getNetAppliedGeneralizedForce()
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestLoads(Model *aModel)
{
	if(aModel==NULL) {
		fprintf(_outFPT,
			"ModelTestSuite.TestLoads: FAILED- NULL model pointer.\n");
		return(false);
	}

	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestLoads:\n");

	int i,j;
	int nb = aModel->getNumBodies();
	int nu = aModel->getNumSpeeds();

	double rF;
	double aPoint[] = { 1.0, 0.0, 0.0 };
	double aForce[] = { 0.0, 1.0, 0.0 };

	BodySet *bodies = aModel->getDynamicsEngine().getBodySet();
	CoordinateSet *coords = aModel->getDynamicsEngine().getCoordinateSet();

	for (i=0;i<nb;i++)
	{
		fprintf(_outFPT,"\n\nBody: %d -----",i);

		aModel->getDynamicsEngine().applyForce(*(*bodies)[i],aPoint,aForce);

		fprintf(_outFPT,"\n\nGeneralized Speed   Force/Torque\n"
			"--------------------------------");

		for (j=0;j<nu;j++)
		{
			rF = aModel->getDynamicsEngine().getNetAppliedGeneralizedForce(*(*coords)[j]);
			fprintf(_outFPT,"\n    %3d             %12.4e",j,rF);
		}
		fprintf(_outFPT,"\n--------------------------------\n");
	}



	fprintf(_outFPT,"\n\nModelTestSuite.TestLoads: PASSED.\n"
		"========================================"
		"=======================================\n\n");

	return(true);
}


//=============================================================================
// DERIVATIVES
//=============================================================================
//__________________________________________________________________________
/**
 * Test class Derivatives.
 *		Model::computeAccelerations()
 *		Model::deriv()
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestDerivatives(Model *aModel)
{
	if(aModel==NULL) {
		fprintf(_outFPT,
			"ModelTestSuite.TestDerivatives: FAILED- NULL model pointer.\n");
		return(false);
	}

	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestDerivatives:\n");

	// ALLOCATIONS
	int nq = aModel->getNumCoordinates();
	int nu = aModel->getNumSpeeds();
	double *dqdt = new double[nq];
	double *dudt = new double[nu];


	// SET STATES
	Array<double> yi(0.0,aModel->getNumStates());
	aModel->getInitialStates(&yi[0]);
	cout<<yi<<endl;
	aModel->setStates(&yi[0]);

	// COMPUTE ACCELERATIONS
	aModel->getDynamicsEngine().computeDerivatives(dqdt,dudt);


	int i;

	fprintf(_outFPT,"\n\nGeneralized Coordinate    dqdt\n"
		"-------------------------------------");

	for (i=0;i<aModel->getNumCoordinates();i++)
		fprintf(_outFPT,"\n     %4d             %12.4e",i,dqdt[i]);

	fprintf(_outFPT,"\n-------------------------------------\n");

	fprintf(_outFPT,"\n\nGeneralized Speed    dudt\n"
		"-------------------------------------");

	for (i=0;i<aModel->getNumSpeeds();i++)
		fprintf(_outFPT,"\n   %4d          %12.4e",i,dudt[i]);

	fprintf(_outFPT,"\n-------------------------------------\n");


	delete dqdt;
	delete dudt;


	fprintf(_outFPT,"\n\nModelTestSuite.TestDerivatives: PASSED.\n"
		"========================================"
		"=======================================\n\n");

	return(true);
}


//=============================================================================
// MASS MATRIX
//=============================================================================
//__________________________________________________________________________
/**
 * Test class rdMassMatrix.
 *		Model::formMassMatrix()
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestMassMatrix(Model *aModel)
{
	if(aModel==NULL) {
		fprintf(_outFPT,
			"ModelTestSuite.TestMassMatrix: FAILED- NULL model pointer.\n");
		return(false);
	}

	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestMassMatrix:\n");

	double *rI;

	int nu = aModel->getNumSpeeds();

	rI = new double[nu*nu];

	aModel->getDynamicsEngine().formMassMatrix(rI);

	for (int i=0;i<nu;i++)
	{
		fprintf(_outFPT,"\n");

		for (int j=0;j<nu;j++)
			fprintf(_outFPT,"%12.4e   ",rI[i*nu + j]);
	}

	delete rI;

	fprintf(_outFPT,"\n\nModelTestSuite.TestMassMatrix: PASSED.\n"
		"========================================"
		"=======================================\n\n");

	return(true);
}



//=============================================================================
// JOCOBIAN
//=============================================================================
//__________________________________________________________________________
/**
 * Test the Jocobian methods.
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestJacobian(Model *aModel)
{
/*
	if(aModel==NULL) {
		fprintf(_outFPT,
			"ModelTestSuite.TestJacobian: FAILED- NULL model pointer.\n");
		return(false);
	}

	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestJacobian:\n");

	int i,j,body;

	// GET NUMBERS
	int nq = aModel->getNumCoordinates();
	int nu = aModel->getNumSpeeds();

	// CONSTRUCT Q's and U's
	double *q = new double[nq];
	double *u = new double[nu];

	// SET VALUES FOR THE Q'S AND U'S
	for(i=0;i<nq;i++) q[i] = 0.0;
	for(i=0;i<nu;i++) u[i] = 0.0;
	//q[3] = Math::DTR*90.0;   
	//q[4] = Math::DTR*20.0;
	//q[5] = Math::DTR*20.0;

	// CONVERT TO QUATERNIONS
	aModel->convertAnglesToQuaternions(q,q);

	// SET THE CONFIGURATION
	aModel->setConfiguration(q,u);

	// DEFINE SOME PARAMETERS
	double point1[] = { 1.0, 1.0, 1.0 };
	double point2[] = { 1.0, 1.0, -1.0 };

	// TEST JACOBIAN METHODS
	double *I = new double[nu*nu];
	double *J = new double[6*nu];
	double *J_1 = new double[nu*6];

	aModel->formMassMatrix(I);
	fprintf(_outFPT,"\nMass Matrix:");
	for (i=0;i<nu;i++)
	{
		fprintf(_outFPT,"\n");

		for (j=0;j<nu;j++)
			fprintf(_outFPT,"%12.5lf   ",I[i*nu + j]);
	}


	for (body=0;body<aModel->getNumBodies();body++)
	{
		fprintf(_outFPT,"\n\nBody: %d ====================\n",body);

		aModel->formJacobian(body,point1,J);
		fprintf(_outFPT,"\nJacobian at point (%.3lf, %.3lf, %.3lf):",
			point1[0],point1[1],point1[2]);
		for (i=0;i<6;i++)
		{
			fprintf(_outFPT,"\n");

			for (j=0;j<nu;j++)
				fprintf(_outFPT,"%12.5lf   ",J[i*nu + j]);
		}

		aModel->formJacobianInverse(I,J,J_1);
		fprintf(_outFPT,"\n\nJacobianInverse at point (%.3lf, %.3lf, %.3lf):",
			point1[0],point1[1],point1[2]);
		for (i=0;i<nu;i++)
		{
			fprintf(_outFPT,"\n");

			for (j=0;j<6;j++)
				fprintf(_outFPT,"%12.5lf   ",J_1[i*nu + j]);
		}

		// MULTIPLY J * J_1
		double identity[6][6];
		Mtx::Multiply(6,nu,6,J,J_1,&identity[0][0]);

		fprintf(_outFPT,"\n\nJacobian * JacobianInverse ?= Identity Matrix:");
		for (i=0;i<6;i++)
		{
			fprintf(_outFPT,"\n");

			for (j=0;j<6;j++)
				fprintf(_outFPT,"%12.5lf   ",identity[i][j]);
		}


		// APPLY A FORCE AT POINT
		double fp[] = { 1.0, 0.0, 0.0 , 0.0, 0.0, 0.0 };

		// COMPUTE EQUIVALENT GENERALIZED FORCES
		double f[6];
		double *JT = new double[nu*6];

		Mtx::Transpose(6,nu,J,JT);

		fprintf(_outFPT,"\n\nJacobianTranspose at point (%.3lf, %.3lf, %.3lf):",
			point1[0],point1[1],point1[2]);
		for (i=0;i<nu;i++)
		{
			fprintf(_outFPT,"\n");

			for (j=0;j<6;j++)
				fprintf(_outFPT,"%12.5lf   ",JT[i*nu + j]);
		}


		Mtx::Multiply(nu,6,1,JT,fp,f);

		fprintf(_outFPT,"\n\nForce at point (%.3lf, %.3lf, %.3lf):",
			point1[0],point1[1],point1[2]);

		fprintf(_outFPT,"\n");

		for (i=0;i<6;i++)
			fprintf(_outFPT,"%.3lf   ",f[i]);


		// GET JACOBIAN AND ITS INVERSE FOR A NEW POINT

		aModel->formJacobian(body,point2,J);
		fprintf(_outFPT,"\n\nJacobian at point (%.3lf, %.3lf, %.3lf):",
			point2[0],point2[1],point2[2]);
		for (i=0;i<6;i++)
		{
			fprintf(_outFPT,"\n");

			for (j=0;j<nu;j++)
				fprintf(_outFPT,"%12.5lf   ",J[i*nu + j]);
		}

		aModel->formJacobianInverse(I,J,J_1);
		fprintf(_outFPT,"\n\nJacobianInverse at point (%.3lf, %.3lf, %.3lf):",
			point2[0],point2[1],point2[2]);
		for (i=0;i<6;i++)
		{
			fprintf(_outFPT,"\n");

			for (j=0;j<nu;j++)
				fprintf(_outFPT,"%12.5lf   ",J_1[i*nu + j]);
		}


		// PROJECT BACK TO THE NEW POINT
		double fpp[6];
		double *J_1T = new double[6*nu];

		Mtx::Transpose(nu,6,J_1,J_1T);

		fprintf(_outFPT,"\n\nJacobianInverseTranspose at point (%.3lf, %.3lf, "
			"%.3lf):",point2[0],point2[1],point2[2]);
		for (i=0;i<nu;i++)
		{
			fprintf(_outFPT,"\n");

			for (j=0;j<6;j++)
				fprintf(_outFPT,"%12.5lf   ",J_1T[i*nu + j]);
		}


		Mtx::Multiply(6,nu,1,J_1T,f,fpp);

		fprintf(_outFPT,"\n\nForce at point (%.3lf, %.3lf, %.3lf):",
			point2[0],point2[1],point2[2]);

		fprintf(_outFPT,"\n");

		for (i=0;i<6;i++)
			fprintf(_outFPT,"%.3lf   ",fpp[i]);


		delete JT;
		delete J_1T;
	}


	delete I;
	delete J;
	delete J_1;

	fprintf(_outFPT,"\n\nModelTestSuite.TestJacobian: PASSED.\n"
		"========================================"
		"=======================================\n\n");

	*/
	return(true);
}

/*
	int i;

	// GET NUMBERS
	int nq = aModel->getNumCoordinates();
	int nu = aModel->getNumSpeeds();

	// CONSTRUCT Q's and U's
	double *q = new double[nq];
	double *u = new double[nu];

	// SET VALUES FOR THE Q'S AND U'S
	for(i=0;i<nq;i++) q[i] = 0.0;
	for(i=0;i<nu;i++) u[i] = 0.0;
	//q[3] = Math::DTR*90.0;   
	//q[4] = Math::DTR*20.0;
	//q[5] = Math::DTR*20.0;

	// CONVERT TO QUATERNIONS
	aModel->convertAnglesToQuaternions(q,q);

	// SET THE CONFIGURATION
	aModel->setConfiguration(q,u);

	// DEFINE SOME PARAMETERS
	int body = 0;
	double point[] = { 1.0, 1.0, 1.0 };

	// TEST JACOBIAN METHODS
	double *I = new double[nu*nu];
	double *J = new double[6*nu];
	double *J_1 = new double[nu*6];
	aModel->formMassMatrix(I);
	fprintf(_outFPT,"\nMass Matrix:\n");  Mtx::Print(nu,nu,I,3);
	aModel->formJacobian(body,point,J);
	fprintf(_outFPT,"\nJacobian:\n");  Mtx::Print(6,nu,J,3);
	aModel->formJacobianInverse(I,J,J_1);

	// MULTIPLY J * J_1
	double identity[6][6];
	Mtx::Multiply(6,nu,6,J,J_1,&identity[0][0]);
	fprintf(_outFPT,"\n\nIdentity?:\n");
	Mtx::Print(6,6,&identity[0][0],3);

	// APPLY A FORCE AT POINT
	double fp[] = { 1.0, 0.0, 0.0 , 0.0, 0.0, 0.0 };

	// COMPUTE EQUIVALENT GENERALIZED FORCES
	double f[6];
	double *JT = new double[nu*6];
	Mtx::Transpose(6,nu,J,JT);
	Mtx::Multiply(nu,6,1,JT,fp,f);

	// GET JACOBIAN AND ITS INVERSE FOR A NEW POINT
	double point2[] = { 1.0, 1.0, -1.0 };
	aModel->formJacobian(body,point2,J);
	aModel->formJacobianInverse(I,J,J_1);

	// PROJECT BACK TO THE NEW POINT
	double fpp[6];
	double *J_1T = new double[6*nu];
	Mtx::Transpose(nu,6,J_1,J_1T);
	Mtx::Multiply(6,nu,1,J_1T,f,fpp);

	// PRINT THE RESULTS
	fprintf(_outFPT,"\n\nForce Testing...\n");
	fprintf(_outFPT,"\nForce:\n");
	Mtx::Print(6,1,fp,3);
	fprintf(_outFPT,"\nGeneralized Forces:\n");
	Mtx::Print(6,1,f,3);
	fprintf(_outFPT,"\nProjected Force:\n");
	Mtx::Print(6,1,fpp,3);


*/


//=============================================================================
// UTILITY
//=============================================================================
//__________________________________________________________________________
/**
 * Test angular conversions.
 *		Model::convertDirectionCosinesToQuaternions()
 *		Model::convertQuaternionsToDirectionCosines()
 *		Model::convertDirectionCosinesToAngles()
 *		Model::convertAnglesToDirectionCosines()
 *		Model::convertAnglesToQuaternions()
 *		Model::convertQuaternionsToAngles()
 *		Model::convertRadiansToDegrees()
 *		Model::convertDegreesToRadians()
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestOrientationUtilities(Model *aModel)
{
	if(aModel==NULL) {
		fprintf(_outFPT,"ModelTestSuite.TestOrientationUtilities: "
			"FAILED- NULL model pointer.\n");
		return(false);
	}


	fprintf(_outFPT,"\n========================================"
		"=======================================\n"
		"ModelTestSuite.TestOrientationUtilities:\n");

	int i;
	double rDirCosStart[3][3],rDirCosOut[3][3];

	BodySet *bodies = aModel->getDynamicsEngine().getBodySet();

	for (i=0;i<aModel->getNumBodies();i++)
	{

		fprintf(_outFPT,"\n\nBody: %d  ------------------------------\n",i);

		aModel->getDynamicsEngine().getDirectionCosines(*(*bodies)[i],rDirCosStart);

		fprintf(_outFPT,"\nDirection Cosines (Start)\n"
			"-------------------------------");

		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			rDirCosStart[0][0],rDirCosStart[0][1],rDirCosStart[0][2]);
		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			rDirCosStart[1][0],rDirCosStart[1][1],rDirCosStart[1][2]);
		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			rDirCosStart[2][0],rDirCosStart[2][1],rDirCosStart[2][2]);

		fprintf(_outFPT,"\n-------------------------------\n");


		double rQ1,rQ2,rQ3,rQ4;

		aModel->getDynamicsEngine().convertDirectionCosinesToQuaternions(rDirCosStart,
			&rQ1,&rQ2,&rQ3,&rQ4);

		fprintf(_outFPT,"\n\nQuaternions\n"
			"------------------------------------------------------");

		fprintf(_outFPT,"\n%12.4lf  %12.4lf  %12.4lf  %12.4lf",
			rQ1,rQ2,rQ3,rQ4);

		fprintf(_outFPT,"\n------------------------------"
			"------------------------\n");


		aModel->getDynamicsEngine().convertQuaternionsToDirectionCosines(rQ1,rQ2,rQ3,rQ4,
			rDirCosOut);

		fprintf(_outFPT,"\n\nDirection Cosines\n"
			"-------------------------------");

		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			rDirCosOut[0][0],rDirCosOut[0][1],rDirCosOut[0][2]);
		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			rDirCosOut[1][0],rDirCosOut[1][1],rDirCosOut[1][2]);
		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			rDirCosOut[2][0],rDirCosOut[2][1],rDirCosOut[2][2]);

		fprintf(_outFPT,"\n-------------------------------\n");


		double rE1,rE2,rE3;

		aModel->getDynamicsEngine().convertDirectionCosinesToAngles(rDirCosOut,&rE1,&rE2,&rE3);

		fprintf(_outFPT,"\n\nAngles\n"
			"----------------------------------------");

		fprintf(_outFPT,"\n%12.4lf  %12.4lf  %12.4lf",rE1,rE2,rE3);

		fprintf(_outFPT,"\n----------------------------------------\n");


		aModel->getDynamicsEngine().convertAnglesToDirectionCosines(rE1,rE2,rE3,rDirCosOut);

		fprintf(_outFPT,"\nDirection Cosines\n"
			"------------------------------");

		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			rDirCosOut[0][0],rDirCosOut[0][1],rDirCosOut[0][2]);
		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			rDirCosOut[1][0],rDirCosOut[1][1],rDirCosOut[1][2]);
		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			rDirCosOut[2][0],rDirCosOut[2][1],rDirCosOut[2][2]);

		fprintf(_outFPT,"\n-------------------------------\n");


		double DCError[3][3];

		for (int j=0;j<3;j++)
			for (int k=0;k<3;k++)
				DCError[j][k] = rDirCosOut[j][k] - rDirCosStart[j][k];

		fprintf(_outFPT,"\n\nDirection Cosines Error\n"
			"-------------------------------");

		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			DCError[0][0],DCError[0][1],DCError[0][2]);
		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			DCError[1][0],DCError[1][1],DCError[1][2]);
		fprintf(_outFPT,"\n%9.6lf  %9.6lf  %9.6lf",
			DCError[2][0],DCError[2][1],DCError[2][2]);

		fprintf(_outFPT,"\n-------------------------------\n");




		int nu = aModel->getNumSpeeds();
		int nq = aModel->getNumCoordinates();

		double *aQAng = new double[nu];
		double *aQ = new double[nq];


		for (i=0;i<nu;i++)
			aQAng[i] = ((double)i) * 0.1;

		fprintf(_outFPT,"\n\nAngles (Start)\n----------");

		for (i=0;i<nu;i++)
			fprintf(_outFPT,"\n%.4lf",aQAng[i]);

		fprintf(_outFPT,"\n---------\n");


		aModel->getDynamicsEngine().convertAnglesToQuaternions(aQAng,aQ);


		fprintf(_outFPT,"\n\nQuaternions\n----------");

		for (i=0;i<nq;i++)
			fprintf(_outFPT,"\n%.4lf",aQ[i]);

		fprintf(_outFPT,"\n---------\n");


		aModel->getDynamicsEngine().convertQuaternionsToAngles(aQ,aQAng);


		fprintf(_outFPT,"\n\nAngles (radians)\n----------");

		for (i=0;i<nu;i++)
			fprintf(_outFPT,"\n%.4lf",aQAng[i]);

		fprintf(_outFPT,"\n---------\n");


		double *rQDeg = new double[nu];

		aModel->getDynamicsEngine().convertRadiansToDegrees(aQAng,rQDeg);

		fprintf(_outFPT,"\n\nAngles (degrees)\n----------");

		for (i=0;i<nu;i++)
			fprintf(_outFPT,"\n%.4lf",rQDeg[i]);

		fprintf(_outFPT,"\n---------\n");

		double *rQAng = new double[nu];

		aModel->getDynamicsEngine().convertDegreesToRadians(rQDeg,rQAng);

		fprintf(_outFPT,"\n\nAngles (radians)\n----------");

		for (i=0;i<nu;i++)
			fprintf(_outFPT,"\n%.4lf",rQAng[i]);

		fprintf(_outFPT,"\n---------\n");


		//delete aQAng;
		//delete aQ;
		//delete rQDeg;
		//delete rQAng;

	}



	fprintf(_outFPT,"\nModelTestSuite.TestOrientationUtilities: PASSED.\n"
		"========================================"
		"=======================================\n\n");


	return(true);
}



//=============================================================================
// CONTACT
//=============================================================================
//__________________________________________________________________________
/**
 * Test class ContactForceSet.
 *
 * Right now these tests don't make use of the model at all; they
 * just test constructing a set of contacts from file and then writing
 * them back out to file.
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestContacts(Model *aModel)
{
	fprintf(_outFPT,"\n\nModelTestSuite.TestContacts...");
	char defaultName[] = "default.ctx";
	char set1Name[] = "set1.ctx";
	char set2Name[] = "set2.ctx";

	// CREATE A DEFAULT SET AND WRITE TO FILE
	ContactForceSet *contacts = new ContactForceSet();
	contacts->print(defaultName);

	// CONSTRUCT CONTACTS FROM FILE
	//ContactForceSet *contacts1 = new ContactForceSet(fileName1);

	// WRITE CONTACTS
	//contacts1->print(set2Name);

	// CLEANUP
	delete contacts;
	//delete contacts1;

	return(true);
}


//=============================================================================
// ACTUATORS
//=============================================================================
//__________________________________________________________________________
/**
 * Test class ActuatorSet.
 *
 * Right now these tests don't make use of the model at all; they
 * just test constructing a set of actuators from file and then writing
 * them back out to file.
 *
 * @param aModel Model on which to test the methods.
 * @return False if the test fails in some manner, true otherwise.
 */
bool ModelTestSuite::
TestActuators(Model *aModel)
{
	fprintf(_outFPT,"\n\nModelTestSuite.TestActuators...");
	char defaultName[] = "default.act";
	char set1Name[] = "set1.act";
	char set2Name[] = "set2.act";

	// CREATE A DEFAULT SET AND WRITE TO FILE
	ActuatorSet *act = new ActuatorSet();
	act->print(defaultName);

	// CONSTRUCT ACTUATORS FROM FILE
	//ActuatorSet *act1 = new ActuatorSet(set1Name);

	// WRITE ACTUATORS
	//act1->print(set2Name);

	// CLEANUP
	delete act;
	//delete act1;

	return(true);
}
