/* -------------------------------------------------------------------------- *
 *                      OpenSim:  testAssemblySolver.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//==========================================================================================================
//	testAssemblySolver loads models with constraints to verify that constraints are
//  adequately satified or that an appropriate exception is thrown.
//
//==========================================================================================================
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testAssembleModelWithConstraints(string modelFile);
void testAssemblySatisfiesConstraints(string modelFile);
double calcLigamentLengthError(const SimTK::State &s, const Model &model);

int main()
{
	try {
		LoadOpenSimLibrary("osimActuators");
		testAssemblySatisfiesConstraints("knee_patella_ligament.osim");
		testAssembleModelWithConstraints("PushUpToesOnGroundExactConstraints.osim");
		testAssembleModelWithConstraints("PushUpToesOnGroundLessPreciseConstraints.osim");
		testAssembleModelWithConstraints("PushUpToesOnGroundWithMuscles.osim");
	}
	catch (const std::exception& e) {
		cout << "\ntestAssemblySolver FAILED " << e.what() <<endl;
        return 1;
    }
    cout << "\ntestAssemblySolver PASSED" << endl;
    return 0;
}

//==========================================================================================================
// Test Cases
//==========================================================================================================
void testAssembleModelWithConstraints(string modelFile)
{
	using namespace SimTK;

	cout << "\n****************************************************************************" << endl;
	cout << " testAssembleModelWithConstraints with "<< modelFile << endl;
	cout << "****************************************************************************\n" << endl;

	//==========================================================================================================
	// Setup OpenSim model
	Model model(modelFile);
	const CoordinateSet &coords = model.getCoordinateSet();
	
	cout << "*********** Coordinates before initSystem ******************** " << endl;
	for(int i=0; i< coords.getSize(); i++) {
		cout << "Coordinate " << coords[i].getName() << " default value = " << coords[i].getDefaultValue() << endl;
	}

    State state = model.initSystem();
	model.equilibrateMuscles(state);

	cout << "*********** Coordinates after initSystem ******************** "  << endl;
	for(int i=0; i< coords.getSize(); i++) {
		cout << "Coordinate " << coords[i].getName() << " get value = " << coords[i].getValue(state) << endl;
	}

	// Verify that the reaction forces at the constraints are not rediculously large
	// They should sum to body-weight (more or less)
	model.getMultibodySystem().realize(state, Stage::Acceleration);

	const ConstraintSet &constraints = model.getConstraintSet();

	Vector_<SpatialVec> constraintBodyForces(constraints.getSize());
	Vector mobilityForces(0);

	double totalYforce = 0;
	
	for(int i=0; i< constraints.getSize(); i++) {
		constraints[i].calcConstraintForces(state, constraintBodyForces, mobilityForces);
		cout << "Constraint " << i << ":  " << constraints[i].getName();
		cout << " Force = " << constraintBodyForces(1)(1)(1) << endl;
		//constraintBodyForces.dump("Consrtaint Body Forces");
		totalYforce += constraintBodyForces(1)(1)(1);
	}
	
	cout << "Total Vertical Constraint Force:" << totalYforce << " N " << endl;

	double bw = -model.getTotalMass(state)*(model.getGravity()[1]);

	ASSERT_EQUAL(totalYforce/bw, 1.0, 0.02);

	//const CoordinateSet &coords = model.getCoordinateSet();
	double q_error = 0;
	for(int i=0; i< coords.getSize(); i++) {
		q_error += fabs(coords[i].getValue(state)-coords[i].getDefaultValue());
	}

	cout << "Average Change in  Default Configuration:" << q_error/coords.getSize() << endl;

	//==========================================================================================================
	// Integrate forward and init the state and update defaults to make sure
	// assembler is not effecting anything more than the pose.
    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0.0);
    manager.setFinalTime(0.01);

	model.equilibrateMuscles(state);
	Vector y1 = state.getY();
 
	// defaults should capture an accurate snapshot of the model
	model.setPropertiesFromState(state);
    manager.integrate(state);
    Vector y2 = state.getY();

	// recreate system with states from defaults
    State state2 = model.initSystem();
    Vector y3 = state2.getY();

	model.setPropertiesFromState(state);
    state2 = model.initSystem();
    Vector y4 = state2.getY();
    for (int i = 0; i < y1.size(); i++) 
    {
        ASSERT_EQUAL(y1[i], y3[i], 1e-4, __FILE__, __LINE__, "Initial state changed after 2nd call to initSystem");
        ASSERT_EQUAL(y2[i], y4[i], 1e-4, __FILE__, __LINE__, "State differed after 2nd simulation from same init state.");
    }
    ASSERT(max(abs(y1-y2)) > 1e-3, __FILE__, __LINE__, "Check that state changed after simulation FAILED");
}


void testAssemblySatisfiesConstraints(string modelFile)
{
	using namespace SimTK;

	cout << "****************************************************************************" << endl;
	cout << " testAssemblySatisfiesConstraints :: "<< modelFile << endl;
	cout << "****************************************************************************\n" << endl;
	//==========================================================================================================
	// Setup OpenSim model
	Model model(modelFile);

	const CoordinateSet &modelcoords = model.getCoordinateSet();
	cout << "*********** Coordinate defaults (before initSystem) ******************** " << endl;
	for(int i=0; i< modelcoords.getSize(); i++) {
		cout << "Coordinate " << modelcoords[i].getName() 
			<< " default value = " << modelcoords[i].getDefaultValue() << endl
			<< " is_free to_satisfy_constraints = " << modelcoords[i].get_is_free_to_satisfy_constraints()
			<< endl;
	}

	//model.setUseVisualizer(true);
    State& state = model.initSystem();

	const CoordinateSet &coords = model.getCoordinateSet();
	cout << "***** Coordinate values (after initSystem including Assembly ********* " << endl;
	for(int i=0; i< coords.getSize(); i++) {
		cout << "Coordinate " << coords[i].getName() << " value = " 
			<< coords[i].getValue(state) << endl;
	}

	//model.getVisualizer().show(state);
	double cerr = calcLigamentLengthError(state, model);

	double kneeAngle = -Pi/3; 

	int N = 100;
	double lower = -2*Pi/3, upper = Pi/18;
	double delta = (upper-lower)/N;

	double qerr = 0;
	
	for(int i=0; i<N; ++i){
		kneeAngle = upper-i*delta;
		coords[0].setValue(state, kneeAngle, true);
		//model.getVisualizer().show(state);
		cerr = calcLigamentLengthError(state, model);
		qerr = coords[0].getValue(state)-kneeAngle;
		//cout << "Assembly errors:: cerr = " << cerr << " m,  qerr = " 
		//	<< convertRadiansToDegrees(qerr) << " degrees" << endl;
		ASSERT_EQUAL(cerr, 0.0, 1e-4, __FILE__, __LINE__, "Constraint NOT satisfied to within assembly accuracy");
	}

}

double calcLigamentLengthError(const SimTK::State &s, const Model &model)
{
	using namespace SimTK;
	double error = 0;

	ConstantDistanceConstraint* constraint = dynamic_cast<ConstantDistanceConstraint*>(&model.getConstraintSet()[0]);
	const BodySet& bodies = model.getBodySet();
	
	if(constraint){
		Vec3 p1inB1, p2inB2, p1inG, p2inG;
		p1inB1 = constraint->get_location_body_1();
		p2inB2 = constraint->get_location_body_2();

		const OpenSim::Body& b1 = bodies.get(constraint->get_body_1());
		const OpenSim::Body& b2 = bodies.get(constraint->get_body_2());

		model.getSimbodyEngine().getPosition(s, b1, p1inB1, p1inG);
		model.getSimbodyEngine().getPosition(s, b2, p2inB2, p2inG);

		double length = (p2inG-p1inG).norm();
		error = length - constraint->get_constant_distance();
	}

	return error;
}
