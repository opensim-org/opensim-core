// testIK.cpp
// Author: Ayman Habib based on Peter Loan's version, Ajay Seth added synthtic data test
/* Copyright (c)  2005, Stanford University and Peter Loan.
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


// INCLUDES
#include <string>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>

using namespace std;
using namespace OpenSim;

#define ASSERT(cond) {if (!(cond)) throw exception();}
#define ASSERT_EQUAL(expected, found, tolerance) {double tol = std::max((tolerance), std::abs((expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}


bool equalStorage(Storage& stdStorage, Storage& actualStorage, double tol)
{
	double dMax = -SimTK::Infinity;

	const Array<std::string> &col_names = actualStorage.getColumnLabels();
	
	bool equal = false;
	double error = 0;
	double max_error = 0;

	for(int i=0; i<col_names.getSize(); i++){
		error = actualStorage.compareColumn(stdStorage, col_names[i], actualStorage.getFirstTime());
		max_error = error > max_error ? error : max_error;
	}

	equal = (max_error <= tol);
		
	return equal;
}

bool testInverseKinematicsGait2354()
{
	// read setup file and construct model
	InverseKinematicsTool* tool = new InverseKinematicsTool("subject01_Setup_InverseKinematics.xml");
	try {
		tool->run();
		Storage actualOutput(tool->getOutputMotionFileName());
		Storage stdStorage("std_subject01_walk1_ik.mot");

		// Check that we can match the input kinematics to within a fifth of a degree
		bool equal = equalStorage(stdStorage, actualOutput, 0.2);
		std::cout << (equal?"Success":"Failure") << endl;
		
		return equal;
	}
	catch(const std::exception& e) {
        cout << "exception: " << e.what() << endl;
        return false;
    }

	return true;
}

bool testInverseKinematicsUWDynamic()
{
	// read setup file and construct model
	InverseKinematicsTool* tool = new InverseKinematicsTool("uwdynamic_setup_ik.xml");
	try {
		tool->run();
	}
	catch(const std::exception& e) {
        cout << "exception: " << e.what() << endl;
        return false;
    }

	tool->print("ik_setup_test.xml");

	return true;
}

bool testInverseKinematicsSolverAPI()
{
	bool success = false;
	try{
		//Load and create the indicated model
		Model model("subject01_simbody.osim");

		// Initialize the the model's underlying computational system and get its default state.
		SimTK::State& s = model.initSystem();

		//Convert old Tasks to references for assembly and tracking
		MarkersReference markersReference;
		Set<MarkerWeight> markerWeights;
		SimTK::Array_<CoordinateReference> coordinateReferences;

		FunctionSet *coordFunctions = NULL;

		// Loop through old "IKTaskSet" and assign weights to the coordinate and marker references
		// For coordinates, create the functions for coordinate reference values
		IKTaskSet tasks("gait2354_IK_Tasks_uniform.xml");
		int index = 0;
		for(int i=0; i < tasks.getSize(); i++){
			if(IKCoordinateTask *coordTask = dynamic_cast<IKCoordinateTask *>(&tasks[i])){
				CoordinateReference *coordRef = NULL;
				if(coordTask->getValueType() == IKCoordinateTask::FromFile){
					 index = coordFunctions->getIndex(coordTask->getName(), index);
					 if(index >= 0){
						 coordRef = new CoordinateReference(coordTask->getName(),coordFunctions->get(index));
					 }
				}
				else if((coordTask->getValueType() == IKCoordinateTask::ManualValue)){
                        Constant reference(Constant(coordTask->getValue()));
						coordRef = new CoordinateReference(coordTask->getName(), reference);
				}
				else{ // assume it should be held at its current/default value
					double value = model.getCoordinateSet().get(coordTask->getName()).getValue(s);
					Constant reference = Constant(value);
					coordRef = new CoordinateReference(coordTask->getName(), reference);
				}

				if(coordRef == NULL)
					throw Exception("InverseKinematicsTool: value for coordinate "+coordTask->getName()+" not found.");

				coordinateReferences.push_back(*coordRef);
			}
			else if(IKMarkerTask *markerTask = dynamic_cast<IKMarkerTask *>(&tasks[i])){
				MarkerWeight *markerWeight = new MarkerWeight(markerTask->getName(), markerTask->getWeight());
				markerWeights.append(markerWeight);
			}
		}

		//Set the weights for markers
		markersReference.setMarkerWeightSet(markerWeights);
		//Load the makers
		markersReference.loadMarkersFile("subject01_synthetic_marker_data.trc");

		// Determine the start time, if the provided time range is not specified then use time from marker reference
		// also adjust the time range for the tool if the provided range exceed that of the marker data
		SimTK::Vec2 markersValidTimRange = markersReference.getValidTimeRange();
		double start_time = markersValidTimRange[0];
		double final_time = markersValidTimRange[1];

		// create the solver given the input data
		InverseKinematicsSolver ikSolver(model, markersReference, coordinateReferences, 10.0);
		s.updTime() = start_time;
		ikSolver.assemble(s);

		SimTK::Array_<SimTK::Vec3> markerLocations;
		ikSolver.computeCurrentMarkerLocations(markerLocations);

		SimTK::Array_<double> sqMarkerErrors;
		ikSolver.computeCurrentSquaredMarkerErrors(sqMarkerErrors);

		SimTK::Array_<double> markerErrors;
		ikSolver.computeCurrentMarkerErrors(markerErrors);

		SimTK::Array_<SimTK::Vec3> markerObservations;
		markersReference.getValues(s, markerObservations);

		const SimTK::Array_<string> &names = markersReference.getNames();

		cout << "Initial assembly squared marker errors:" << endl;
		for (unsigned int i = 1; i < sqMarkerErrors.size(); i++) {
			double err = (markerLocations[i]-markerObservations[i]).norm();
			cout << names[i] << ": " << markerErrors[i] << "  validated as: " <<  err << endl;
			ASSERT_EQUAL(markerErrors[i], err, 1e-6);
			ASSERT_EQUAL(std::sqrt(sqMarkerErrors[i]), err, 1e-6);
		}

		//Select a marker at random
		SimTK::Random::Uniform random;
		int mIndex = names.size()*random.getValue();

		string mName = names[mIndex];
		double mErr1 = ikSolver.computeCurrentMarkerError(mName);
		double mErr2 = ikSolver.computeCurrentMarkerError(mIndex);
		ASSERT_EQUAL(mErr1, mErr2, 1e-6);

		
		double dt = 1.0/markersReference.getSamplingFrequency();
		int Nframes = 3;
		for (int i = 1; i < Nframes; i++) {
			s.updTime() = start_time + i*dt;
			//Increase weighting with time
			ikSolver.updateMarkerWeight(mName, 2.0*i);
			ikSolver.track(s);
			mErr2 = ikSolver.computeCurrentMarkerError(mIndex);
			//Error should decrease if weighting is increasing
			ASSERT(mErr2 < mErr1);
			mErr1 = mErr2;
		}

		success = true;
	}
	catch (std::exception ex) {
		std::cout << "test InverseKinematicsSolver Failed: " << ex.what() << std::endl;
	}

	return success;
}

//______________________________________________________________________________
/**
* Test program to read test IK.
*
*/
int main()
{
	if(!testInverseKinematicsSolverAPI()){
		cout << "testInverseKinematicsSolverAPI Failed." << endl;
		return 1;
	}
	
	if(!testInverseKinematicsGait2354()){
		cout << "testInverseKinematicsGait2354 Failed." << endl;
		return 1;
	}

	if(!testInverseKinematicsUWDynamic()){
		cout << "testInverseKinematicsUWDynamic Failed." << endl;
		return 1;
	}

	// Construct model and read parameters file
	IKTool* tool = new IKTool("subject01_Setup_IK.xml");
	Model& model = tool->getModel();

    SimTK::State& s = model.initSystem();
    model.getSystem().realize(s, SimTK::Stage::Position );
	tool->run();
	Storage *actualOutput = tool->getIKTrialSet()[0].getOutputStorage();
	Storage stdStorage("std_subject_trial_ik.mot");
	bool equal = equalStorage(stdStorage, *actualOutput, 5e-2);
	std::cout << (equal?"Success":"Failure") << endl;
	delete tool;
	return (equal?0:1);
}

