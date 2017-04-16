/* -------------------------------------------------------------------------- *
*                         OpenSim:  testKneeLigament.cpp                     *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2012 Stanford University and the Authors                *
* Author(s): Dimitar Stanev                                                  *
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

//============================================================================
// Tests the KneeLigament by attaching an acl ligament and running FD by 
// letting the knee to fall from -130deg. Records the acl force and print the 
// adjusted model.
//
//============================================================================

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/KneeLigament.h>
#include <OpenSim/Analyses/ForceReporter.h>
#include <OpenSim/Simulation/Manager/Manager.h>

using std::cout;
using std::endl;
using std::string;
using namespace OpenSim;
using namespace SimTK;

KneeLigament* create_ligament(Model& model);
void setup_configuration(Model& model);
void simulate(Model& model, State& state);
void set_all_muscles_on_off(Model& model, State& state, bool isDisabled);

int main()
{
	try {
		Model model("gait2354_simbody.osim");

		model.addForce(create_ligament(model));

		ForceReporter* force_analysis = new ForceReporter(&model);
		model.addAnalysis(force_analysis);

		setup_configuration(model);

		model.buildSystem();

		model.print("gait2354_simbody_knee_ligament.osim");

		State& s = model.initializeState();

		set_all_muscles_on_off(model, s, true);

		simulate(model, s);

		force_analysis->printResults("knee_ligament");

	}
	catch (const std::exception& ex)
	{
		cout << "Exception: " << ex.what() << endl;
	}
	catch (...)
	{
		cout << "Unrecognized exception " << endl;
	}
	system("pause");
	return 0;
}

KneeLigament* create_ligament(Model& model)
{
	State dummy = State();

	//create ligament
	KneeLigament* ligament = new KneeLigament();
	ligament->setName("acl");

	//create point path
	PathPointSet* ligament_path = new PathPointSet();

	PathPoint femur_point = PathPoint();
	femur_point.setName("acl_femur");
	femur_point.setBody(model.updBodySet().get("femur_r"));
	femur_point.setLocation(dummy, Vec3(-0.00718, -0.40037, 0.00407));
	ligament_path->insert(0, femur_point);

	PathPoint tibia_point = PathPoint();
	tibia_point.setName("acl_tibia");
	tibia_point.setBody(model.updBodySet().get("tibia_r"));
	tibia_point.setLocation(dummy, Vec3(0.01657, -0.03009, -0.00074));
	ligament_path->insert(1, tibia_point);

	//create geometry path
	GeometryPath* geometry_path = new GeometryPath();
	geometry_path->setName("acl_geometry_path");
	geometry_path->updPathPointSet() = *ligament_path;
	ligament->updGeometryPath() = *geometry_path;

	//parameters
	ligament->setRestingLength(0.0323 / (1 + 0.02));
	ligament->setEpsilonLength(0.03);
	ligament->setLigamentStiffnessg(2000);

	return ligament;
}

void setup_configuration(Model& model)
{
	CoordinateSet& coordinate_set = model.updCoordinateSet();

	//set pelvis tilt
	coordinate_set.get("pelvis_tilt").setDefaultValue(0);

	//set hip flexion
	coordinate_set.get("hip_flexion_r").setDefaultValue(0);

	//set knee flexion
	coordinate_set.get("knee_angle_r").setDefaultValue(convertDegreesToRadians(-130));


	Array<string> coordinate_names;
	coordinate_set.getNames(coordinate_names);

	//lock except knee angle 
	for (int i = 0; i < coordinate_names.size(); i++)
	{
		coordinate_set.get(coordinate_names[i]).setDefaultLocked(true);
	}
	coordinate_set.get("knee_angle_r").setDefaultLocked(false);
}

void simulate(Model& model, State& state)
{
	//setup integrator
	RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	integrator.setAccuracy(1E-7);

	//manager
	Manager manager(model, integrator);
	manager.setInitialTime(0);
	manager.setFinalTime(1);

	//integrate
	clock_t begin = clock();
	cout << "Integrating ..." << endl;
	manager.integrate(state);
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Elapsed time: " << elapsed_secs << endl;
	cout.flush();

	//state results
	OpenSim::Storage state_reporter(manager.getStateStorage());
	model.updSimbodyEngine().convertRadiansToDegrees(state_reporter);
	state_reporter.print("knee_ligament.sto");
}

void set_all_muscles_on_off(Model& model, State& state, bool isDisabled)
{
	for (int i = 0; i < model.updMuscles().getSize(); i++)
	{
		model.updMuscles().get(i).setDisabled(state, isDisabled);
	}
}