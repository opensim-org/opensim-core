/* -------------------------------------------------------------------------- *
 *                     OpenSim:  testPrescribedForce.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Eastman, Ajay Seth                                        *
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
//  testPrescribedForce tests the application of function specified forces applied to a body
//  as a force and torque on the body, with the point force application also a function
//  Tests Include:
//      1. No force
//      2. Force on the body
//      3. Force at a point
//      4. Torque on a body
//      4. Forces from a file.
//     Add tests here
//
//==========================================================================================================
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PrescribedForce.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/TransformAxis.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include "SimTKsimbody.h"
#include "SimTKmath.h"

using namespace OpenSim;
using namespace std;

//==========================================================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-6;
const static double duration = 1.0;
const static SimTK::Vec3 gravity_vec = SimTK::Vec3(0, -9.8065, 0);

SimTK::MassProperties ballMass = SimTK::MassProperties(8.806, SimTK::Vec3(0), SimTK::Inertia(SimTK::Vec3(0.1268, 0.0332, 0.1337)));
//==========================================================================================================
static int counter=0;

void testNoForce();
void testForceAtOrigin();
void testForceAtPoint();
void testTorque();

int main()
{
    SimTK::Array_<std::string> failures;

    try { testNoForce(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testNoForce");
    }
    try { testForceAtOrigin(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testForceAtOrigin");
    }
    try { testForceAtPoint(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testForceAtPoint");
    }
    try { testTorque(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testTorque");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "testPrescribedForce Done" << endl;
    return 0;
}

//==========================================================================================================
// Test Cases
//==========================================================================================================
void testPrescribedForce(OpenSim::Function* forceX, OpenSim::Function* forceY, OpenSim::Function* forceZ,
                 OpenSim::Function* pointX, OpenSim::Function* pointY, OpenSim::Function* pointZ,
                 OpenSim::Function* torqueX, OpenSim::Function* torqueY, OpenSim::Function* torqueZ,
                 vector<SimTK::Real>& times, vector<SimTK::Vec3>& accelerations, vector<SimTK::Vec3>& angularAccelerations)
{
    using namespace SimTK;

    //==========================================================================================================
    // Setup OpenSim model
    Model *osimModel = new Model;
    //OpenSim bodies
    const Ground& ground = osimModel->getGround();
    OpenSim::Body ball;
    ball.setName("ball");
    ball.setMass(0);
    // Add joints
    FreeJoint free("free", ground, Vec3(0), Vec3(0), ball, Vec3(0), Vec3(0));

    // Rename coordinates for a free joint
    for(int i=0; i<free.numCoordinates(); i++){
        std::stringstream coord_name;
        coord_name << "free_q" << i;
        free.upd_coordinates(i).setName(coord_name.str());
    }

    osimModel->addBody(&ball);
    osimModel->addJoint(&free);

    // Add a PrescribedForce.
    PrescribedForce force("forceOnBall", ball);
    if (forceX != NULL)
        force.setForceFunctions(forceX, forceY, forceZ);
    if (pointX != NULL)
        force.setPointFunctions(pointX, pointY, pointZ);
    if (torqueX != NULL)
        force.setTorqueFunctions(torqueX, torqueY, torqueZ);

    counter++;
    osimModel->updForceSet().append(&force);

    // BAD: have to set memoryOwner to false or program will crash when this test is complete.
    osimModel->disownAllComponents();

    //Set mass
    ball.setMass(ballMass.getMass());
    ball.setMassCenter(ballMass.getMassCenter());
    ball.setInertia(ballMass.getInertia());

    osimModel->setGravity(gravity_vec);
    osimModel->finalizeConnections();
    osimModel->print("TestPrescribedForceModel.osim");

    delete osimModel;
    // Check that serialization/deserialization is working correctly as well
    osimModel = new Model("TestPrescribedForceModel.osim");
    SimTK::State& osim_state = osimModel->initSystem();
    osimModel->getMultibodySystem().realize(osim_state, Stage::Position );

    //==========================================================================================================
    // Compute the force and torque at the specified times.

    const OpenSim::Body& body = osimModel->getBodySet().get("ball");
    for (unsigned int i = 0; i < times.size(); ++i)
    {
        osim_state.updTime() = times[i];
        osimModel->getMultibodySystem().realize(osim_state, Stage::Acceleration);
        Vec3 accel = body.findStationAccelerationInGround(osim_state, Vec3(0));
        Vec3 angularAccel = body.getAccelerationInGround(osim_state)[0];

        ASSERT_EQUAL(accelerations[i], accel, integ_accuracy);
        ASSERT_EQUAL(angularAccelerations[i], angularAccel, integ_accuracy);
    }
}

void testNoForce()
{
    using namespace SimTK;

    vector<Real> times;
    vector<Vec3> accel;
    vector<Vec3> angularAccel;
    times.push_back(0.0);
    accel.push_back(gravity_vec);
    angularAccel.push_back(Vec3(0));
    times.push_back(0.5);
    accel.push_back(gravity_vec);
    angularAccel.push_back(Vec3(0));
    times.push_back(1.0);
    accel.push_back(gravity_vec);
    angularAccel.push_back(Vec3(0));
    testPrescribedForce(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, times, accel, angularAccel);
}

void testForceAtOrigin()
{
    using namespace SimTK;

    vector<Real> times;
    vector<Vec3> accel;
    vector<Vec3> angularAccel;
    times.push_back(0.0);
    accel.push_back(gravity_vec+Vec3(1.0/ballMass.getMass(), 0, 0));
    angularAccel.push_back(Vec3(0));
    times.push_back(0.5);
    accel.push_back(gravity_vec+Vec3(0.5/ballMass.getMass(), 0.5/ballMass.getMass(), 0));
    angularAccel.push_back(Vec3(0));
    times.push_back(1.0);
    accel.push_back(gravity_vec+Vec3(0, 1/ballMass.getMass(), 0));
    angularAccel.push_back(Vec3(0));
    PiecewiseLinearFunction *forceX = new PiecewiseLinearFunction(), *forceY = new PiecewiseLinearFunction(), *forceZ = new PiecewiseLinearFunction();
    forceX->addPoint(0, 1);
    forceX->addPoint(1, 0);
    forceY->addPoint(0, 0);
    forceY->addPoint(1, 1);
    forceZ->addPoint(0, 0);
    testPrescribedForce(forceX, forceY, forceZ, NULL, NULL, NULL, NULL, NULL, NULL, times, accel, angularAccel);
}

void testForceAtPoint()
{
    using namespace SimTK;

    Mat33 invInertia = ballMass.getInertia().toMat33().invert();
    vector<Real> times;
    vector<Vec3> accel;
    vector<Vec3> angularAccel;
    times.push_back(0.0);
    accel.push_back(gravity_vec+Vec3(1.0/ballMass.getMass(), 0, 0));
    angularAccel.push_back(invInertia*(Vec3(1, -1, 0)%Vec3(1.0, 0, 0)));
    PiecewiseLinearFunction *forceX = new PiecewiseLinearFunction(), *forceY = new PiecewiseLinearFunction(), *forceZ = new PiecewiseLinearFunction();
    forceX->addPoint(0, 1);
    forceY->addPoint(0, 0);
    forceZ->addPoint(0, 0);
    PiecewiseLinearFunction *pointX = new PiecewiseLinearFunction(), *pointY = new PiecewiseLinearFunction(), *pointZ = new PiecewiseLinearFunction();
    pointX->addPoint(0, 1);
    pointY->addPoint(0, -1);
    pointZ->addPoint(0, 0);
    testPrescribedForce(forceX, forceY, forceZ, pointX, pointY, pointZ, NULL, NULL, NULL, times, accel, angularAccel);
}

void testTorque()
{
    using namespace SimTK;

    Mat33 invInertia = ballMass.getInertia().toMat33().invert();
    vector<Real> times;
    vector<Vec3> accel;
    vector<Vec3> angularAccel;
    times.push_back(0.0);
    accel.push_back(gravity_vec);
    angularAccel.push_back(invInertia*Vec3(1, 0.5, 0));
    PiecewiseLinearFunction *torqueX = new PiecewiseLinearFunction(), *torqueY = new PiecewiseLinearFunction(), *torqueZ = new PiecewiseLinearFunction();
    torqueX->addPoint(0, 1);
    torqueY->addPoint(0, 0.5);
    torqueZ->addPoint(0, 0);
    testPrescribedForce(NULL, NULL, NULL, NULL, NULL, NULL, torqueX, torqueY, torqueZ, times, accel, angularAccel);
}
