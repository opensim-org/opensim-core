/* -------------------------------------------------------------------------- *
 *                   OpenSim:  testSimulationUtilities.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2018 Stanford University and the Authors                *
 * Author(s): Chris Dembia
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

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimulationUtilities.h>
#include <OpenSim/Actuators/PointActuator.h>

using namespace OpenSim;
using namespace std;

void testUpdatePre40KinematicsFor40MotionType();

int main() {
    // The following model(s) contains Actuators that are registered when the
    // osimActuators library is loaded. But unless we call at least one
    // function defined in the osimActuators library, some linkers will omit
    // its dependency from the executable and it will not be loaded at
    // startup.
    { PointActuator t; }

    SimTK_START_TEST("testSimulationUtilities");
        SimTK_SUBTEST(testUpdatePre40KinematicsFor40MotionType);
    SimTK_END_TEST();
}

// Ensure the simulate() method works as intended.
void testSimulate() {
    cout << "Running testSimulate" << endl;

    using SimTK::Vec3;
    const double gravity = 9.81;

    // Create a simple model consisting of an unconstrained ball.
    Model model;
    model.setGravity(Vec3(0, -gravity, 0));
    auto ball = new Body("ball", 1., Vec3(0), SimTK::Inertia::sphere(1.));
    model.addBody(ball);
    auto freeJoint = new FreeJoint("freeJoint", model.getGround(), *ball);
    model.addJoint(freeJoint);

    // Simulate from time t0 to t1. The ball should end up at -1/2*g*duration^2.
    auto testSim = [&](double t0, double t1) -> void {
        cout << "- simulating from t = " << t0 << " to " << t1 << "..." << endl;
        SimTK::State& s = model.initSystem();
        const double y1expected = -0.5 * gravity * (t1-t0) * (t1-t0);
        s.setTime(t0);
        s = simulate(model, s, t1);

        SimTK_TEST_EQ(s.getTime(), t1);
        model.realizePosition(s);
        const Vec3 pos1 = model.getBodySet().get("ball").getPositionInGround(s);
        SimTK_TEST_EQ(pos1[SimTK::YAxis], y1expected);
    };
    testSim(0., 10.);
    testSim(5., 15.);

    // No simulation should occur if t1 <= t0.
    {
        cout << "- ensuring simulation aborts if t1 <= t0..." << endl;
        SimTK::State& s = model.initSystem();
        const double t0 = 2.;
        s.setTime(t0);
        s = simulate(model, s, t0-1.);
        SimTK_TEST_EQ(s.getTime(), t0);
    }
}

void testUpdatePre40KinematicsFor40MotionType() {
    
    // The model and motion files for this test are from the opensim-models
    // repository. This PR is related to issues #2240 and #2088.
    
    Model model("testSimulationUtilities_leg6dof9musc_20303.osim");
    
    // Ensure the model file has an inconsistent motion type
    // (that it wasn't accidentally updated in the repository).
    SimTK_TEST(!model.getWarningMesssageForMotionTypeInconsistency().empty());
    
    const std::string origKinematicsFile =
            "testSimulationUtilities_leg69_IK_stance_pre4.mot";
    Storage origKinematics(origKinematicsFile);
    {
        auto updatedKinematics =
            updatePre40KinematicsStorageFor40MotionType(model, origKinematics);
        
        // Undo the only change that update...() should have made and
        // ensure we get back the original data.
        updatedKinematics->multiplyColumn(
                updatedKinematics->getStateIndex("knee_angle_pat_r"),
                SimTK_RADIAN_TO_DEGREE);
        
        const int numColumns = origKinematics.getColumnLabels().getSize();
        CHECK_STORAGE_AGAINST_STANDARD(*updatedKinematics, origKinematics,
                std::vector<double>(numColumns, 1e-14),
                __FILE__, __LINE__,
                "updatePre40KinematicsStorageFor40MotionType() altered columms incorrectly.");
    }
    
    // Test updatePre40KinematicsFilesFor40MotionType().
    const std::string updatedKinematicsFile =
            "testSimulationUtilities_leg69_IK_stance_pre4_updated.mot";
    if (IO::FileExists(updatedKinematicsFile)) {
        std::remove(updatedKinematicsFile.c_str());
    }
    updatePre40KinematicsFilesFor40MotionType(model, {origKinematicsFile});
    SimTK_TEST(IO::FileExists(updatedKinematicsFile));
    
    {
        Storage updatedKinematics(updatedKinematicsFile);
        updatedKinematics.multiplyColumn(
            updatedKinematics.getStateIndex("knee_angle_pat_r"),
            SimTK_RADIAN_TO_DEGREE);
    
        const int numColumns = origKinematics.getColumnLabels().getSize();
        // We lose precision when through the file.
        CHECK_STORAGE_AGAINST_STANDARD(updatedKinematics, origKinematics,
                std::vector<double>(numColumns, 1e-6),
                __FILE__, __LINE__,
                "updatePre40KinematicsFilesFor40MotionType() altered columms incorrectly.");
    }
    
    {
        // Check that we can still update data with a copy of the model
        // (even though the XML document is gone).
        Model modelCopy(model);
        SimTK_TEST(!model.getWarningMesssageForMotionTypeInconsistency().empty());
        
        auto updatedKinematics =
            updatePre40KinematicsStorageFor40MotionType(model, origKinematics);
        
        updatedKinematics->multiplyColumn(
                updatedKinematics->getStateIndex("knee_angle_pat_r"),
                SimTK_RADIAN_TO_DEGREE);
        
        const int numColumns = origKinematics.getColumnLabels().getSize();
        CHECK_STORAGE_AGAINST_STANDARD(*updatedKinematics, origKinematics,
                std::vector<double>(numColumns, 1e-14),
                __FILE__, __LINE__,
                "updatePre40KinematicsStorageFor40MotionType(), with a copied "
                "model, altered columms incorrectly.");
    }
    const std::string updatedModelFile =
            "testSimulationUtilities_leg6dof9musc_updated.osim";
    if (IO::FileExists(updatedModelFile)) {
        std::remove(updatedModelFile.c_str());
    }
    {
        // Check that we get an exception if a model's document version is
        // too recent to need updating.
        model.print(updatedModelFile);
        Model updatedModel(updatedModelFile);
        SimTK_TEST(updatedModel.getWarningMesssageForMotionTypeInconsistency().empty());
        
        SimTK_TEST_MUST_THROW_EXC(
            updatePre40KinematicsStorageFor40MotionType(updatedModel, origKinematics),
            Exception);
    }
}




















