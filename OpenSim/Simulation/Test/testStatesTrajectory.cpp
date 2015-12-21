/* -------------------------------------------------------------------------- *
 *                     OpenSim:  testStatesTrajectory.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
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

#include <OpenSim/Simulation/osimSimulation.h>
#include <chrono>

using namespace OpenSim;
using namespace SimTK;

// TODO what happens if the storage file has a hole? NaN?
// TODO append two trajectories.
// TODO ensure sequential.
// TODO formFromStorage versus readStorage.
// TODO test API for populating a StatesTrajectory.
// TODO test modeling options (locked coordinates, etc.)
// TODO option to fill out a statestrajectory muscle states by equilibrating.
// TODO access an individual state by time.
        // Access by time.
        // ---------------
        // TODO
// TODO test convenience createFromStorage(model, filename).
// TODO createFromStorage if model seems to be the wrong one.
// TODO createFromKinematicsStorage
// TODO append two StateTrajectories together.
// TODO bounds checking of get() vs upd().
// TODO segfaults if state is not realized.

const std::string statesStoFname = "testStatesTrajectory_readStorage_states.sto";

// Helper function to get a state variable value from a storage file.
Real getStorageEntry(const Storage& sto,
        const int timeIndex, const std::string& columnName) {
    Real value;
    const int columnIndex = sto.getStateIndex(columnName);
    sto.getData(timeIndex, columnIndex, value);
    return value;
}

void testPopulateTrajectory() {
    Model model("gait2354_simbody.osim");

    // To assist with creating interesting (non-zero) coordinate values:
    model.updCoordinateSet().get("pelvis_ty").setDefaultLocked(true);

    // TODO add controllers so the muscle states are interesting.
    auto& state = model.initSystem();

    SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
    SimTK::TimeStepper ts(model.getSystem(), integrator);
    ts.initialize(state);
    ts.setReportAllSignificantStates(true);
    integrator.setReturnEveryInternalStep(true);

    StatesTrajectory states;
    const double finalTime = 0.15;
    std::vector<double> times;
    while (ts.getState().getTime() < finalTime) {
        ts.stepTo(finalTime);
        times.push_back(ts.getState().getTime());
        // StatesTrajectory API for appending states:
        states.append(ts.getState());
    }

    // Make sure we have all the states
    SimTK_TEST_EQ((int)states.getSize(), (int)times.size());
    // ...and that they aren't all just references to the same single state.
    for (int i = 0; i < states.getSize(); ++i) {
        SimTK_TEST_EQ(states[i].getTime(), times[i]);
    }

}

// Create states storage file to for states storage tests.
void createStateStorageFile() {

    Model model("gait2354_simbody.osim");

    // To assist with creating interesting (non-zero) coordinate values:
    model.updCoordinateSet().get("pelvis_ty").setDefaultLocked(true);

    // TODO add controllers so the muscle states are interesting.
    auto& initState = model.initSystem();
    SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
    Manager manager(model, integrator);
    manager.setFinalTime(0.15);
    manager.integrate(initState);
    manager.getStateStorage().print(statesStoFname);
}

void testFromStatesStorageGivesCorrectStates() {

    // Read in trajectory.
    // -------------------
    {
        // It's important that we try using a model that is not initialized,
        // since `readStorage()` should be able to work with such a model.
        Model model("gait2354_simbody.osim");
        model.initSystem();

        Storage sto(statesStoFname);

        auto start = std::chrono::system_clock::now();
        auto states = StatesTrajectory::createFromStatesStorage(model, sto);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = (end - start);
    
        std::cout << "DEBUG duration: " << elapsed.count() << std::endl;

        // Test that the states are correct, and also that the iterator works.
        // -------------------------------------------------------------------
        int itime = 0;
        double currTime;
        for (const auto& state : states) {
            // Time.
            sto.getTime(itime, currTime);
            SimTK_TEST_EQ(currTime, state.getTime());

            // Multibody states.
            for (int ic = 0; ic < model.getCoordinateSet().getSize(); ++ic) {
                const auto& coord = model.getCoordinateSet().get(ic);
                auto coordName = coord.getName();
                auto jointName = coord.getJoint().getName();
                auto coordPath = jointName + "/" + coordName;
                // Coordinate.
                SimTK_TEST_EQ(getStorageEntry(sto, itime, coordPath + "/value"),
                              coord.getValue(state));

                // Speed.
                SimTK_TEST_EQ(getStorageEntry(sto, itime, coordPath + "/speed"),
                              coord.getSpeedValue(state));

            }

            // Muscle states.
            for (int im = 0; im < model.getMuscles().getSize(); ++im) {
                const auto& muscle = model.getMuscles().get(im);
                auto muscleName = muscle.getName();

                // Activation.
                // TODO Simply accessing these state variables requires realizing
                // to Velocity; I think this is a bug.
                model.getMultibodySystem().realize(state, SimTK::Stage::Velocity);
                SimTK_TEST_EQ(
                        getStorageEntry(sto, itime, muscleName + "/activation"),
                        muscle.getActivation(state));

                // Fiber length.
                SimTK_TEST_EQ(
                        getStorageEntry(sto, itime, muscleName + "/fiber_length"), 
                        muscle.getFiberLength(state));

                // More complicated computation based on state.
                SimTK_TEST(!SimTK::isNaN(muscle.getFiberForce(state)));
            }

            // More complicated computations based on state.
            auto loc = model.getBodySet().get("tibia_r")
                .findLocationInAnotherFrame(state,
                                            SimTK::Vec3(1, 0.5, 0.25),
                                            model.getGround());
            SimTK_TEST(!loc.isNaN());

            SimTK_TEST(!model.calcMassCenterVelocity(state).isNaN());
            // TODO acceleration-level stuff gives segfault.
            // TODO model.getMultibodySystem().realize(state, SimTK::Stage::Acceleration);
            // TODO SimTK_TEST(!model.calcMassCenterAcceleration(state).isNaN());

            itime++;
        }
    }
}

void testFromStatesStorageInconsistentModel() {

    // States are missing from the Storage.
    // ------------------------------------
    {
        Model model("gait2354_simbody.osim");
        Storage sto(statesStoFname);

        // Effectively remove state columns by editing column names.
        auto labels = sto.getColumnLabels();
        auto origLabel10 = labels[10];
        auto origLabel15 = labels[15];
        labels[10] = "star_wars";
        labels[15] = "the_force_awakens";
        sto.setColumnLabels(labels);

        // Test that an exception is thrown.
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, sto),
                StatesMissingFromStorage
                );
        // Check some other similar calls.
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, sto,
                    true, true),
                StatesMissingFromStorage
                );
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, sto,
                    true, false),
                StatesMissingFromStorage
                );

        // No exception if checkMissingFromStorage set to false.
        auto states = StatesTrajectory::createFromStatesStorage(model, sto, false);
        // The unspecified states are set to NaN (for at least two random
        // states).
        SimTK_TEST(SimTK::isNaN(
                    model.getStateVariableValue(states[0], origLabel10)));
        SimTK_TEST(SimTK::isNaN(
                    model.getStateVariableValue(states[4], origLabel15)));
        // Behavior is independent of value for checkMissingFromModel.
        StatesTrajectory::createFromStatesStorage(model, sto, false, true);
        StatesTrajectory::createFromStatesStorage(model, sto, false, false);
    }

    // States are missing from the Model.
    // ----------------------------------
    {
        Model model("gait2354_simbody.osim");
        Storage sto(statesStoFname);
        // Remove a few of the muscles.
        model.updForceSet().remove(0);
        model.updForceSet().remove(10);
        model.updForceSet().remove(30);

        // Test that an exception is thrown.
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, sto),
                StatesMissingFromModel
                );
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, sto, true, true),
                StatesMissingFromModel
                );
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, sto, false, true),
                StatesMissingFromModel
                );

        // No exception if checkMissingFromModel set to false, and behavior is
        // independent of value for checkMissingFromStorage.
        StatesTrajectory::createFromStatesStorage(model, sto,
                true, false);
        StatesTrajectory::createFromStatesStorage(model, sto,
                false, false);
    }
}

/*
void testEqualityOperator() {
    // Test trajectories that hold a single state.
    {
        Model model("gait2354_simbody.osim");
        const auto& state = model.initSystem();

        StatesTrajectory statesA;
        statesA.append(state);

        StatesTrajectory statesB;
        statesB.append(state);

        SimTK_TEST(statesA == statesB);

        // Ensure that two different trajectories are not equal.
        // -----------------------------------------------------
        {
            SimTK::State differentState(state);
            differentState.setTime(53.67);

            StatesTrajectory differentStates;
            differentStates.append(differenState);

            SimTK_TEST(states != differentStates);
        }

        // Copy constructor.
        // -----------------
        StatesTrajectory statesB2(statesB);
        SimTK_TEST(statesB2 == statesB);

        // Copy assignment.
        // ----------------
        StatesTrajectory statesB3 = statesB;
        SimTK_TEST(statesB2 == statesB);
        // TODO ensure two non-equal copied states come up as such.

    }
    
    // TODO auto state1 = SimTK::State(state);
    // TODO state1.setTime(
    // TODO auto state2 = SimTK::State(state1);
    // TODO better testing of copy constructor.
}
*/

void testModifyStates() {
    // TODO when we have a proper states serialization, use that instead of a
    // STO file.
    Model model("gait2354_simbody.osim");
    model.initSystem();
    StatesTrajectory states = StatesTrajectory::
        createFromStatesStorage(model, statesStoFname);

    for (auto& state : states) {
        auto& coord0 = model.getCoordinateSet()[0];
        coord0.setValue(state, 2.0 + coord0.getValue(state));
    }

    // TODO print state trajectory.
}

int main() {
        std::cout << "DEBUG-1" << std::endl;
    SimTK_START_TEST("testStatesTrajectory");

        std::cout << "DEBUG0" << std::endl;
        SimTK_SUBTEST(testPopulateTrajectory);

        // Test creation of trajectory from states storage.
        // ------------------------------------------------
        std::cout << "DEBUG1" << std::endl;
        createStateStorageFile();
        std::cout << "DEBUG2" << std::endl;
        SimTK_SUBTEST(testFromStatesStorageGivesCorrectStates);
        // TODO SimTK_SUBTEST(testFromStatesStorageInconsistentModel);

        // TODO SimTK_SUBTEST(testEqualityOperator);
        // TODO read from proper State serialization.

        SimTK_SUBTEST(testModifyStates);

    SimTK_END_TEST();
}
