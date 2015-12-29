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
#include <random>
#include <OpenSim/Common/Constant.h>
#include <cstdio>

using namespace OpenSim;
using namespace SimTK;

// TODO bounds checking of get() vs upd().
// TODO python wrapping.
// TODO write documentation for StatesTrajectory.

// TODO example code.
// TODO test convenience createFromStorage(model, filename).
// TODO handle pre-4.0 state storages (w/out full paths to the state variable).
// TODO what happens if the storage file has a hole? NaN?
// TODO option to fill out a statestrajectory muscle states by equilibrating.
// TODO option to assemble() model.
// TODO segfaults if state is not realized.
// TODO accessing acceleration-level outputs.

// TODO append two StateTrajectories together.
// TODO createFromKinematicsStorage
// TODO test modeling options (locked coordinates, etc.)

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
    const double finalTime = 0.05;
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

    // Randomly assign muscle excitations to create interesting activation
    // histories.
    auto* controller = new PrescribedController();
    // For consistent results, use same seed each time.
    std::default_random_engine generator(0); 
    // Uniform distribution between 0.1 and 0.9.
    std::uniform_real_distribution<double> distribution(0.1, 0.8);

    for (int im = 0; im < model.getMuscles().getSize(); ++im) {
        controller->addActuator(model.getMuscles()[im]);
        controller->prescribeControlForActuator(
            model.getMuscles()[im].getName(),
            new Constant(distribution(generator))
            );
    }

    model.addController(controller);

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

        auto states = StatesTrajectory::createFromStatesStorage(model, sto);

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

Storage newStorageWithRemovedRows(const Storage& origSto,
        const std::set<int>& rowsToRemove) {
    Storage sto(1000);
    auto labels = origSto.getColumnLabels();
    auto numOrigColumns = origSto.getColumnLabels().getSize() - 1;

    // Remove in reverse order so it's easier to keep track of indices.
     for (auto it = rowsToRemove.rbegin(); it != rowsToRemove.rend(); ++it) {
         labels.remove(*it);
     }
    sto.setColumnLabels(labels);

    double time;
    for (int itime = 0; itime < origSto.getSize(); ++itime) {
        SimTK::Vector rowData(numOrigColumns);
        origSto.getData(itime, numOrigColumns, rowData);

        SimTK::Vector newRowData(numOrigColumns - (int)rowsToRemove.size());
        int iNew = 0;
        for (int iOrig = 0; iOrig < numOrigColumns; ++iOrig) {
            if (rowsToRemove.count(iOrig) == 0) {
                newRowData[iNew] = rowData[iOrig];
                ++iNew;
            }
        }

        origSto.getTime(itime, time);
        sto.append(time, newRowData);
    }
    return sto;
}

void testFromStatesStorageInconsistentModel() {

    // States are missing from the Storage.
    // ------------------------------------
    {
        Model model("gait2354_simbody.osim");
        Storage sto(statesStoFname);

        // Create new Storage with fewer columns.
        auto labels = sto.getColumnLabels();
        auto origLabel10 = labels[10];
        auto origLabel15 = labels[15];
        Storage stoMissingCols = newStorageWithRemovedRows(sto, {10, 15});

        // Test that an exception is thrown.
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, stoMissingCols),
                MissingColumnsInStatesStorage
                );
        // Check some other similar calls.
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, stoMissingCols,
                    false, true),
                MissingColumnsInStatesStorage
                );
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, stoMissingCols,
                    false, false),
                MissingColumnsInStatesStorage
                );

        // No exception if allowing missing columns.
        #if defined(NDEBUG)
            // The unspecified states are set to NaN (for at least two random
            // states).
            auto states = StatesTrajectory::createFromStatesStorage(
                    model, stoMissingCols, true);
            model.initSystem();
            SimTK_TEST(SimTK::isNaN(
                        model.getStateVariableValue(states[0], origLabel10)));
            SimTK_TEST(SimTK::isNaN(
                        model.getStateVariableValue(states[4], origLabel15)));
            // Behavior is independent of value for allowMissingColumns.
            StatesTrajectory::createFromStatesStorage(model, stoMissingCols,
                    true, true);
            StatesTrajectory::createFromStatesStorage(model, stoMissingCols,
                    true, false);
        #else
            // In DEBUG, even setting a state variable to NaN causes an
            // exception.
            SimTK_TEST_MUST_THROW(StatesTrajectory::createFromStatesStorage(
                        model, stoMissingCols, true));
        #endif
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
                ExtraColumnsInStatesStorage
                );
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, sto,
                    true, false),
                ExtraColumnsInStatesStorage
                );
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, sto,
                    false, false),
                ExtraColumnsInStatesStorage
                );

        // No exception if allowing extra columns, and behavior is
        // independent of value for allowMissingColumns.
        StatesTrajectory::createFromStatesStorage(model, sto, true, true);
        StatesTrajectory::createFromStatesStorage(model, sto, false, true);
    }
}

void testFromStatesStorageUniqueColumnLabels() {

    Model model("gait2354_simbody.osim");
    Storage sto(statesStoFname);
    
    // Edit column labels so that they are not unique.
    auto labels = sto.getColumnLabels();
    labels[10] = labels[7];
    sto.setColumnLabels(labels); 
   
    SimTK_TEST_MUST_THROW_EXC(
            StatesTrajectory::createFromStatesStorage(model, sto),
            NonUniqueColumnsInStatesStorage);
    SimTK_TEST_MUST_THROW_EXC(
            StatesTrajectory::createFromStatesStorage(model, sto, true, true),
            NonUniqueColumnsInStatesStorage);
    SimTK_TEST_MUST_THROW_EXC(
            StatesTrajectory::createFromStatesStorage(model, sto, true, false),
            NonUniqueColumnsInStatesStorage);
    SimTK_TEST_MUST_THROW_EXC(
            StatesTrajectory::createFromStatesStorage(model, sto, false, true),
            NonUniqueColumnsInStatesStorage);
    SimTK_TEST_MUST_THROW_EXC(
            StatesTrajectory::createFromStatesStorage(model, sto, false, false),
            NonUniqueColumnsInStatesStorage);

    // TODO unique even considering old and new formats for state variable
    // names (/value and /speed).
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

void testAppendTimesAreNonDecreasing() {
    Model model("gait2354_simbody.osim");
    auto& state = model.initSystem();
    state.setTime(1.0);

    StatesTrajectory states;
    states.append(state);

    // Multiple states can have the same time; does not throw an exception:
    states.append(state);

    state.setTime(0.9999);
    SimTK_TEST_MUST_THROW_EXC(states.append(state),
            SimTK::Exception::APIArgcheckFailed);
}

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
    SimTK_START_TEST("testStatesTrajectory");
    
        // Make sure the states Storage file doesn't already exist.
        remove(statesStoFname.c_str());

        SimTK_SUBTEST(testPopulateTrajectory);

        // Test creation of trajectory from states storage.
        // ------------------------------------------------
        createStateStorageFile();
        SimTK_SUBTEST(testFromStatesStorageGivesCorrectStates);
        SimTK_SUBTEST(testFromStatesStorageInconsistentModel);
        SimTK_SUBTEST(testFromStatesStorageUniqueColumnLabels);

        // TODO SimTK_SUBTEST(testEqualityOperator);

        SimTK_SUBTEST(testAppendTimesAreNonDecreasing);
        // TODO read from proper State serialization.
        SimTK_SUBTEST(testModifyStates);

    SimTK_END_TEST();
}
