/* -------------------------------------------------------------------------- *
 *                     OpenSim:  testStatesTrajectory.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Actuators/PointActuator.h>
#include <random>
#include <cstdio>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace SimTK;

// Small to-do's:
// TODO nondecreasing or increasing? might affect upper_bound/lower_bound.
// TODO detailed exceptions when integrity checks fail.
// TODO currently, one gets segfaults if state is not realized.

// Big to-do's:
// TODO append two StateTrajectories together.
// TODO test modeling options (locked coordinates, etc.)
// TODO store a model within a StatesTrajectory.

const std::string statesStoFname = "testStatesTrajectory_readStorage_states.sto";
const std::string pre40StoFname = "std_subject01_walk1_states.sto";

// Helper function to get a state variable value from a storage file.
Real getStorageEntry(const Storage& sto,
        const int timeIndex, const std::string& columnName) {
    Real value;
    const int columnIndex = sto.getStateIndex(columnName);
    sto.getData(timeIndex, columnIndex, value);
    return value;
}

void testPopulateTrajectoryAndStatesTrajectoryReporter() {
    Model model("gait2354_simbody.osim");

    // To assist with creating interesting (non-zero) coordinate values:
    model.updCoordinateSet().get("pelvis_ty").setDefaultLocked(true);

    // Also, test the StatesTrajectoryReporter.
    auto* statesCol = new StatesTrajectoryReporter();
    statesCol->setName("states_collector_all_steps");
    model.addComponent(statesCol);

        const double finalTime = 0.05;
    {
        auto& state = model.initSystem();

        SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
        SimTK::TimeStepper ts(model.getSystem(), integrator);
        ts.initialize(state);
        ts.setReportAllSignificantStates(true);
        integrator.setReturnEveryInternalStep(true);

        StatesTrajectory states;
        std::vector<double> times;
        while (ts.getState().getTime() < finalTime) {
            ts.stepTo(finalTime);
            times.push_back(ts.getState().getTime());
            // StatesTrajectory API for appending states:
            states.append(ts.getState());
            // For the StatesTrajectoryReporter:
            model.getMultibodySystem().realize(ts.getState(), SimTK::Stage::Report);
        }

        // Make sure we have all the states
        SimTK_TEST_EQ((int)states.getSize(), (int)times.size());
        SimTK_TEST_EQ((int)statesCol->getStates().getSize(), (int)times.size());
        // ...and that they aren't all just references to the same single state.
        for (int i = 0; i < (int)states.getSize(); ++i) {
            SimTK_TEST_EQ(states[i].getTime(), times[i]);
            SimTK_TEST_EQ(statesCol->getStates()[i].getTime(), times[i]);
        }
    }

    // Test the StatesTrajectoryReporter with a constant reporting interval.
    statesCol->clear();
    auto* statesColInterval = new StatesTrajectoryReporter();
    statesColInterval->setName("states_collector_interval");
    statesColInterval->set_report_time_interval(0.01);
    model.addComponent(statesColInterval);

    {
        auto& state = model.initSystem();
        SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
        SimTK::TimeStepper ts(model.getSystem(), integrator);
        ts.initialize(state);
        ts.setReportAllSignificantStates(true);
        integrator.setReturnEveryInternalStep(true);

        while (ts.getState().getTime() < finalTime) {
            ts.stepTo(finalTime);
            model.getMultibodySystem().realize(ts.getState(), SimTK::Stage::Report);
        }

        SimTK_TEST(statesColInterval->getStates().getSize() == 6);
        std::vector<double> times { 0, 0.01, 0.02, 0.03, 0.04, 0.05 };
        int i = 0;
        for (const auto& s : statesColInterval->getStates()) {
            ASSERT_EQUAL(s.getTime(), times[i], 1e-5);
            ++i;
        }
    }
}

void testFrontBack() {
    Model model("arm26.osim");
    const auto& state = model.initSystem();
    StatesTrajectory states;
    states.append(state);
    states.append(state);
    states.append(state);

    SimTK_TEST(&states.front() == &states[0]);
    SimTK_TEST(&states.back() == &states[2]);
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
            Constant(distribution(generator))
            );
    }

    model.addController(controller);

    auto& initState = model.initSystem();
    Manager manager(model);
    initState.setTime(0.0);
    manager.initialize(initState);
    manager.integrate(0.15);
    manager.getStateStorage().print(statesStoFname);
}

void testFromStatesStorageGivesCorrectStates() {

    // Read in trajectory.
    // -------------------
    Model model("gait2354_simbody.osim");

    Storage sto(statesStoFname);

    // Note: we are verifying that we can load a trajectory without
    // invoking model.initSystem() ourselves.
    auto states = StatesTrajectory::createFromStatesStorage(model, sto);

    // However, we eventually *do* need to call initSystem() to make use of the
    // trajectory with the model.
    model.initSystem();

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
            // get value and speed state names for this coordinate
            const auto coordStateNames = coord.getStateVariableNames();

            // Coordinate.
            SimTK_TEST_EQ(getStorageEntry(sto, itime, coordStateNames[0]),
                    coord.getValue(state));

            // Speed.
            SimTK_TEST_EQ(getStorageEntry(sto, itime, coordStateNames[1]),
                    coord.getSpeedValue(state));
        }

        // Muscle states.
        for (int im = 0; im < model.getMuscles().getSize(); ++im) {
            const auto& muscle = model.getMuscles().get(im);
            // get the activation and fiber_length state names of the muscles
            const auto muscleStateNames = muscle.getStateVariableNames();

            // Activation.
            {
                SimTK_TEST_EQ(
                    getStorageEntry(sto, itime, muscleStateNames[0]),
                        muscle.getStateVariableValue(state, "activation") );
            }

            // Fiber length.
            {
                SimTK_TEST_EQ(
                    getStorageEntry(sto, itime, muscleStateNames[1]),
                        muscle.getStateVariableValue(state, "fiber_length") );
            }

        }

        // More complicated computation based on state.
        // These calculations require that the state is realized to velocity.
        model.getMultibodySystem().realize(state, SimTK::Stage::Velocity);

        for (int im = 0; im < model.getMuscles().getSize(); ++im) {
            const auto& muscle = model.getMuscles().get(im);
            auto muscleName = muscle.getName();
            SimTK_TEST(!SimTK::isNaN(muscle.getFiberForce(state)));
        }

        auto loc = model.getBodySet().get("tibia_r")
            .findStationLocationInAnotherFrame(state,
                    SimTK::Vec3(1, 0.5, 0.25),
                    model.getGround());
        SimTK_TEST(!loc.isNaN());

        SimTK_TEST(!model.calcMassCenterVelocity(state).isNaN());

        // Make sure that we can realize to Dynamics without an issue.
        // There used to be a bug (GitHub Issue #1455) wherein Instance-stage
        // cache variables contain raw pointers to SimTK::Force objects,
        // therefore one cannot use such a state with different instances
        // of the same model.
        model.getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
        SimTK_TEST(!SimTK::isNaN(
                    model.getMuscles().get(0).getActiveFiberForce(state)));

        // Similarly, make sure we can do an acceleration-level calculation.
        model.getMultibodySystem().realize(state, SimTK::Stage::Acceleration);
        SimTK_TEST(!model.calcMassCenterAcceleration(state).isNaN());

        itime++;
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

void testFromStatesStorageInconsistentModel(const std::string &stoFilepath) {

    // States are missing from the Storage.
    // ------------------------------------
    {
        Model model("gait2354_simbody.osim");
        model.initSystem();

        const auto stateNames = model.getStateVariableNames();
        Storage sto(stoFilepath);
        // So the test doesn't take so long.
        sto.resampleLinear((sto.getLastTime() - sto.getFirstTime()) / 10);

        // Create new Storage with fewer columns.
        auto labels = sto.getColumnLabels();

        auto origLabel10 = stateNames[10];
        auto origLabel15 = stateNames[15];
        Storage stoMissingCols = newStorageWithRemovedRows(sto, {
                // gymnastics to be compatible with pre-v4.0 column names:
                sto.getStateIndex(origLabel10) + 1,
                sto.getStateIndex(origLabel15) + 1});

        // Test that an exception is thrown.
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, stoMissingCols),
                StatesTrajectory::MissingColumns
                );
        // Check some other similar calls.
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, stoMissingCols,
                    false, true),
                StatesTrajectory::MissingColumns
                );
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, stoMissingCols,
                    false, false),
                StatesTrajectory::MissingColumns
                );

        // No exception if allowing missing columns.
        // The unspecified states are set to NaN (for at least two random
        // states).
        auto states = StatesTrajectory::createFromStatesStorage(
                model, stoMissingCols, true);
        SimTK_TEST(SimTK::isNaN(
                    model.getStateVariableValue(states[0], origLabel10)));
        SimTK_TEST(SimTK::isNaN(
                    model.getStateVariableValue(states[4], origLabel15)));
        // Behavior is independent of value for allowMissingColumns.
        StatesTrajectory::createFromStatesStorage(model, stoMissingCols,
                true, true);
        StatesTrajectory::createFromStatesStorage(model, stoMissingCols,
                true, false);
    }

    // States are missing from the Model.
    // ----------------------------------
    {
        Model model("gait2354_simbody.osim");
        Storage sto(stoFilepath);
        // So the test doesn't take so long.
        sto.resampleLinear((sto.getLastTime() - sto.getFirstTime()) / 10);

        // Remove a few of the muscles.
        model.updForceSet().remove(0);
        model.updForceSet().remove(10);
        model.updForceSet().remove(30);

        // Test that an exception is thrown.
        // Must call initSystem() here; otherwise the _propertySubcomponents
        // would be stale and would still include the forces we removed above.
        model.initSystem();
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, sto),
                StatesTrajectory::ExtraColumns
                );
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, sto,
                    true, false),
                StatesTrajectory::ExtraColumns
                );
        SimTK_TEST_MUST_THROW_EXC(
                StatesTrajectory::createFromStatesStorage(model, sto,
                    false, false),
                StatesTrajectory::ExtraColumns
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
            NonUniqueLabels);
    SimTK_TEST_MUST_THROW_EXC(
            StatesTrajectory::createFromStatesStorage(model, sto, true, true),
            NonUniqueLabels);
    SimTK_TEST_MUST_THROW_EXC(
            StatesTrajectory::createFromStatesStorage(model, sto, true, false),
            NonUniqueLabels);
    SimTK_TEST_MUST_THROW_EXC(
            StatesTrajectory::createFromStatesStorage(model, sto, false, true),
            NonUniqueLabels);
    SimTK_TEST_MUST_THROW_EXC(
            StatesTrajectory::createFromStatesStorage(model, sto, false, false),
            NonUniqueLabels);

    // TODO unique even considering old and new formats for state variable
    // names (/value and /speed) in the same file.
}

void testFromStatesStoragePre40CorrectStates() {
    // This test is very similar to testFromStatesStorageGivesCorrectStates
    // TODO could avoid the duplicate function since getStateIndex handles
    // pre-4.0 names.

    // Read in trajectory.
    // -------------------
    Model model("gait2354_simbody.osim");

    Storage sto(pre40StoFname);
    // So the test doesn't take so long.
    sto.resampleLinear(0.01);
    auto states = StatesTrajectory::createFromStatesStorage(model, sto);
    model.initSystem();

    // Test that the states are correct.
    // ---------------------------------
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

            // Coordinate.
            SimTK_TEST_EQ(getStorageEntry(sto, itime, coordName),
                    coord.getValue(state));

            // Speed.
            SimTK_TEST_EQ(getStorageEntry(sto, itime, coordName + "_u"),
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
                    getStorageEntry(sto, itime, muscleName + ".activation"),
                    muscle.getActivation(state));

            // Fiber length.
            SimTK_TEST_EQ(
                    getStorageEntry(sto, itime, muscleName + ".fiber_length"),
                    muscle.getFiberLength(state));

            // More complicated computation based on state.
            SimTK_TEST(!SimTK::isNaN(muscle.getFiberForce(state)));
        }

        // More complicated computations based on state.
        auto loc = model.getBodySet().get("tibia_r")
            .findStationLocationInAnotherFrame(state,
                    SimTK::Vec3(1, 0.5, 0.25),
                    model.getGround());
        SimTK_TEST(!loc.isNaN());

        SimTK_TEST(!model.calcMassCenterVelocity(state).isNaN());

        // Make sure that we can realize to Dynamics without an issue.
        // There used to be a bug (GitHub Issue #1455) wherein Instance-stage
        // cache variables contain raw pointers to SimTK::Force objects,
        // therefore one cannot use such a state with different instances
        // of the same model.
        model.getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
        SimTK_TEST(!SimTK::isNaN(
                    model.getMuscles().get(0).getActiveFiberForce(state)));

        // Similarly, make sure we can do an acceleration-level calculation.
        model.getMultibodySystem().realize(state, SimTK::Stage::Acceleration);
        SimTK_TEST(!model.calcMassCenterAcceleration(state).isNaN());

        itime++;
    }
}


void testCopying() {
    Model model("gait2354_simbody.osim");
    auto& state = model.initSystem();

    StatesTrajectory states;
    {
        state.setTime(0.5);
        states.append(state);
        state.setTime(1.3);
        states.append(state);
        state.setTime(3.5);
        states.append(state);
    }

    {
        StatesTrajectory statesCopyConstruct(states);
        // Ideally we'd check for equality (operator==()), but State does not
        // have an equality operator.
        SimTK_TEST_EQ((int)statesCopyConstruct.getSize(), (int)states.getSize());
        for (size_t i = 0; i < states.getSize(); ++i) {
            SimTK_TEST_EQ(statesCopyConstruct[i].getTime(), states[i].getTime());
        }
    }

    {
        StatesTrajectory statesCopyAssign;
        statesCopyAssign = states;
        SimTK_TEST_EQ((int)statesCopyAssign.getSize(), (int)states.getSize());
        for (size_t i = 0; i < states.getSize(); ++i) {
            SimTK_TEST_EQ(statesCopyAssign[i].getTime(), states[i].getTime());
        }
    }
}

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

void testBoundsCheck() {
    Model model("gait2354_simbody.osim");
    const auto& state = model.initSystem();
    StatesTrajectory states;
    states.append(state);
    states.append(state);
    states.append(state);
    states.append(state);

    #ifdef NDEBUG
        // In DEBUG, Visual Studio puts asserts into the index operator.
        states[states.getSize() + 100];
        states[4];
        states[5];
    #endif

    // SimTK::Array_<> only throws exceptions when in a DEBUG build
    #ifdef DEBUG
        SimTK_TEST_MUST_THROW_EXC(states.get(4), IndexOutOfRange);
        SimTK_TEST_MUST_THROW_EXC(states.get(states.getSize() + 100),
                                  IndexOutOfRange);
    #endif
}

void testIntegrityChecks() {
    Model arm26("arm26.osim");
    const auto& s26 = arm26.initSystem();

    Model gait2354("gait2354_simbody.osim");
    const auto& s2354 = gait2354.initSystem();
    // TODO add models with events, unilateral constraints, etc.

    // Times are nondecreasing.
    {
        StatesTrajectory states;
        auto state0(s26);
        state0.setTime(0.5);
        auto state1(s26);
        state1.setTime(0.6);

        states.append(state0);
        states.append(state1);

        SimTK_TEST(states.isConsistent());
        SimTK_TEST(states.isNondecreasingInTime());
        SimTK_TEST(states.hasIntegrity());

        // Users should never do this const cast; it's just for the sake of
        // the test.
        const_cast<SimTK::State*>(&states[1])->setTime(0.2);

        SimTK_TEST(states.isConsistent());
        SimTK_TEST(!states.isNondecreasingInTime());
        SimTK_TEST(!states.hasIntegrity());
    }

    // Consistency and compatibility with a model.
    {
        StatesTrajectory states;
        // An empty trajectory is consistent.
        SimTK_TEST(states.isConsistent());

        // A length-1 trajectory is consistent.
        states.append(s26);
        SimTK_TEST(states.isConsistent());

        // This trajectory is compatible with the arm26 model.
        SimTK_TEST(states.isCompatibleWith(arm26));

        // Not compatible with gait2354 model.
        // Ensures a lower-dimensional trajectory can't pass for a higher
        // dimensional model.
        SimTK_TEST(!states.isCompatibleWith(gait2354));

        // The checks still work with more than 1 state.
        states.append(s26);
        states.append(s26);
        SimTK_TEST(states.isNondecreasingInTime());
        SimTK_TEST(states.isConsistent());
        SimTK_TEST(states.hasIntegrity());
        SimTK_TEST(states.isCompatibleWith(arm26));
        SimTK_TEST(!states.isCompatibleWith(gait2354));
    }

    {
        StatesTrajectory states;
        states.append(s2354);

        // Reverse of the previous check; to ensure that a larger-dimensional
        // trajectory can't pass for the smaller dimensional model.
        SimTK_TEST(states.isCompatibleWith(gait2354));
        SimTK_TEST(!states.isCompatibleWith(arm26));

        // Check still works with more than 1 state.
        states.append(s2354);
        states.append(s2354);
        SimTK_TEST(states.isNondecreasingInTime());
        SimTK_TEST(states.isConsistent());
        SimTK_TEST(states.hasIntegrity());
        SimTK_TEST(states.isCompatibleWith(gait2354));
        SimTK_TEST(!states.isCompatibleWith(arm26));
    }

    {
        // Cannot append inconsistent states.
        StatesTrajectory states;
        states.append(s26);
        SimTK_TEST_MUST_THROW_EXC(states.append(s2354),
                StatesTrajectory::InconsistentState);

        // Same check, but swap the models.
        StatesTrajectory states2;
        states2.append(s2354);
        SimTK_TEST_MUST_THROW_EXC(states2.append(s26),
                StatesTrajectory::InconsistentState);
    }

    // TODO Show weakness of the test: two models with the same number of Q's, U's,
    // and Z's both pass the check.
}

void tableAndTrajectoryMatch(const Model& model,
                             const TimeSeriesTable& table,
                             const StatesTrajectory& states,
                             std::vector<std::string> columns = {}) {

    const auto stateNames = model.getStateVariableNames();

    size_t numColumns{};
    if (columns.empty()) {
        numColumns = stateNames.getSize();
    } else {
        numColumns = columns.size();
    }
    SimTK_TEST(table.getNumColumns() == numColumns);
    SimTK_TEST(table.getNumRows() == states.getSize());

    const auto& colNames = table.getColumnLabels();

    std::vector<int> stateValueIndices(colNames.size());
    for (size_t icol = 0; icol < colNames.size(); ++icol) {
        stateValueIndices[icol] = stateNames.findIndex(colNames[icol]);
    }

    SimTK::Vector stateValues; // working memory

    // Test that the data table has exactly the same numbers.
    for (size_t itime = 0; itime < states.getSize(); ++itime) {
        // Test time.
        SimTK_TEST(table.getIndependentColumn()[itime] ==
                   states[itime].getTime());

        stateValues = model.getStateVariableValues(states[itime]);

        // Test state values.
        for (size_t icol = 0; icol < table.getNumColumns(); ++icol) {
            const auto& valueInStates = stateValues[stateValueIndices[icol]];

            const auto& column = table.getDependentColumnAtIndex(icol);
            const auto& valueInTable = column[static_cast<int>(itime)];

            SimTK_TEST(valueInStates == valueInTable);
        }
    }
}

void testExport() {
    Model gait("gait2354_simbody.osim");
    gait.initSystem();

    // Exported data exactly matches data in the trajectory.
    const auto stateNames = gait.getStateVariableNames();
    Storage sto(statesStoFname);
    auto states = StatesTrajectory::createFromStatesStorage(gait, sto);

    {
        auto tableAll = states.exportToTable(gait);
        tableAndTrajectoryMatch(gait, tableAll, states);
    }

    // Exporting only certain columns.
    std::vector<std::string> columns{
        gait.getCoordinateSet().get("knee_angle_l").getStateVariableNames()[0],
        gait.getCoordinateSet().get("knee_angle_r").getStateVariableNames()[0],
        gait.getCoordinateSet().get("knee_angle_r").getStateVariableNames()[1]};

    {
        auto tableKnee = states.exportToTable(gait, columns);
        tableAndTrajectoryMatch(gait, tableKnee, states, columns);
    }

    // Trying to export the trajectory with an incompatible model.
    {
        Model arm26("arm26.osim");
        SimTK::State differentState = arm26.initSystem();
        SimTK_TEST_MUST_THROW_EXC(states.exportToTable(arm26),
                                  StatesTrajectory::IncompatibleModel);
    }

    // Exception if given a non-existent column name.
    SimTK_TEST_MUST_THROW_EXC(
            states.exportToTable(gait, {columns[0],
                                        "not_an_actual_state",
                                        columns[2]}),
            OpenSim::Exception);
    SimTK_TEST_MUST_THROW_EXC(
            states.exportToTable(gait, {columns[0],
                                        "nor/is/this",
                                        columns[2]}),
            OpenSim::Exception);
}

int main() {
    SimTK_START_TEST("testStatesTrajectory");

    // The following model(s) contains Actuators that are registered when the
    // osimActuators library is loaded. But unless we call at least one
    // function defined in the osimActuators library, some linkers will omit
    // its dependency from the executable and it will not be loaded at
    // startup.
    { PointActuator t; }

        // Make sure the states Storage file doesn't already exist; we'll
        // generate it later and we don't want to use a stale one by accident.
        remove(statesStoFname.c_str());

        SimTK_SUBTEST(testPopulateTrajectoryAndStatesTrajectoryReporter);
        SimTK_SUBTEST(testFrontBack);
        SimTK_SUBTEST(testBoundsCheck);
        SimTK_SUBTEST(testIntegrityChecks);
        SimTK_SUBTEST(testAppendTimesAreNonDecreasing);
        SimTK_SUBTEST(testCopying);

        // Test creation of trajectory from a states storage.
        // -------------------------------------------------
        // Using a pre-4.0 states storage file with old column names.
        SimTK_SUBTEST(testFromStatesStoragePre40CorrectStates);
        SimTK_SUBTEST1(testFromStatesStorageInconsistentModel, pre40StoFname);

        // v4.0 states storage
        createStateStorageFile();
        SimTK_SUBTEST(testFromStatesStorageGivesCorrectStates);
        SimTK_SUBTEST1(testFromStatesStorageInconsistentModel, statesStoFname);
        SimTK_SUBTEST(testFromStatesStorageUniqueColumnLabels);

        // Export to data table.
        SimTK_SUBTEST(testExport);

    SimTK_END_TEST();
}
