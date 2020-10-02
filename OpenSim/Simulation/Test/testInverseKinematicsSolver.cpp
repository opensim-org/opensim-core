/* -------------------------------------------------------------------------- *
 *                OpenSim:  testInverseKinematicsSolver.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

//=============================================================================
// testInverseKinematicsSolver verifies that changes to the accuracy and marker
// weights have expected effects on the inverse kinematics results/errors
//=============================================================================
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/BufferedOrientationsReference.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <random>

using namespace OpenSim;
using namespace std;


// Verify that the marker weight are consistent with the initial Set
// of MarkerWeights used to construct the MarkersReference
void testMarkersReference();
// Verify that the orientations sensor weights are consistent with the initial
// Set of OrientationWeights used to construct the OrientationsReference
void testOrientationsReference();

// Utility function to build a simple pendulum with markers attached
Model* constructPendulumWithMarkers();
// Using a model with markers and trajectory of states, create synthetic
// marker data. If noiseRadius is provided use it to scale the noise
// that perturbs the marker data. Optionally, use the constantOffset
// parameter true to use the same noise for each time frame, otherwise
// randomly select the noise to be added at each frame.
TimeSeriesTable_<SimTK::Vec3>
generateMarkerDataFromModelAndStates(const Model& model,
                        const StatesTrajectory& states,
                        const SimTK::RowVector_<SimTK::Vec3>& biases,
                        double noiseRadius = 0, 
                        bool constantOffset = false);

// Utility function to build a simple 3dof leg model
Model* constructLegWithOrientationFrames();
// Using a model with orientation reference frames and a trajectory of
// states, create synthetic orientation data. If noiseRadius is provided
// use it to scale the noise that perturbs the orientation data. Optionally,
// use the constantOffset parameter true to use the same noise for each time
// frame, otherwise randomly select the noise to be added at each frame.
TimeSeriesTable_<SimTK::Rotation>
generateOrientationsDataFromModelAndStates(const Model& model,
    const StatesTrajectory& states,
    const SimTK::RowVector_<SimTK::Rotation>& biases,
    double noiseRadius,
    bool constantOffset = false);

// Verify that accuracy improves the number of decimals points to which
// the solver solution (coordinates) can be trusted as it is tightened.
void testAccuracy();
// Verify that the marker weights impact the solver and has the expected
// effect of reducing the error for the marker weight that is increased. 
void testUpdateMarkerWeights();
// Verify that the track() solution is also effected by updating marker
// weights and marker error is being reduced as its weighting increases.
void testTrackWithUpdateMarkerWeights();

// Verify that solver does not confuse/mismanage markers when reference
// has more markers than the model, order is changed or marker reference
// includes intervals with NaNs (no observation)
void testNumberOfMarkersMismatch();
void testNumberOfOrientationsMismatch();

int main()
{
    SimTK::Array_<std::string> failures;

    try { testMarkersReference(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testMarkersReference");
    }
    try { testOrientationsReference(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testOrientationsReference");
    }
    
    try { testAccuracy(); }
    catch (const std::exception& e) {
        cout << e.what() << endl; failures.push_back("testAccuracy");
    }
    try { testUpdateMarkerWeights(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testUpdateMarkerWeights");
    }
    try { testTrackWithUpdateMarkerWeights(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testTrackWithUpdateMarkerWeights");
    }

    try { testNumberOfMarkersMismatch(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testNumberOfMarkersMismatch");
    }

    try { testNumberOfOrientationsMismatch(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testNumberOfOrientationsMismatch");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done. All cases passed." << endl;

    return 0;
}

//=============================================================================
// Test Cases
//=============================================================================
void testMarkersReference()
{
    // column labels for marker data
    vector<std::string> labels{ "A", "B", "C", "D", "E", "F" };
    // for testing construct a set of marker weights is a different order 
    vector<int> order = { 3, 5, 1, 4, 0, 2 };

    size_t nc = labels.size();
    size_t nr = 5;

    TimeSeriesTable_<SimTK::Vec3> markerData;
    markerData.setColumnLabels(labels);
    for (size_t r{0}; r < nr; ++r) {
        SimTK::RowVector_<SimTK::Vec3> row{ int(nc), SimTK::Vec3(0) };
        markerData.appendRow(0.1*r, row);
    }

    Set<MarkerWeight> markerWeights;
    for (size_t m{0}; m < nc; ++m)
    markerWeights.adoptAndAppend(
        new MarkerWeight(labels[order[m]], double(order[m])) );

    std::cout << markerWeights.dump() << std::endl;

    MarkersReference markersRef(markerData, markerWeights);

    Model model;
    SimTK::State& s = model.initSystem();
    s.updTime() = 0.0;

    SimTK::Array_<string> names = markersRef.getNames();

    SimTK::Array_<double> weights;
    markersRef.getWeights(s, weights);

    SimTK_ASSERT_ALWAYS(names.size() == weights.size(), 
        "Number of markers does not match number of weights.");

    for (unsigned int i{ 0 }; i < names.size(); ++i) {
        std::cout << names[i] << ": " << weights[i] << std::endl;
        SimTK_ASSERT_ALWAYS(weights[i] == double(i),
            "Mismatched weight to marker.");
    }

    // Add marker weights for markers not present in the data
    markerWeights.adoptAndAppend(new MarkerWeight("X", 0.1));
    markerWeights.insert(0, new MarkerWeight("Y", 0.01));

    MarkersReference markersRef2(markerData, markerWeights);

    auto& mWeightSet = markersRef2.get_marker_weights();

    // verify that internal weight set was updated 
    std::cout << mWeightSet.dump() << std::endl;

    names = markersRef2.getNames();
    markersRef2.getWeights(s, weights);

    SimTK_ASSERT_ALWAYS(names.size() == weights.size(),
        "Number of markers does not match number of weights.");

    for (unsigned int i=0; i < names.size(); ++i) {
        std::cout << names[i] << ": " << weights[i] << std::endl;
        SimTK_ASSERT_ALWAYS(weights[i] == double(i),
            "Mismatched weight to marker.");
    }
}

void testOrientationsReference() {
    // column labels for orientation sensor data
    vector<std::string> labels{"A", "B", "C", "D", "E", "F"};
    // for testing construct a set of marker weights in a different order
    vector<int> order = {3, 5, 1, 4, 0, 2};

    size_t nc = labels.size(); // number of columns of orientation data
    size_t nr = 5;             // number of rows of orientation data

    TimeSeriesTable_<SimTK::Rotation> orientationData;
    orientationData.setColumnLabels(labels);
    for (size_t r{0}; r < nr; ++r) {
        SimTK::RowVector_<SimTK::Rotation> row{int(nc), SimTK::Rotation()};
        orientationData.appendRow(0.1 * r, row);
    }

    Set<OrientationWeight> orientationWeights;
    for (size_t m{0}; m < nc; ++m)
        orientationWeights.adoptAndAppend(
                new OrientationWeight(labels[order[m]], double(order[m])));

    std::cout << orientationWeights.dump() << std::endl;

    OrientationsReference orientationsRef(orientationData, &orientationWeights);

    Model model;
    SimTK::State& s = model.initSystem();
    s.updTime() = 0.0;

    SimTK::Array_<string> names = orientationsRef.getNames();

    SimTK::Array_<double> weights;
    orientationsRef.getWeights(s, weights);

    SimTK_ASSERT_ALWAYS(names.size() == weights.size(),
            "Number of markers does not match number of weights.");

    for (unsigned int i{0}; i < names.size(); ++i) {
        std::cout << names[i] << ": " << weights[i] << std::endl;
        SimTK_ASSERT_ALWAYS(
                weights[i] == double(i), "Mismatched weight to marker.");
    }

    // Add marker weights for markers not present in the data
    orientationWeights.adoptAndAppend(new OrientationWeight("X", 0.1));
    orientationWeights.insert(0, new OrientationWeight("Y", 0.01));

    OrientationsReference orientationsRef2(
            orientationData, &orientationWeights);

    auto& oWeightSet = orientationsRef2.get_orientation_weights();

    // verify that internal weight set was updated
    std::cout << oWeightSet.dump() << std::endl;

    names = orientationsRef2.getNames();
    orientationsRef2.getWeights(s, weights);

    SimTK_ASSERT_ALWAYS(names.size() == weights.size(),
            "Number of orientation sensors does not match number of weights.");

    for (unsigned int i = 0; i < names.size(); ++i) {
        std::cout << names[i] << ": " << weights[i] << std::endl;
        SimTK_ASSERT_ALWAYS(weights[i] == double(i),
                "Mismatched weight to orientation sensor.");
    }
}


void testAccuracy()
{
    cout << "\ntestInverseKinematicsSolver::testAccuracy()" << endl;

    std::unique_ptr<Model> pendulum{ constructPendulumWithMarkers() };
    Coordinate& coord = pendulum->getCoordinateSet()[0];

    double refVal = 0.123456789;
    double looseAccuracy = 1e-3;
    double tightAccuracy = 1.0e-9;

    SimTK::Array_<CoordinateReference> coordRefs;
    Constant coordRefFunc(refVal);
    CoordinateReference coordRef(coord.getName(), coordRefFunc);
    coordRef.setWeight(1.0);
    coordRefs.push_back(coordRef);

    SimTK::State state = pendulum->initSystem();
    double coordValue = coord.getValue(state);

    cout.precision(10);
    cout << "Initial " << coord.getName() << " value = " << coordValue <<
        " referenceValue = " << coordRefs[0].getValue(state) << endl;

    coord.setValue(state, refVal);
    StatesTrajectory states;
    states.append(state);

    SimTK::RowVector_<SimTK::Vec3> biases(3, SimTK::Vec3(0));
    std::shared_ptr<MarkersReference> 
        markersRef(
            new MarkersReference(generateMarkerDataFromModelAndStates(
                    *pendulum, states, biases),
            Set<MarkerWeight>()));
    markersRef->setDefaultWeight(1.0);

    // Reset the initial coordinate value
    coord.setValue(state, 0.0);
    InverseKinematicsSolver ikSolver(*pendulum, markersRef, coordRefs);
    ikSolver.setAccuracy(looseAccuracy);
    ikSolver.assemble(state);

    coordValue = coord.getValue(state);
    cout << "Assembled " << coord.getName() << " value = " << coordValue << endl;

    double accuracy = abs(coordValue - refVal);
    cout << "Specified accuracy: " << looseAccuracy << "; achieved: "
        << accuracy << endl;

    // verify that the target accuracy was met after assemble()
    SimTK_ASSERT_ALWAYS(accuracy <= looseAccuracy,
        "InverseKinematicsSolver assemble() failed to meet specified accuracy");

    ikSolver.track(state);
    coordValue = coord.getValue(state);
    cout << "Tracked " << coord.getName() << " value = " << coordValue << endl;
    accuracy = abs(coordValue - refVal);
    // verify that the target accuracy was met after track()
    SimTK_ASSERT_ALWAYS(accuracy <= looseAccuracy,
        "InverseKinematicsSolver track() failed to meet specified accuracy");

    SimTK::Array_<double> sqMarkerErrors;
    double looseSumSqError = 0;
    ikSolver.computeCurrentSquaredMarkerErrors(sqMarkerErrors);
    for (const double& err : sqMarkerErrors) {
        looseSumSqError += err;
    }
    
    cout << "For accuracy: " << looseAccuracy << "; Sum-squared Error: " 
        << looseSumSqError << endl;

    // Reset the initial coordinate value
    coord.setValue(state, 0.0);
    ikSolver.setAccuracy(tightAccuracy);

    // verify that track() throws after changing the accuracy of the Solver
    SimTK_TEST_MUST_THROW_EXC(ikSolver.track(state), Exception);

    ikSolver.setAccuracy(tightAccuracy);
    ikSolver.assemble(state);

    coordValue = coord.getValue(state);
    cout << "Assembled " << coord.getName() <<" value = "<< coordValue << endl;

    accuracy = abs(coord.getValue(state) - refVal);

    cout << "Specified accuracy: " << tightAccuracy << "; achieved: "
        << accuracy << endl;

    // verify that the target accuracy was met after tightening the accuracy
    SimTK_ASSERT_ALWAYS(accuracy <= tightAccuracy,
        "InverseKinematicsSolver assemble() failed to meet tightened accuracy");

    // perturb the solution to verify that track() achieves the accuracy
    coord.setValue(state, refVal-looseAccuracy);
    ikSolver.track(state);
    accuracy = abs(coord.getValue(state) - refVal);
    // verify that track() achieves the tightened accuracy
    SimTK_ASSERT_ALWAYS(accuracy <= tightAccuracy,
        "InverseKinematicsSolver track() failed to meet tightened accuracy");

    double tightSumSqError = 0;
    ikSolver.computeCurrentSquaredMarkerErrors(sqMarkerErrors);
    for (const double& err : sqMarkerErrors) {
        tightSumSqError += err;
    }

    cout << "For accuracy: " << tightAccuracy << "; Sum-squared Error: "
        << tightSumSqError << endl;

    // refining the accuracy should not increase tracking errors
    SimTK_ASSERT_ALWAYS(tightSumSqError <= looseSumSqError,
        "InverseKinematicsSolver failed to maintain or lower marker errors "
        "when accuracy was tightened.");
}

void testUpdateMarkerWeights()
{
    cout << "\ntestInverseKinematicsSolver::testUpdateMarkerWeights()" << endl;

    std::unique_ptr<Model> pendulum{ constructPendulumWithMarkers() };
    Coordinate& coord = pendulum->getCoordinateSet()[0];

    double refVal = 0.123456789;

    SimTK::State state = pendulum->initSystem();
    coord.setValue(state, refVal);

    StatesTrajectory states;
    states.append(state);

    SimTK::RowVector_<SimTK::Vec3> biases(3, SimTK::Vec3(0));
    std::shared_ptr<MarkersReference> 
        markersRef(
            new MarkersReference(generateMarkerDataFromModelAndStates(*pendulum,
                                                        states, biases,
                                                        0.02),
            Set<MarkerWeight>()));
    auto& markerNames = markersRef->getNames();

    for (const auto& name : markerNames) {
        markersRef->updMarkerWeightSet().adoptAndAppend(
            new MarkerWeight(name, 1.0));
    }

    SimTK::Array_<CoordinateReference> coordRefs;
    // Reset the initial coordinate value
    coord.setValue(state, 0.0);
    InverseKinematicsSolver ikSolver(*pendulum, markersRef, coordRefs);
    ikSolver.setAccuracy(1.0e-8);
    ikSolver.assemble(state);

    double coordValue = coord.getValue(state);
    cout << "Assembled " << coord.getName() << " value = "
        << coordValue << endl;

    SimTK::Array_<double> nominalMarkerErrors;
    ikSolver.computeCurrentMarkerErrors(nominalMarkerErrors);

    SimTK::Array_<double> markerWeights;
    markersRef->getWeights(state, markerWeights);

    for (unsigned int i = 0; i < markerNames.size(); ++i) {
        cout << markerNames[i] << "(weight = " << markerWeights[i]
            << ") error = " << nominalMarkerErrors[i] << endl;
    }

    // Increase the weight of the right marker 
    markerWeights[1] *= 10.0;
    ikSolver.updateMarkerWeights(markerWeights);

    // Reset the initial coordinate value
    coord.setValue(state, 0.0);
    ikSolver.track(state);

    coordValue = coord.getValue(state);
    cout << "Assembled " << coord.getName() << " value = "
        << coordValue << endl;

    SimTK::Array_<double> rightMarkerWeightedErrors;
    ikSolver.computeCurrentMarkerErrors(rightMarkerWeightedErrors);

    for (unsigned int i = 0; i < markerNames.size(); ++i) {
        cout << markerNames[i] << "(weight = " << markerWeights[i]
            << ") error = " << rightMarkerWeightedErrors[i] << endl;
    }

    // increasing the marker weight (marker[1] = "mR") should cause that marker
    // error to decrease
    SimTK_ASSERT_ALWAYS(rightMarkerWeightedErrors[1] < nominalMarkerErrors[1],
        "InverseKinematicsSolver failed to lower 'right' marker error when "
        "marker weight was increased.");

    // update the marker weights and repeat for the left hand marker "mL"
    markerWeights[2] *= 20.0; // "mL"
    ikSolver.updateMarkerWeights(markerWeights);

    // Reset the initial coordinate value and reassemble
    coord.setValue(state, 0.0);
    ikSolver.track(state);

    coordValue = coord.getValue(state);
    cout << "Assembled " << coord.getName() << " value = "
        << coordValue << endl;

    SimTK::Array_<double> leftMarkerWeightedErrors;
    ikSolver.computeCurrentMarkerErrors(leftMarkerWeightedErrors);

    for (unsigned int i = 0; i < markerNames.size(); ++i) {
        cout << markerNames[i] << "(weight = " << markerWeights[i]
            << ") error = " << leftMarkerWeightedErrors[i] << endl;
    }

    // increasing the marker weight (marker[2] = "mL") should cause that marker
    // error to decrease
    SimTK_ASSERT_ALWAYS(
        leftMarkerWeightedErrors[2] < rightMarkerWeightedErrors[2],
        "InverseKinematicsSolver failed to lower 'left' marker error when "
        "marker weight was increased.");
}

void testTrackWithUpdateMarkerWeights()
{
    cout << 
        "\ntestInverseKinematicsSolver::testTrackWithUpdateMarkerWeights()" 
        << endl;
    std::unique_ptr<Model> pendulum{ constructPendulumWithMarkers() };
    Coordinate& coord = pendulum->getCoordinateSet()[0];

    SimTK::State state = pendulum->initSystem();

    StatesTrajectory states;

    // sample time
    double dt = 0.01;

    for (int i = 0; i < 101; ++i) {
        state.updTime()=i*dt;
        coord.setValue(state, SimTK::Pi / 3);
        states.append(state);
    } 

    SimTK::RowVector_<SimTK::Vec3> biases(3, SimTK::Vec3(0));
    std::shared_ptr<MarkersReference> markersRef(
            new MarkersReference(generateMarkerDataFromModelAndStates(
                    *pendulum, states, biases, 0.02, true),
                    Set<MarkerWeight>()));
    auto& markerNames = markersRef->getNames();

    for (const auto& name : markerNames) {
        markersRef->updMarkerWeightSet().adoptAndAppend(
            new MarkerWeight(name, 1.0));
    }

    SimTK::Array_<CoordinateReference> coordRefs;
    // Reset the initial coordinate value
    coord.setValue(state, 0.0);
    InverseKinematicsSolver ikSolver(*pendulum, markersRef, coordRefs);
    ikSolver.setAccuracy(1e-6);
    ikSolver.assemble(state);

    SimTK::Array_<double> markerWeights;
    markersRef->getWeights(state, markerWeights);

    SimTK::Array_<double> leftMarkerWeightedErrors;

    double previousErr = 0.1;

    for (unsigned i = 0; i < markersRef->getNumFrames(); ++i) {
        state.updTime() = i*dt;
        // increment the weight of the left marker each time  
        markerWeights[2] = 0.1*i+1;
        ikSolver.updateMarkerWeights(markerWeights);
        ikSolver.track(state);

        if (i>0 && (i % 10 == 0)) {
            //get the marker errors
            ikSolver.computeCurrentMarkerErrors(leftMarkerWeightedErrors);

            cout << "time: " << state.getTime() << " | " << markerNames[2] 
                << "(weight = " << markerWeights[2] << ") error = " 
                << leftMarkerWeightedErrors[2] << endl;

            // increasing the marker weight (marker[2] = "mL") should cause
            //  that marker error to decrease
            SimTK_ASSERT_ALWAYS(
                leftMarkerWeightedErrors[2] < previousErr,
                "InverseKinematicsSolver track failed to lower 'left' "
                "marker error when marker weight was increased.");

            previousErr = leftMarkerWeightedErrors[2];
        }
    }
}

void testNumberOfMarkersMismatch()
{
    cout << 
        "\ntestInverseKinematicsSolver::testNumberOfMarkersMismatch()"
        << endl;

    std::unique_ptr<Model> pendulum{ constructPendulumWithMarkers() };
    const Coordinate& coord = pendulum->getCoordinateSet()[0];

    SimTK::State state = pendulum->initSystem();
    StatesTrajectory states;

    // sample time
    double dt = 0.1;
    int N = 11;
    for (int i = 0; i < N; ++i) {
        state.updTime() = i*dt;
        coord.setValue(state, i*dt*SimTK::Pi / 3);
        states.append(state);
    }

    double err = 0.05;
    SimTK::RowVector_<SimTK::Vec3> biases(3, SimTK::Vec3(0));
    // bias m0
    biases[0] += SimTK::Vec3(0, err, 0);
    cout << "biases: " << biases << endl;

    auto markerTable = generateMarkerDataFromModelAndStates(*pendulum,
                        states,
                        biases,
                        0.0,
                        true);

    SimTK::Vector_<SimTK::Vec3> unusedCol(N, SimTK::Vec3(0.987654321));

    auto usedMarkerNames = markerTable.getColumnLabels();

    // add an unused marker to the marker data
    markerTable.appendColumn("unused", unusedCol);

    cout << "Before:\n" << markerTable << endl;
    
    // re-order "observed" marker data
    SimTK::Matrix_<SimTK::Vec3> dataGutsCopy = markerTable.getMatrix();
    int last = dataGutsCopy.ncol() - 1;
    // swap first and last columns 
    markerTable.updMatrix()(0) = dataGutsCopy(last);
    markerTable.updMatrix()(last) = dataGutsCopy(0);
    auto columnNames = markerTable.getColumnLabels();
    markerTable.setColumnLabel(0, columnNames[last]);
    markerTable.setColumnLabel(last, columnNames[0]);
    columnNames = markerTable.getColumnLabels();

    // Inject NaN in "observations" of "mR" marker data
    for (int i = 4; i < 7; ++i) {
        markerTable.updMatrix()(i, 1) = SimTK::NaN;
    }

    cout << "After reorder and NaN injections:\n" << markerTable << endl;

    Set<MarkerWeight> markerWeightSet;
    std::shared_ptr<MarkersReference> markersRef(new MarkersReference(markerTable, markerWeightSet));
    int nmr = markersRef->getNumRefs();
    auto& markerNames = markersRef->getNames();
    cout << markerNames << endl;

    SimTK::Array_<CoordinateReference> coordRefs;
    // Reset the initial coordinate value
    coord.setValue(state, 0.0);
    InverseKinematicsSolver ikSolver(*pendulum, markersRef, coordRefs);
    double tol = 1e-4;
    ikSolver.setAccuracy(tol);
    ikSolver.assemble(state);

    int nm = ikSolver.getNumMarkersInUse();

    SimTK::Array_<double> markerErrors(nm);
    for (unsigned i = 0; i < markersRef->getNumFrames(); ++i) {
        state.updTime() = i*dt;
        ikSolver.track(state);

        //get the marker errors
        ikSolver.computeCurrentMarkerErrors(markerErrors);

        int nme = markerErrors.size();

        SimTK_ASSERT_ALWAYS(nme == nm,
            "InverseKinematicsSolver failed to account "
            "for unused marker reference (observation).");

        cout << "time: " << state.getTime() << " |";
        auto namesIter = usedMarkerNames.begin();
        for (int j = 0; j < nme; ++j) {
            const auto& markerName = ikSolver.getMarkerNameForIndex(j);
            cout << " " << markerName << " error = " << markerErrors[j];

            SimTK_ASSERT_ALWAYS( *namesIter++ != "unused",
                "InverseKinematicsSolver failed to ignore "
                "unused marker reference (observation).");
            
            if (markerName == "m0") {//should see error on biased marker
                SimTK_ASSERT_ALWAYS(abs(markerErrors[j]-err) <= tol,
                    "InverseKinematicsSolver mangled marker order.");
            }
            else { // other markers should be minimally affected
                SimTK_ASSERT_ALWAYS(markerErrors[j] <= tol,
                    "InverseKinematicsSolver mangled marker order.");
            }
        }
        cout << endl;
    }
}

void testNumberOfOrientationsMismatch()
{
    cout <<
        "\ntestInverseKinematicsSolver::testNumberOfOrientationsMismatch()"
        << endl;

    std::unique_ptr<Model> leg{ constructLegWithOrientationFrames() };
    const Coordinate& coord = leg->getCoordinateSet()[0];

    SimTK::State state = leg->initSystem();
    StatesTrajectory states;

    // sample time
    double dt = 0.1;
    int N = 11;
    for (int i = 0; i < N; ++i) {
        state.updTime() = i*dt;
        coord.setValue(state, i*dt*SimTK::Pi / 3);
        states.append(state);
    }

    double err = 0.1;
    SimTK::RowVector_<SimTK::Rotation> biases(3, SimTK::Rotation());
    // bias thigh_imu
    biases[0] *= SimTK::Rotation(err, SimTK::XAxis);
    cout << "biases: " << biases << endl;

    auto orientationsTable =
            generateOrientationsDataFromModelAndStates(*leg,
                states,
                biases,
                0.0,
                true);

    SimTK::Vector_<SimTK::Rotation> unusedCol(N,
        SimTK::Rotation(0.987654321, SimTK::ZAxis));

    auto usedOrientationNames = orientationsTable.getColumnLabels();

    // add an unused orientation sensor to the given orientation data
    orientationsTable.appendColumn("unused", unusedCol);

    cout << "Before:\n" << orientationsTable << endl;

    // re-order "observed" orientation data
    SimTK::Matrix_<SimTK::Rotation> dataGutsCopy
        = orientationsTable.getMatrix();
    int last = dataGutsCopy.ncol() - 1;
    // swap first and last columns 
    orientationsTable.updMatrix()(0) = dataGutsCopy(last);
    orientationsTable.updMatrix()(last) = dataGutsCopy(0);
    auto columnNames = orientationsTable.getColumnLabels();
    orientationsTable.setColumnLabel(0, columnNames[last]);
    orientationsTable.setColumnLabel(last, columnNames[0]);
    columnNames = orientationsTable.getColumnLabels();

    // Inject NaN in "observations" of thigh_imu orientation data
    for (int i = 4; i < 7; ++i) {
        orientationsTable.updMatrix()(i, 1).scalarMultiply(SimTK::NaN);
    }

    cout << "After reorder and NaN injections:\n" << orientationsTable << endl;

    std::shared_ptr<OrientationsReference> orientationsRef(
            new OrientationsReference(orientationsTable));
    int nmr = orientationsRef->getNumRefs();
    auto& osNames = orientationsRef->getNames();
    cout << osNames << endl;

    SimTK::Array_<CoordinateReference> coordRefs;
    // Reset the initial coordinate value
    coord.setValue(state, 0.0);
    InverseKinematicsSolver ikSolver(*leg, nullptr, orientationsRef, coordRefs);
    double tol = 1e-4;
    ikSolver.setAccuracy(tol);
    ikSolver.assemble(state);

    int nos = ikSolver.getNumOrientationSensorsInUse();

    SimTK::Array_<double> orientationErrors(nos);
    for (double t : orientationsRef->getTimes()) {
        state.updTime() = t;
        ikSolver.track(state);

        //get the  orientation errors
        ikSolver.computeCurrentOrientationErrors(orientationErrors);
        int nose = orientationErrors.size();

        SimTK_ASSERT_ALWAYS(nose == nos,
            "InverseKinematicsSolver failed to account "
            "for unused orientations reference (observation).");

        cout << "time: " << state.getTime() << " |";
        auto namesIter = usedOrientationNames.begin();
        for (int j = 0; j < nose; ++j) {
            const auto& orientationName = 
                ikSolver.getOrientationSensorNameForIndex(j);

            cout << " " << orientationName << " error = " << orientationErrors[j];

            SimTK_ASSERT_ALWAYS(*namesIter++ != "unused",
                "InverseKinematicsSolver failed to ignore "
                "unused orientation reference (observation).");

            if (orientationName == "thigh_imu") {//should see error on biased marker
                SimTK_ASSERT_ALWAYS(abs(orientationErrors[j]) <= err,
                    "InverseKinematicsSolver mangled marker order.");
            }
            else { // other markers should be minimally affected
                SimTK_ASSERT_ALWAYS(orientationErrors[j] <= tol,
                    "InverseKinematicsSolver mangled marker order.");
            }
        }
        cout << endl;
    }
}

Model* constructPendulumWithMarkers()
{
    Model* pendulum = new Model();
    pendulum->setName("pendulum");
    Body* ball =
        new Body("ball", 1.0, SimTK::Vec3(0), SimTK::Inertia::sphere(0.05));
    pendulum->addBody(ball);

    // PinJoint hinge is 1m above ground origin and 1m above the ball in the ball
    // reference frame such that the ball center is at the origin with the hinge
    // angle is zero
    PinJoint* hinge = new PinJoint("hinge", pendulum->getGround(),
        SimTK::Vec3(0, 1.0, 0), SimTK::Vec3(0),
        *ball, SimTK::Vec3(0, 1.0, 0), SimTK::Vec3(0));
    hinge->updCoordinate().setName("theta");
    pendulum->addJoint(hinge);

    // Add Markers
    Marker*m0 = new Marker();
    m0->setName("m0");
    m0->setParentFrame(*ball);
    m0->set_location(SimTK::Vec3(0));
    pendulum->addMarker(m0);
    
    // Shifted Right 1cm
    Marker*mR = new Marker();
    mR->setName("mR");
    mR->setParentFrame(*ball);
    mR->set_location(SimTK::Vec3(0.01, 0, 0));
    pendulum->addMarker(mR);
    
    // Shifted Left 2cm
    Marker*mL = new Marker();
    mL->setName("mL");
    mL->setParentFrame(*ball);
    mL->set_location(SimTK::Vec3(-0.02, 0, 0));
    pendulum->addMarker(mL);

    return pendulum;
}

TimeSeriesTable_<SimTK::Vec3>
generateMarkerDataFromModelAndStates(const Model& model,
                                     const StatesTrajectory& states,
                                     const SimTK::RowVector_<SimTK::Vec3>& biases,
                                     double noiseRadius,
                                     bool isConstantOffset) {
    // use a fixed seed so that we can reproduce and debug failures.
    std::mt19937 gen(0);
    std::normal_distribution<double> noise(0.0, 1);

    unique_ptr<Model> m{ model.clone() };
    
    auto* markerReporter = new TableReporterVec3();
    
    auto markers = m->updComponentList<Marker>();
    int cnt = 0;
    for (auto& marker : markers) {
        marker.set_location(marker.get_location() + biases[cnt++]);
        markerReporter->addToReport(
                marker.getOutput("location"), marker.getName());
    }

    m->addComponent(markerReporter);

    SimTK::State s = m->initSystem();

    for (const auto& state : states) {
        // collect results into reporter
        m->realizeReport(state);
    }

    // make a copy of the reported table
    auto results = markerReporter->getTable();

    SimTK::Vec3 offset = noiseRadius*SimTK::Vec3(double(noise(gen)),
                                                 double(noise(gen)),
                                                 double(noise(gen)));

    if (noiseRadius >= SimTK::Eps) {
        for (size_t i = 0; i < results.getNumRows(); ++i) {
            auto row = results.updRowAtIndex(i);
            for (int j = 0; j < row.size(); ++j) {
                if (!isConstantOffset) {
                    offset = noiseRadius*SimTK::Vec3(double(noise(gen)),
                                                     double(noise(gen)),
                                                     double(noise(gen)));
                }
                // add noise to each marker
                row[j] += offset;
            }
        }
    }

    return results;
}

Model* constructLegWithOrientationFrames()
{
    std::unique_ptr<Model> leg{ new Model() };
    leg->setName("leg");
    Body* thigh =
        new Body("thigh", 5.0, SimTK::Vec3(0),
                 SimTK::Inertia::cylinderAlongY(0.1, 0.5) );
    leg->addBody(thigh);
    Body* shank =
        new Body("shank", 2.0, SimTK::Vec3(0),
                 SimTK::Inertia::cylinderAlongY(0.04, 0.4) );
    leg->addBody(shank);
    Body* foot =
        new Body("foot", 1.0, SimTK::Vec3(0),
            SimTK::Inertia::cylinderAlongY(0.02, 0.1));
    leg->addBody(foot);


    // PinJoint hip is 1m above ground origin and 1m above the ball in the ball
    // reference frame such that the ball center is at the origin with the hinge
    // angle is zero
    PinJoint* hip = new PinJoint("hip", leg->getGround(),
        SimTK::Vec3(0, 1.0, 0), SimTK::Vec3(0),
        *thigh, SimTK::Vec3(0, 0.25, 0), SimTK::Vec3(0));
    hip->updCoordinate().setName("flex");
    leg->addJoint(hip);

    PinJoint* knee = new PinJoint("knee", *thigh,
        SimTK::Vec3(0, -0.25, 0), SimTK::Vec3(0),
        *shank, SimTK::Vec3(0, 0.2, 0), SimTK::Vec3(0));
    knee->updCoordinate().setName("flex");
    leg->addJoint(knee);

    PinJoint* ankle = new PinJoint("ankle", *shank,
        SimTK::Vec3(0, -0.2, 0), SimTK::Vec3(0),
        *foot, SimTK::Vec3(0, 0.1, 0), SimTK::Vec3(0));
    ankle->updCoordinate().setName("flex");
    leg->addJoint(ankle);

    // Add Orientation Sensor Frames
    SimTK::Transform offset(SimTK::Rotation(0.378, SimTK::YAxis) );
    thigh->addComponent(new PhysicalOffsetFrame("thigh_imu", *thigh, offset));
    shank->addComponent(new PhysicalOffsetFrame("shank_imu", *shank, offset));
    foot->addComponent(new PhysicalOffsetFrame("foot_imu", *foot, offset));

    return leg.release();
}

TimeSeriesTable_<SimTK::Rotation>
generateOrientationsDataFromModelAndStates(const Model& model,
    const StatesTrajectory& states,
    const SimTK::RowVector_<SimTK::Rotation>& biases,
    double noiseLevel, // noise standard deviation in radians
    bool constantOffset) {
    // use a fixed seed so that we can reproduce and debug failures.
    std::mt19937 gen(0);
    std::normal_distribution<double> noise(0.0, 1);

    unique_ptr<Model> m{ model.clone() };

    auto* orientationsReporter = new TableReporter_<SimTK::Rotation>();

    auto imus = m->updComponentList<PhysicalOffsetFrame>();
    int cnt = 0;
    for (auto& imu : imus) {
        if (imu.getName().find("_imu") != std::string::npos) {
            imu.setOffsetTransform(
                SimTK::Transform(imu.getOffsetTransform().R()*biases[cnt++]));
            orientationsReporter->addToReport(
                imu.getOutput("rotation"), imu.getName());
        }
    }

    m->addComponent(orientationsReporter);

    SimTK::State s = m->initSystem();

    for (const auto& state : states) {
        // collect results into reporter
        m->realizeReport(state);
    }

    // make a copy of the reported table
    auto results = orientationsReporter->getTable();

    SimTK::Rotation offset = SimTK::Rotation(
        SimTK::BodyRotationSequence,
        noiseLevel*double(noise(gen)), SimTK::XAxis,
        noiseLevel*double(noise(gen)), SimTK::YAxis,
        noiseLevel*double(noise(gen)), SimTK::ZAxis );

    cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;

    if (noiseLevel >= SimTK::Eps) {
        for (size_t i = 0; i < results.getNumRows(); ++i) {
            auto row = results.updRowAtIndex(i);
            for (int j = 0; j < row.size(); ++j) {
                if (!constantOffset) {
                    offset = SimTK::Rotation(SimTK::BodyRotationSequence,
                        noiseLevel*double(noise(gen)), SimTK::XAxis,
                        noiseLevel*double(noise(gen)), SimTK::YAxis,
                        noiseLevel*double(noise(gen)), SimTK::ZAxis);
                }
                // add noise to each orientation sensor
                row[j] *= offset;
            }
        }
    }

    return results;
}