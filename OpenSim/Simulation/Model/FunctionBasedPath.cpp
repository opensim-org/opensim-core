/* -------------------------------------------------------------------------- *
 *                      OpenSim:  FunctionBasedPath.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

#include "FunctionBasedPath.h"
#include "Model.h"
#include <future>

#include <SimTKcommon/internal/IteratorRange.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Common/Assertion.h>
#include <OpenSim/Common/LatinHypercubeDesign.h>
#include <OpenSim/Common/MultivariatePolynomialFunction.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR
//=============================================================================
FunctionBasedPath::FunctionBasedPath() : AbstractPath()
{
    setAuthors("Nicholas Bianco");
    constructProperties();
}

//=============================================================================
// GET AND SET METHODS
//=============================================================================
void FunctionBasedPath::appendCoordinatePath(const std::string& coordinatePath)
{
    append_coordinate_paths(coordinatePath);
}

void FunctionBasedPath::setCoordinatePaths(
        const std::vector<std::string>& coordinatePaths)
{
    updProperty_coordinate_paths().clear();
    for (const auto& coordinatePath : coordinatePaths) {
        appendCoordinatePath(coordinatePath);
    }
}

std::vector<std::string> FunctionBasedPath::getCoordinatePaths() const
{
    std::vector<std::string> coordinates;
    coordinates.reserve(getProperty_coordinate_paths().size());
    for (int i = 0; i < getProperty_coordinate_paths().size(); ++i) {
        coordinates.push_back(get_coordinate_paths(i));
    }

    return coordinates;
}

void FunctionBasedPath::setLengthFunction(const Function& lengthFunction)
{
    set_length_function(lengthFunction);
}

const Function& FunctionBasedPath::getLengthFunction() const
{
    return get_length_function();
}

void FunctionBasedPath::setLengtheningSpeedFunction(
        const Function& speedFunction)
{
    set_lengthening_speed_function(speedFunction);
}

const Function& FunctionBasedPath::getLengtheningSpeedFunction() const {
    return get_lengthening_speed_function();
}

void FunctionBasedPath::appendMomentArmFunction(
        const Function& momentArmFunction)
{
    append_moment_arm_functions(momentArmFunction);
}

void FunctionBasedPath::setMomentArmFunctions(
        const std::vector<Function>& momentArmFunctions)
{
    for (const auto& momentArmFunction : momentArmFunctions) {
        append_moment_arm_functions(momentArmFunction);
    }
}

const Function& FunctionBasedPath::getMomentArmFunction(
        const std::string& coordinatePath) const
{
    auto it = _coordinateIndices.find(coordinatePath);
    OPENSIM_THROW_IF_FRMOBJ(it == _coordinateIndices.end(), Exception,
            fmt::format("Path does not depend on Coordinate '{}'.",
                    coordinatePath))
    return get_moment_arm_functions(it->second);
}

const SimTK::Vector& FunctionBasedPath::getMomentArms(
        const SimTK::State& s) const {
    computeMomentArms(s);
    return getCacheVariableValue<SimTK::Vector>(s, _momentArmsCV);
}

//=============================================================================
// ABSTRACT PATH INTERFACE
//=============================================================================
double FunctionBasedPath::getLength(const SimTK::State& s) const
{
    computeLength(s);
    return getCacheVariableValue<double>(s, _lengthCV);
}

double FunctionBasedPath::computeMomentArm(const SimTK::State& s,
        const Coordinate& coord) const
{
    auto it = _coordinateIndices.find(coord.getAbsolutePathString());
    if (it != _coordinateIndices.end()) {
        computeMomentArms(s);
        return getCacheVariableValue<SimTK::Vector>(s, _momentArmsCV)
                .get(it->second);
    } else {
        return 0.0;
    }
}

double FunctionBasedPath::getLengtheningSpeed(const SimTK::State& s) const
{
    computeLengtheningSpeed(s);
    return getCacheVariableValue<double>(s, _lengtheningSpeedCV);
}

void FunctionBasedPath::addInEquivalentForces(const SimTK::State& state,
        const double& tension,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& mobilityForces) const
{
    // Get the moment arms.
    computeMomentArms(state);
    const auto& momentArms =
            getCacheVariableValue<SimTK::Vector>(state, _momentArmsCV);
    OPENSIM_ASSERT_ALWAYS(momentArms.size() == (int)_coordinates.size());

    // Apply the mobility forces.
    const SimTK::SimbodyMatterSubsystem& matter =
            getModel().getMatterSubsystem();
    for (int i = 0; i < (int)_coordinates.size(); ++i) {
        const SimTK::MobilizedBody& mobod =
                matter.getMobilizedBody(_coordinates[i]->getBodyIndex());
        mobod.applyOneMobilityForce(state,
                _coordinates[i]->getMobilizerQIndex(),
                momentArms[i] * tension,
                mobilityForces);
    }
}

//=============================================================================
// CONVENIENCE METHODS
//=============================================================================
void FunctionBasedPath::constructProperties()
{
    constructProperty_coordinate_paths();
    constructProperty_length_function();
    constructProperty_moment_arm_functions();
    constructProperty_lengthening_speed_function();
}

SimTK::Vector FunctionBasedPath::computeCoordinateValues(
        const SimTK::State& s) const
{
    SimTK::Vector coordinateValues((int)_coordinates.size(), 0.0);
    for (int i = 0; i < (int)_coordinates.size(); ++i) {
        coordinateValues[i] = _coordinates[i]->getValue(s);
    }

    return coordinateValues;
}

SimTK::Vector FunctionBasedPath::computeCoordinateDerivatives(
        const SimTK::State& s) const
{
    SimTK::Vector coordinateSpeeds((int)_coordinates.size(), 0.0);
    for (int i = 0; i < (int)_coordinates.size(); ++i) {
        coordinateSpeeds[i] = _coordinates[i]->getQDotValue(s);
    }

    return coordinateSpeeds;
}

void FunctionBasedPath::computeLength(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _lengthCV)) {
        return;
    }

    setCacheVariableValue(s, _lengthCV,
            getLengthFunction().calcValue(computeCoordinateValues(s)));
}

void FunctionBasedPath::computeMomentArms(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _momentArmsCV)) {
        return;
    }

    const auto& values = computeCoordinateValues(s);
    SimTK::Vector momentArms((int)_coordinates.size(), 0.0);
    if (_computeMomentArms) {
        // If we do not have moment arm functions, then compute the moment arms
        // based on the derivative of the length function with respect to each
        // coordinate.
        for (int i = 0; i < (int)_coordinates.size(); ++i) {
            // Negative sign to obey the OpenSim convention.
            momentArms[i] = -getLengthFunction().calcDerivative({i}, values);
        }
    } else {
        const auto& momentArmFunctions = getProperty_moment_arm_functions();
        for (int i = 0; i < momentArmFunctions.size(); ++i) {
            const auto& momentArmFunction = get_moment_arm_functions(i);
            momentArms[i] = momentArmFunction.calcValue(values);
        }
    }
    setCacheVariableValue(s, _momentArmsCV, momentArms);
}

void FunctionBasedPath::computeLengtheningSpeed(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _lengtheningSpeedCV)) {
        return;
    }

    if (_computeLengtheningSpeed) {
        // If we do not have a speed function, then compute the lengthening
        // speed based on the scalar product between the moment arms and the
        // coordinate derivatives.
        computeMomentArms(s);
        const auto& momentArms =
                getCacheVariableValue<SimTK::Vector>(s, _momentArmsCV);
        const SimTK::Vector& qdot = computeCoordinateDerivatives(s);
        // Negate the moment arms to cancel out the negative sign from the
        // OpenSim convention.
        const double lengtheningSpeed = ~qdot * momentArms.negate();
        setCacheVariableValue(s, _lengtheningSpeedCV, lengtheningSpeed);
    } else {
        SimTK::Vector coordinatesState(2*(int)_coordinates.size(), 0.0);
        for (int i = 0; i < (int)_coordinates.size(); ++i) {
            coordinatesState[i] = _coordinates[i]->getValue(s);
            coordinatesState[i + (int)_coordinates.size()] =
                    _coordinates[i]->getSpeedValue(s);
        }
        setCacheVariableValue(s, _lengtheningSpeedCV,
                getLengtheningSpeedFunction().calcValue(coordinatesState));
    }
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void FunctionBasedPath::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    // Check the properties.
    OPENSIM_THROW_IF_FRMOBJ(getProperty_length_function().empty(),
            Exception, "Expected 'length_function' to be provided, but the "
                       "property was empty.")

    OPENSIM_THROW_IF_FRMOBJ(getProperty_coordinate_paths().empty(), Exception,
            "This path should be dependent on at least one coordinate, but"
            "no coordinates were provided.")

    // Check that the `coordinate_paths` are unique.
    std::vector<std::string> uniqueCoordinatePaths;
    uniqueCoordinatePaths.reserve(getProperty_coordinate_paths().size());
    for (int i = 0; i < getProperty_coordinate_paths().size(); ++i) {
        const auto& coordinatePath = get_coordinate_paths(i);
        if (std::find(uniqueCoordinatePaths.begin(),
                      uniqueCoordinatePaths.end(),
                    coordinatePath) != uniqueCoordinatePaths.end()) {
            OPENSIM_THROW_FRMOBJ(Exception,
                    fmt::format("Coordinate '{}' was provided more than once.",
                            coordinatePath))
        }
        uniqueCoordinatePaths.push_back(coordinatePath);
    }

    OPENSIM_THROW_IF_FRMOBJ(getLengthFunction().getArgumentSize() !=
            getProperty_coordinate_paths().size(), Exception,
            fmt::format("Expected the number of arguments in 'length_function' "
                        "({}) to equal the number of coordinates ({}).",
                        getLengthFunction().getArgumentSize(),
                        getProperty_coordinate_paths().size()))

    if (getProperty_moment_arm_functions().empty()) {
        _computeMomentArms = true;
        OPENSIM_THROW_IF_FRMOBJ(getLengthFunction().getMaxDerivativeOrder() < 1,
                Exception, "Since moment arm functions were not provided, "
                           "expected the length function to be at least "
                           "first-order differentiable with respect to "
                           "the coordinate values, but it is not.")
    } else {
        OPENSIM_THROW_IF_FRMOBJ(getProperty_moment_arm_functions().size() !=
                                getProperty_coordinate_paths().size(), Exception,
                fmt::format("Expected the number of moment arm functions ({}) "
                            "to equal the number of coordinates ({}).",
                        getProperty_moment_arm_functions().size(),
                        getProperty_coordinate_paths().size()))

        for (int i = 0; i < getProperty_moment_arm_functions().size(); ++i) {
            OPENSIM_THROW_IF_FRMOBJ(
                    get_moment_arm_functions(i).getArgumentSize() !=
                            getProperty_coordinate_paths().size(), Exception,
                    fmt::format("Expected the number of arguments in "
                                "'moment_arm_functions[{}]' ({}) to equal the "
                                "number of coordinates ({}).",
                            i, get_moment_arm_functions(i).getArgumentSize(),
                            getProperty_coordinate_paths().size()))
        }
    }

    if (getProperty_lengthening_speed_function().empty()) {
        _computeLengtheningSpeed = true;
    } else {
        OPENSIM_THROW_IF_FRMOBJ(
                getLengtheningSpeedFunction().getArgumentSize() !=
                2*getProperty_coordinate_paths().size(), Exception,
                fmt::format("Expected the number of arguments in "
                            "'speed_function' ({}) to equal to twice the "
                            "number of coordinates ({}).",
                        getLengtheningSpeedFunction().getArgumentSize(),
                        getProperty_coordinate_paths().size()))
    }

    // Populate the coordinate index map. In case "finalizeFromProperties()" is
    // called multiple times, clear the map first.
    _coordinateIndices.clear();
    for (int i = 0; i < getProperty_coordinate_paths().size(); ++i) {
        _coordinateIndices[get_coordinate_paths(i)] = i;
    }
}
void FunctionBasedPath::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // Check that the coordinates are in the model. If so, grab references
    // pointers to them. In case "connectToModel()" is called multiple times,
    // clear any existing references first.
    _coordinates.clear();
    for (int i = 0; i < getProperty_coordinate_paths().size(); ++i) {
        const auto& coordinatePath = get_coordinate_paths(i);
        OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<Coordinate>(coordinatePath),
                Exception,
                fmt::format("Coordinate '{}' does not exist in the model.",
                        coordinatePath))

        _coordinates.emplace_back(
                &model.getComponent<const Coordinate>(coordinatePath));
    }
}
void FunctionBasedPath::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    _lengthCV = addCacheVariable<double>("length", 0.0, SimTK::Stage::Position);
    _momentArmsCV = addCacheVariable<SimTK::Vector>("moment_arms",
            SimTK::Vector(getProperty_coordinate_paths().size(), 0.0),
            SimTK::Stage::Position);
    _lengtheningSpeedCV = addCacheVariable<double>("lengthening_speed", 0.0,
            SimTK::Stage::Velocity);
}

//=============================================================================
// FUNCTION-BASED PATH FITTER BOUNDS
//=============================================================================
FunctionBasedPathFitterBounds::FunctionBasedPathFitterBounds() : Object()
{
    setAuthors("Nicholas Bianco");
    constructProperties();
}

FunctionBasedPathFitterBounds::FunctionBasedPathFitterBounds(
        const std::string& coordinatePath, const SimTK::Vec2& bounds) :
        FunctionBasedPathFitterBounds() {
    set_coordinate_path(coordinatePath);
    OPENSIM_THROW_IF_FRMOBJ(bounds[0] >= bounds[1], Exception,
            "Expected the lower bound to be less than the upper bound, but "
            "this is not the case.")
    set_bounds(bounds);
}

void FunctionBasedPathFitterBounds::constructProperties() {
    constructProperty_coordinate_path("");
    constructProperty_bounds({-5.0, 5.0});
}

//=============================================================================
// FUNCTION-BASED PATH FITTER
//=============================================================================

FunctionBasedPathFitter::FunctionBasedPathFitter() : Object()
{
    setAuthors("Nicholas Bianco");
    constructProperties();
}

FunctionBasedPathFitter::~FunctionBasedPathFitter() noexcept = default;

FunctionBasedPathFitter::FunctionBasedPathFitter(
        FunctionBasedPathFitter const&) = default;

FunctionBasedPathFitter& FunctionBasedPathFitter::operator=(
        const FunctionBasedPathFitter&) = default;

FunctionBasedPathFitter::FunctionBasedPathFitter(
        FunctionBasedPathFitter&& other) = default;

FunctionBasedPathFitter& FunctionBasedPathFitter::operator=(
        FunctionBasedPathFitter&& other) = default;

void FunctionBasedPathFitter::constructProperties() {
    constructProperty_model(ModelProcessor());
    constructProperty_coordinate_values(TableProcessor());
    constructProperty_moment_arm_tolerance(1e-3);
    constructProperty_minimum_polynomial_order(2);
    constructProperty_maximum_polynomial_order(9);
    constructProperty_parallel((int)std::thread::hardware_concurrency() - 2);
    constructProperty_default_coordinate_sampling_bounds({-5.0, 5.0});
    constructProperty_coordinate_sampling_bounds();
    constructProperty_num_samples_per_frame(25);
}

std::string FunctionBasedPathFitter::getDocumentDirectory() const {
    std::string setupDir;
    {
        bool dontApplySearchPath;
        std::string fileName, extension;
        SimTK::Pathname::deconstructPathname(getDocumentFileName(),
                dontApplySearchPath, setupDir, fileName, extension);
    }
    return setupDir;
}

void FunctionBasedPathFitter::runFittingPipeline() {

    // Process the inputs.
    // -------------------
    // Load the model.
    Model model = get_model().process(getDocumentDirectory());
    model.initSystem();

    // Load the coordinate values.
    TableProcessor tableProcessor = get_coordinate_values();
    tableProcessor.append(TabOpUseAbsoluteStateNames());
    tableProcessor.append(TabOpAppendCoupledCoordinateValues());
    TimeSeriesTable values = tableProcessor.processAndConvertToRadians(
            getDocumentDirectory(), model);

    // Validate the coordinate values table.
    std::vector<std::string> jointsToWeld;
    for (auto& coordinate : model.updComponentList<Coordinate>()) {
        std::string valuePath = fmt::format("{}/value",
                coordinate.getAbsolutePathString());

        // If the coordinate is locked, but the user provided a column for the
        // coordinate, then we unlock it. Otherwise, we will weld the joint that
        // the coordinate belongs to.
        if (coordinate.get_locked()) {
            if (values.hasColumn(valuePath)) {
                coordinate.set_locked(false);
            } else {
                jointsToWeld.push_back(coordinate.getJoint().getName());
            }
        } else {
            OPENSIM_THROW_IF_FRMOBJ(!values.hasColumn(valuePath), Exception,
                    fmt::format("Expected the coordinate values table to "
                                "contain a column for '{}' (this coordinate is "
                                "not locked), but it does not.",
                            coordinate.getAbsolutePathString()))
        }
    }

    // If we detected any joints to be welded, update the model.
    if (!jointsToWeld.empty()) {
        auto modelProcessor = ModelProcessor(model) |
                              ModOpReplaceJointsWithWelds(jointsToWeld);
        model = modelProcessor.process();
    }

    // Check coordinate sampling bounds.
    // ---------------------------------
    m_coordinateBoundsMap.reserve(
            getProperty_coordinate_sampling_bounds().size());
    for (int i = 0; i < getProperty_coordinate_sampling_bounds().size(); ++i) {
        const auto& coordinatePath =
                get_coordinate_sampling_bounds(i).get_coordinate_path();
        OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<Coordinate>(coordinatePath),
                Exception, "Expected the model to contain the coordinate '{}', "
                           "but it does not.", coordinatePath)
        const auto& coordinate = model.getComponent<Coordinate>(coordinatePath);
        SimTK::Vec2 bounds = get_coordinate_sampling_bounds(i).get_bounds();
        if (coordinate.get_clamped()) {
            if (bounds[0] < coordinate.getRangeMin()) {
                bounds[0] = coordinate.getRangeMin();
                log_warn("FunctionBasedPathFitter: The lower bound for "
                         "coordinate '{}' was clamped to the coordinate's "
                         "minimum value ({}).", coordinatePath, bounds[0]);
            }
            if (bounds[1] > coordinate.getRangeMax()) {
                bounds[1] = coordinate.getRangeMax();
                log_warn("FunctionBasedPathFitter: The upper bound for "
                         "coordinate '{}' was clamped to the coordinate's "
                         "maximum value ({}).", coordinatePath, bounds[1]);
            }
        }
        m_coordinateBoundsMap.insert({coordinatePath, bounds});
    }

    // Validate settings.
    // ------------------
    OPENSIM_THROW_IF_FRMOBJ(get_moment_arm_tolerance() < 0 ||
            get_moment_arm_tolerance() > 1, Exception,
            "Expected 'moment_arm_tolerance' to be in the range [0, 1], but "
            "received {}.", get_moment_arm_tolerance())

    OPENSIM_THROW_IF_FRMOBJ(get_minimum_polynomial_order() < 1, Exception,
            "Expected 'minimum_polynomial_order' to be at least 1, but "
            "received {}.", get_minimum_polynomial_order())

    OPENSIM_THROW_IF_FRMOBJ(get_maximum_polynomial_order() > 9, Exception,
            "Expected 'maximum_polynomial_order' to be at most 9, but "
            "received {}.", get_maximum_polynomial_order())

    OPENSIM_THROW_IF_FRMOBJ(get_minimum_polynomial_order() >
            get_maximum_polynomial_order(), Exception,
            "Expected 'minimum_polynomial_order' to be less than or equal to "
            "'maximum_polynomial_order', but received {} and {}, "
            "respectively.", get_minimum_polynomial_order(),
            get_maximum_polynomial_order())

    OPENSIM_THROW_IF_FRMOBJ(get_parallel() < 1 ||
            get_parallel() > (int)std::thread::hardware_concurrency(), Exception,
            "Expected 'threads' to be between 1 and {}, but received {}.",
            std::thread::hardware_concurrency(), get_parallel())

    // Sample coordinate values around the provided trajectory.
    // --------------------------------------------------------
    sampleAndAppendCoordinateValues(values);

    // Recompute the coupled coordinate values.
    auto tableProcessorSampled = TableProcessor(values) |
                                 TabOpAppendCoupledCoordinateValues();
    values = tableProcessor.process(&model);

    // Compute path lengths and moment arms.
    // -------------------------------------
    // TODO

    // Fit the FunctionBasedPaths.
    // ---------------------------
    // TODO

    // Evaluate the fit.
    // -----------------
    // TODO

    // TODO save/print new model or throw exception?
}

void FunctionBasedPathFitter::sampleAndAppendCoordinateValues(
        TimeSeriesTable& values) {
    LatinHypercubeDesign lhs;
    lhs.setNumSamples(get_num_samples_per_frame());
    lhs.setNumVariables((int)values.getNumColumns());


    auto sampleCoordinateValuesSubset = [lhs, values](
            std::vector<int>::iterator begin_iter,
            std::vector<int>::iterator end_iter) -> SimTK::Matrix {

        SimTK::Matrix results(std::distance(begin_iter, end_iter),
                values.getNumColumns());
        for (auto it = begin_iter; it != end_iter; ++it) {

        }


    };



    std::vector<int> timeIndexes(values.getNumRows());
    std::iota(timeIndexes.begin(), timeIndexes.end(), 0);





    // Divide the sampling across multiple threads.
    int stride = static_cast<int>(
            std::floor(values.getNumRows() / get_parallel()));
    std::vector<std::future<SimTK::Matrix>> futures;
    int offset = 0;
    for (int ithread = 0; ithread < get_parallel(); ++ithread) {
        auto begin_iter = timeIndexes.begin() + offset;
        auto end_iter = (ithread == get_parallel()-1) ?
                timeIndexes.end() :
                timeIndexes.begin() + offset + stride;
        futures.push_back(std::async(std::launch::async,
                sampleCoordinateValuesSubset,
                begin_iter, end_iter));
        offset += stride;
    }



}

void FunctionBasedPathFitter::computePathLengthsAndMomentArms(
        Model model,
        const TimeSeriesTable& coordinateValues,
        TimeSeriesTable& pathLengths,
        TimeSeriesTable& momentArms,
        std::map<std::string, std::vector<std::string>>& momentArmMap) {

    // Check inputs.
    OPENSIM_THROW_IF(
            get_parallel() < 1 ||
            get_parallel() > (int)std::thread::hardware_concurrency(),
            Exception, "Number of threads must be between 1 and {}.",
            std::thread::hardware_concurrency());
    OPENSIM_THROW_IF(pathLengths.getNumRows() != 0, Exception,
            "Expected 'pathLengths' to be empty.");
    OPENSIM_THROW_IF(momentArms.getNumRows() != 0, Exception,
            "Expected 'momentArms' to be empty.");
    momentArmMap.clear();

    // Load model.
    SimTK::State state = model.initSystem();

    // Load coordinate values.
    // TODO check coordinate values
    // TODO delete coordinate speeds
    //    TableProcessor tableProcessor = TableProcessor(coordinateValues) |
    //                                    TabOpUseAbsoluteStateNames() |
    //                                    TabOpAppendCoupledCoordinateValues();
    //    TimeSeriesTable coordinateValuesProcessed =
    //            tableProcessor.processAndConvertToRadians(model);

    Array<std::string> stateVariableNames = model.getStateVariableNames();
    for (const auto& label : coordinateValues.getColumnLabels()) {
        OPENSIM_THROW_IF(stateVariableNames.findIndex(label) == -1, Exception,
                "Expected the model to contain the coordinate value state "
                "'{}', but it does not.", label);
    }
    auto statesTrajectory = StatesTrajectory::createFromStatesTable(
            model, coordinateValues, true, false, false);

    // Determine the maximum number of path and moment arm evaluations.
    const auto& paths = model.getComponentList<AbstractPath>();
    int numPaths = (int)std::distance(paths.begin(), paths.end());
    int numCoordinates = (int)coordinateValues.getNumColumns();
    int numColumns = numPaths + (numPaths * numCoordinates);

    // Define helper function for path length and moment arm computations.
    auto calcPathLengthsAndMomentArmsSubset = [numColumns, numPaths](
                      Model model, StatesTrajectory::IteratorRange subsetStates)
            -> SimTK::Matrix {
        model.initSystem();

        // Create a matrix to store the results
        SimTK::Matrix results(
                (int)std::distance(subsetStates.begin(), subsetStates.end()),
                numColumns);

        int row = 0;
        const auto& forces = model.getComponentList<Force>();
        for (const auto& state : subsetStates) {
            model.realizePosition(state);

            int ip = 0;
            int ima = 0;
            for (const auto& force : forces) {
                if (force.hasProperty("path")) {
                    const AbstractPath& path =
                            force.getProperty<AbstractPath>("path").getValue();

                    // Compute path length.
                    results(row, ip++) = path.getLength(state);

                    // Compute moment arms.
                    for (const auto& coordinate :
                            model.getComponentList<Coordinate>()) {
                        results(row, numPaths + ima++) =
                                path.computeMomentArm(state, coordinate);
                    }
                }
            }
            row++;
        }

        return results;
    };

    // Divide the path length and moment arm computations across multiple
    // threads.
    int stride = static_cast<int>(
            std::floor(coordinateValues.getNumRows() / get_parallel()));
    std::vector<std::future<SimTK::Matrix>> futures;
    int offset = 0;
    for (int ithread = 0; ithread < get_parallel(); ++ithread) {
        auto begin_iter = statesTrajectory.begin() + offset;
        auto end_iter = (ithread == get_parallel()-1) ?
                statesTrajectory.end() :
                statesTrajectory.begin() + offset + stride;
        futures.push_back(std::async(std::launch::async,
                calcPathLengthsAndMomentArmsSubset,
                model,
                makeIteratorRange(begin_iter, end_iter)));
        offset += stride;
    }

    // Wait for threads to finish and collect results
    std::vector<SimTK::Matrix> outputs(get_parallel());
    for (int i = 0; i < get_parallel(); ++i) {
        outputs[i] = futures[i].get();
    }

    // Assemble results into one TimeSeriesTable
    std::vector<double> time = coordinateValues.getIndependentColumn();
    int itime = 0;
    for (int i = 0; i < get_parallel(); ++i) {
        for (int j = 0; j < outputs[i].nrow(); ++j) {
            pathLengths.appendRow(time[itime],
                    outputs[i].block(j, 0, 1, numPaths).getAsRowVector());
            momentArms.appendRow(time[itime], outputs[i].block(j, numPaths, 1,
                    numPaths * numCoordinates).getAsRowVector());
            itime++;
        }
    }

    int ip = 0;
    int ima = 0;
    std::vector<std::string> pathLengthLabels(numPaths);
    std::vector<std::string> momentArmLabels(numPaths * numCoordinates);
    const auto& forces = model.getComponentList<Force>();
    for (const auto& force : forces) {
        if (force.hasProperty("path")) {
            pathLengthLabels[ip++] =
                    fmt::format("{}_length", force.getAbsolutePathString());
            for (const auto& coordinate :
                    model.getComponentList<Coordinate>()) {
                momentArmLabels[ima++] = fmt::format("{}_moment_arm_{}",
                        force.getAbsolutePathString(), coordinate.getName());
            }
        }
    }

    std::cout << "momentArmLabels size: " << momentArmLabels.size() << std::endl;
    pathLengths.setColumnLabels(pathLengthLabels);
    momentArms.setColumnLabels(momentArmLabels);

    // Remove columns for coupled coordinates.
    for (const auto& couplerConstraint :
            model.getComponentList<CoordinateCouplerConstraint>()) {
        auto momentArmLabel = fmt::format("_moment_arm_{}",
                couplerConstraint.getDependentCoordinateName());
        for (const auto& label : momentArms.getColumnLabels()) {
            if (label.find(momentArmLabel) != std::string::npos) {
                momentArms.removeColumn(label);
            }
        }
    }

    // Remove moment arm columns that contain values below the specified
    // moment arm tolerance.
    for (const auto& label : momentArms.getColumnLabels()) {
        if (label.find("_moment_arm_") != std::string::npos) {
            const auto& col = momentArms.getDependentColumn(label);
            if (col.normInf() < get_moment_arm_tolerance()) {
                momentArms.removeColumn(label);
            } else {
                std::string path = label.substr(0, label.find("_moment_arm_"));
                std::string coordinate = label.substr(
                        label.find("_moment_arm_") + 12);
                momentArmMap[path].push_back(coordinate);
            }
        }
    }
}

double FunctionBasedPathFitter::fitFunctionBasedPathCoefficients(
        Model model,
        const TimeSeriesTable& coordinateValues,
        const TimeSeriesTable& pathLengths,
        const TimeSeriesTable& momentArms,
        const std::map<std::string, std::vector<std::string>>& momentArmMap,
        std::string outputPath,
        const int minOrder, const int maxOrder) {

    // Factorial helper function.
    auto factorial = [](int n) {
        int result = 1;
        for (int i = 1; i <= n; ++i) {
            result *= i;
        }
        return result;
    };

    // n-choose-k helper function.
    auto nchoosek = [factorial](int n, int k) {
        return factorial(n) / (factorial(k) * factorial(n - k));
    };

    // Initialize model.
    // -----------------
    model.initSystem();

    // Coordinate references.
    // ----------------------
    const CoordinateSet& coordinateSet = model.getCoordinateSet();
    const int numCoordinates = coordinateSet.getSize();
    const int numTimes = (int)coordinateValues.getNumRows();

    // Build a FunctionBasedPath for each path-based force in the model.
    // -----------------------------------------------------------------
    // Solving A*x = b, where x is the vector of coefficients for the
    // FunctionBasedPath, A is a matrix of polynomial terms, and b is a vector
    // of path lengths and moment arms.
    Set<FunctionBasedPath> functionBasedPaths;
    const auto forces = model.getComponentList<Force>();
    const int numForces = (int)std::distance(forces.begin(), forces.end());
    SimTK::Vector bestRootMeanSquareErrors(numForces, SimTK::Infinity);
    int iforce = 0;
    for (const auto& force : model.getComponentList<Force>())  {

        // Check if the current force is dependent on any coordinates in the
        // model. If not, skip it.
        if (momentArmMap.find(force.getAbsolutePathString()) ==
                momentArmMap.end()) {
            bestRootMeanSquareErrors[iforce] = 0.0;
            ++iforce;
            continue;
        }

        // The current force path and the number of coordinates it depends on.
        const std::string& forcePath = force.getAbsolutePathString();
        std::vector<std::string> coordinatesNamesThisForce =
                momentArmMap.at(forcePath);
        int numCoordinatesThisForce = (int)coordinatesNamesThisForce.size();
        std::vector<std::string> coordinatePathsThisForce;
        for (const auto& coordinateName : coordinatesNamesThisForce) {
            coordinatePathsThisForce.push_back(
                    coordinateSet.get(coordinateName).getAbsolutePathString());
        }

        // Initialize the 'b' vector. This is the same for all polynomial
        // orders.
        SimTK::Vector b(numTimes * (numCoordinatesThisForce + 1), 0.0);

        // The path lengths for this force. This is the first N elements of the
        // 'b' vector.
        b(0, numTimes) = pathLengths.getDependentColumn(
                fmt::format("{}_length", forcePath));

        // The moment arms and coordinate values for this force.
        SimTK::Matrix coordinatesThisForce(numTimes, numCoordinatesThisForce,
                0.0);
        for (int ic = 0; ic < numCoordinatesThisForce; ++ic) {
            const std::string& coordinateName = coordinatesNamesThisForce[ic];
            b((ic+1)*numTimes, numTimes) = momentArms.getDependentColumn(
                    fmt::format("{}_moment_arm_{}", forcePath, coordinateName));

            const SimTK::VectorView coordinateValuesThisCoordinate =
                    coordinateValues.getDependentColumn(fmt::format("{}/value",
                            coordinatePathsThisForce[ic]));
            for (int itime = 0; itime < numTimes; ++itime) {
                coordinatesThisForce.set(
                        itime, ic, coordinateValuesThisCoordinate[itime]);
            }
        }

        // Polynomial fitting.
        // -------------------
        double bestRootMeanSquareError = SimTK::Infinity;
        SimTK::Vector bestCoefficients;
        int bestOrder = minOrder;
        for (int order = minOrder; order <= maxOrder; ++order) {

            // Initialize the multivariate polynomial function.
            int numCoefficients =
                    nchoosek(numCoordinatesThisForce + order, order);
            SimTK::Vector dummyCoefficients(numCoefficients, 0.0);
            MultivariatePolynomialFunction dummyFunction(dummyCoefficients,
                    numCoordinatesThisForce, order);

            // Initialize the 'A' matrix.
            SimTK::Matrix A(numTimes * (numCoordinatesThisForce + 1),
                    numCoefficients, 0.0);

            // Fill in the A matrix. This contains the polynomial terms for the
            // path length and moment arms.
            for (int itime = 0; itime < numTimes; ++itime) {
                A(itime, 0, 1, numCoefficients) = dummyFunction.getTermValues(
                        coordinatesThisForce.row(itime).getAsVector());

                for (int ic = 0; ic < numCoordinatesThisForce; ++ic) {
                    SimTK::Vector termDerivatives =
                            dummyFunction.getTermDerivatives({ic},
                         coordinatesThisForce.row(itime).getAsVector()).negate();
                    A((ic+1)*numTimes + itime, 0, 1, numCoefficients) =
                            termDerivatives;
                }
            }

            // Solve the least-squares problem. A is a rectangular matrix with
            // full column rank, so we can use the left pseudo-inverse to solve
            // for the coefficients.
            SimTK::Matrix pinv_A = (~A * A).invert() * ~A;
            SimTK::Vector coefficients = pinv_A * b;

            // Calculate the RMS error.
            SimTK::Vector b_fit = A * coefficients;
            SimTK::Vector error = b - b_fit;
            const double rmsError = std::sqrt((error.normSqr() / error.size()));

            // Save best solution.
            if (rmsError < bestRootMeanSquareErrors[iforce]) {
                bestRootMeanSquareErrors[iforce] = rmsError;
                bestCoefficients = coefficients;
                bestOrder = order;
            }

            ++order;
        }

        // Create a FunctionBasedPath for the current path-based force.
        MultivariatePolynomialFunction lengthFunction;
        lengthFunction.setDimension(numCoordinatesThisForce);
        lengthFunction.setOrder(bestOrder);
        lengthFunction.setCoefficients(bestCoefficients);

        auto functionBasedPath = std::make_unique<FunctionBasedPath>();
        functionBasedPath->setName(forcePath);
        functionBasedPath->setCoordinatePaths(coordinatePathsThisForce);
        functionBasedPath->setLengthFunction(lengthFunction);
        functionBasedPaths.adoptAndAppend(functionBasedPath.release());

        ++iforce;
    }

    // Save the function-based paths to a file.
    // ----------------------------------------
    functionBasedPaths.print(outputPath);

    // Return the average RMS error.
    // -----------------------------
    double averageRootMeanSquareError = bestRootMeanSquareErrors.sum() /
                                        bestRootMeanSquareErrors.size();

    return averageRootMeanSquareError;
}



