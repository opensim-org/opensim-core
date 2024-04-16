/* -------------------------------------------------------------------------- *
 *                    OpenSim:  PolynomialPathFitter.cpp                      *
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

#include "PolynomialPathFitter.h"

#include <future>
#include <OpenSim/Actuators/ModelOperators.h>

#include <OpenSim/Common/LatinHypercubeDesign.h>
#include <OpenSim/Common/MultivariatePolynomialFunction.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>

using namespace OpenSim;

//=============================================================================
// FUNCTION-BASED PATH FITTER BOUNDS
//=============================================================================
PolynomialPathFitterBounds::PolynomialPathFitterBounds() : Object()
{
    setAuthors("Nicholas Bianco");
    constructProperties();
}

PolynomialPathFitterBounds::PolynomialPathFitterBounds(
        const std::string& coordinatePath, const SimTK::Vec2& bounds) :
        PolynomialPathFitterBounds() {
    set_coordinate_path(coordinatePath);
    OPENSIM_THROW_IF_FRMOBJ(bounds[0] >= bounds[1], Exception,
            "Expected the lower bound to be less than the upper bound, but "
            "it is not.")
    set_bounds(bounds);
}

void PolynomialPathFitterBounds::constructProperties() {
    constructProperty_coordinate_path("");
    constructProperty_bounds({-10.0, 10.0});
}

//=============================================================================
// FUNCTION-BASED PATH FITTER
//=============================================================================

PolynomialPathFitter::PolynomialPathFitter() : Object()
{
    setAuthors("Nicholas Bianco");
    constructProperties();
}

PolynomialPathFitter::~PolynomialPathFitter() noexcept = default;

PolynomialPathFitter::PolynomialPathFitter(
        PolynomialPathFitter const&) = default;

PolynomialPathFitter& PolynomialPathFitter::operator=(
        const PolynomialPathFitter&) = default;

PolynomialPathFitter::PolynomialPathFitter(
        PolynomialPathFitter&& other) = default;

PolynomialPathFitter& PolynomialPathFitter::operator=(
        PolynomialPathFitter&& other) = default;

//=============================================================================
// PATH FITTING PIPELINE
//=============================================================================

void PolynomialPathFitter::run() {

    log_info("");
    log_info("====================");
    log_info("PolynomialPathFitter");
    log_info("====================");
    log_info("");

    // Process the inputs.
    // -------------------
    log_info("Step 1/9: Load the model and coordinate values table.");
    log_info("-----------------------------------------------------");

    // Load the model.
    Model model = get_model().process(getDocumentDirectory());
    model.initSystem();

    // Load the coordinate values table.
    TimeSeriesTable values = loadCoordinateValuesAndValidateModel(
            getDocumentDirectory(), get_coordinate_values(), model);

    // Coordinate sampling bounds.
    // ---------------------------
    log_info("");
    log_info("Step 2/9: Set the coordinate bounds.");
    log_info("------------------------------------");
    // Set the global bounds for all coordinates.
    SimTK::Vec2 globalBounds = get_global_coordinate_sampling_bounds();
    log_info("Global bounds: [{}, {}] degrees.",
             globalBounds[0], globalBounds[1]);

    globalBounds[0] = SimTK::convertDegreesToRadians(globalBounds[0]);
    globalBounds[1] = SimTK::convertDegreesToRadians(globalBounds[1]);
    m_coordinateBoundsMap.reserve(model.getNumCoordinates());
    m_coordinateRangeMap.reserve(model.getNumCoordinates());
    for (const auto& coordinate : model.getComponentList<Coordinate>()) {
        std::string valuePath = fmt::format("{}/value",
                coordinate.getAbsolutePathString());
        m_coordinateBoundsMap.insert({valuePath, globalBounds});

        // If the coordinate is clamped, then we also store the allowable range
        // of motion.
        if (coordinate.get_clamped()) {
            SimTK::Vec2 range = {coordinate.getRangeMin(),
                    coordinate.getRangeMax()};
            m_coordinateRangeMap.insert({valuePath, range});
        } else {
            m_coordinateRangeMap.insert({valuePath,
                    {-SimTK::Infinity, SimTK::Infinity}});
        }
    }

    // Set any coordinate-specific bounds. This will overwrite the default
    // bounds.
    for (int i = 0; i < getProperty_coordinate_sampling_bounds().size(); ++i) {
        const auto& coordinatePath =
                get_coordinate_sampling_bounds(i).get_coordinate_path();
        OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<Coordinate>(coordinatePath),
                Exception, "Expected the model to contain the coordinate '{}', "
                           "but it does not.", coordinatePath)
        SimTK::Vec2 bounds = get_coordinate_sampling_bounds(i).get_bounds();
        log_info("Bounds for coordinate '{}': [{}, {}] degrees.",
                coordinatePath, bounds[0], bounds[1]);

        bounds[0] = SimTK::convertDegreesToRadians(bounds[0]);
        bounds[1] = SimTK::convertDegreesToRadians(bounds[1]);
        std::string valuePath = fmt::format("{}/value", coordinatePath);
        m_coordinateBoundsMap[valuePath] = bounds;
    }

    // Validate settings.
    // ------------------
    log_info("");
    log_info("Step 3/9: Verify the user-defined settings.");
    log_info("-------------------------------------------");
    // Parallelization.
    OPENSIM_THROW_IF_FRMOBJ(get_num_parallel_threads() < 1 ||
            get_num_parallel_threads() >
                (int)std::thread::hardware_concurrency(), Exception,
            "Expected 'num_parallel_threads' to be between 1 and {}, but "
            "received {}.", std::thread::hardware_concurrency(),
            get_num_parallel_threads())
    log_info("Number of parallel threads = {}", get_num_parallel_threads());

    // Number of samples per frame.
    OPENSIM_THROW_IF_FRMOBJ(get_num_samples_per_frame() < 1, Exception,
            "Expected 'num_samples_per_frame' to be a non-zero integer value, "
            "but received {}.", get_num_samples_per_frame());
    log_info("Number of samples per frame = {}", get_num_samples_per_frame());

    // Latin hypercube algorithm.
    checkPropertyValueIsInSet(getProperty_latin_hypercube_algorithm(),
            {"random", "ESEA"});
    m_useStochasticEvolutionaryLHS =
            (get_latin_hypercube_algorithm() == "ESEA");
    log_info("Latin hypercube algorithm = '{}'",
            get_latin_hypercube_algorithm());

    // Moment arm threshold.
    OPENSIM_THROW_IF_FRMOBJ(get_moment_arm_threshold() < 0 ||
                            get_moment_arm_threshold() > 1, Exception,
            "Expected 'moment_arm_threshold' to be in the range [0, 1], but "
            "received {:2g}.", get_moment_arm_threshold())
    log_info("Moment arm threshold = {:1.1e} meters",
            get_moment_arm_threshold(), 1);

    // Polynomial order.
    checkPropertyValueIsPositive(getProperty_minimum_polynomial_order());
    checkPropertyValueIsPositive(getProperty_maximum_polynomial_order());
    OPENSIM_THROW_IF_FRMOBJ(get_maximum_polynomial_order() > 9, Exception,
            "Expected 'maximum_polynomial_order' to be at most 9, but "
            "received {}.", get_maximum_polynomial_order())
    OPENSIM_THROW_IF_FRMOBJ(get_minimum_polynomial_order() >
                            get_maximum_polynomial_order(), Exception,
            "Expected 'minimum_polynomial_order' to be less than or equal to "
            "'maximum_polynomial_order', but received {} and {}, "
            "respectively.", get_minimum_polynomial_order(),
            get_maximum_polynomial_order())
    log_info("Minimum polynomial order = {}", get_minimum_polynomial_order());
    log_info("Maximum polynomial order = {}", get_maximum_polynomial_order());

    // Fitting tolerances.
    checkPropertyValueIsInRangeOrSet(getProperty_path_length_tolerance(),
                                     0.0, 1.0, {});
    checkPropertyValueIsInRangeOrSet(getProperty_moment_arm_tolerance(),
                                     0.0, 1.0, {});
    log_info("Path length fitting tolerance = {:1.1e} meters",
             get_path_length_tolerance(), 1);
    log_info("Moment arm fitting tolerance = {:1.1e} meters",
            get_moment_arm_tolerance(), 1);

    // Output directory.
    std::string outputDir = get_output_directory();
    if (outputDir.empty()) {
        if (getDocumentDirectory().empty()) {
            outputDir = SimTK::Pathname::getCurrentWorkingDirectory();
        } else {
            outputDir = getDocumentDirectory();
        }
    } else {
        IO::makeDir(outputDir);
    }

    // Sample coordinate values around the provided trajectory.
    // --------------------------------------------------------
    log_info("");
    log_info("Step 4/9: Sample coordinate values around the provided trajectory.");
    log_info("------------------------------------------------------------------");
    TimeSeriesTable valuesSampled = sampleCoordinateValues(values);

    // Recompute the coupled coordinate values.
    auto tableProcessorSampled = TableProcessor(valuesSampled) |
                                 TabOpAppendCoupledCoordinateValues();
    valuesSampled = tableProcessorSampled.process(&model);
    log_info("Total number of samples = {}", valuesSampled.getNumRows());
    if (valuesSampled.getNumRows() < 500) {
        log_warn("The number of samples is less than 500. This may result in "
                 "poorly fit paths. Consider increasing the number of samples "
                 "per frame or the number of frames in the coordinate values "
                 "table.");
    }

    // Compute path lengths and moment arms.
    // -------------------------------------
    log_info("");
    log_info("Step 5/9: Compute path lengths and moment arms.");
    log_info("-----------------------------------------------");
    log_info("Computing path lengths and moment arms for the original "
             "coordinate data...");
    TimeSeriesTable pathLengths;
    TimeSeriesTable momentArms;
    computePathLengthsAndMomentArms(model, values, get_num_parallel_threads(),
            pathLengths, momentArms);

    log_info("");
    log_info("Computing path lengths and moment arms for the sampled "
             "coordinate data...");
    TimeSeriesTable pathLengthsSampled;
    TimeSeriesTable momentArmsSampled;
    computePathLengthsAndMomentArms(model, valuesSampled,
            get_num_parallel_threads(), pathLengthsSampled, momentArmsSampled);

    // Filter sampled data.
    // --------------------
    log_info("");
    log_info("Step 6/9: Filter the sampled path data.");
    log_info("---------------------------------------");
    MomentArmMap momentArmMap;
    filterSampledData(model, valuesSampled, pathLengthsSampled,
            momentArmsSampled, momentArmMap);

    // Fit the FunctionBasedPaths.
    // ---------------------------
    log_info("");
    log_info("Step 7/9: Fit the polynomial coefficients.");
    log_info("------------------------------------------");
    Set<FunctionBasedPath> functionBasedPaths = fitPolynomialCoefficients(
            model, valuesSampled, pathLengthsSampled, momentArmsSampled,
            momentArmMap);
    Array<std::string> pathNames;
    functionBasedPaths.getNames(pathNames);

    // Evaluate the fit.
    // -----------------
    log_info("");
    log_info("Step 8/9: Evaluate the fit.");
    log_info("---------------------------");

    // Find the longest path name.
    int longestPathName = 0;
    for (int i = 0; i < pathNames.getSize(); ++i) {
        std::string pathName = pathNames[i];
        if ((int)pathName.size() > longestPathName) {
            longestPathName = (int)pathName.size();
        }
    }

    // Print the information for each path.
    log_info("");
    std::string pathName = fmt::format("{}path",
            std::string((int)(0.5*longestPathName)-2, ' '));
    std::string fitName = fmt::format("{}polynomial fit", std::string(15, ' '));
    std::string line = fmt::format("{:{}} | {:{}}", pathName, longestPathName,
            fitName, 44);
    std::string separator(line.size(), '-');
    log_info(separator);
    log_info(line);
    log_info(separator);
    for (int i = 0; i < functionBasedPaths.getSize(); ++i) {
        auto path = functionBasedPaths.get(pathNames[i]);
        auto function = dynamic_cast<const MultivariatePolynomialFunction&>(
                path.getLengthFunction());
        int numCoefficients = function.getCoefficients().size();
        line = fmt::format("{:{}} | order = {}, dimension = "
                "{}, coefficients = {}", path.getName(), longestPathName,
                function.getOrder(), function.getDimension(), numCoefficients);
        log_info(line);
    }
    log_info(separator);

    // Add the FunctionBasedPaths to the model.
    log_info("");
    log_info("Computing path lengths and moment arms from the fitted paths...");
    Model modelFitted = model;
    modelFitted.initSystem();
    ModelFactory::replacePathsWithFunctionBasedPaths(modelFitted,
            functionBasedPaths);

    // Recompute the path lengths and moment arms.
    TimeSeriesTable pathLengthsFitted;
    TimeSeriesTable momentArmsFitted;
    computePathLengthsAndMomentArms(modelFitted, values,
            get_num_parallel_threads(), pathLengthsFitted, momentArmsFitted);

    TimeSeriesTable pathLengthsSampledFitted;
    TimeSeriesTable momentArmsSampledFitted;
    computePathLengthsAndMomentArms(modelFitted, valuesSampled,
            get_num_parallel_threads(), pathLengthsSampledFitted,
            momentArmsSampledFitted);

    // Remove moment arm columns that are not in the map.
    removeMomentArmColumns(momentArms, momentArmMap);
    removeMomentArmColumns(momentArmsFitted, momentArmMap);
    removeMomentArmColumns(momentArmsSampledFitted, momentArmMap);

    // Compute the RMS error between the original and fitted path lengths and
    // moment arms.
    computeFittingErrors(modelFitted, pathLengthsSampled, momentArmsSampled,
            pathLengthsSampledFitted, momentArmsSampledFitted,
            get_path_length_tolerance(), get_moment_arm_tolerance());

    // Print out results.
    // ------------------
    log_info("");
    log_info("Step 9/9: Print the results to the output directory.");
    log_info("----------------------------------------------------");

    // Print the FunctionBasedPaths to file.
    std::string functionBasedPathsFileName =
            SimTK::Pathname::getAbsolutePathname(
                    fmt::format("{}/{}_FunctionBasedPathSet.xml",
                            outputDir, model.getName()));
    log_info("Printing the FunctionBasedPaths to '{}'...",
            functionBasedPathsFileName);
    functionBasedPaths.print(functionBasedPathsFileName);

    // Print the coordinate values to file.
    std::string coordinatesFileName =
            SimTK::Pathname::getAbsolutePathname(
                    fmt::format("{}/{}_coordinate_values.sto",
                            outputDir, model.getName()));
    std::string sampledCoordinatesFileName =
            SimTK::Pathname::getAbsolutePathname(
                    fmt::format("{}/{}_coordinate_values_sampled.sto",
                            outputDir, model.getName()));
    log_info("");
    log_info(fmt::format("Printing original coordinate values to '{}'...",
            coordinatesFileName));
    STOFileAdapter::write(values, coordinatesFileName);
    log_info(fmt::format("Printing sampled coordinate values to '{}'...",
            sampledCoordinatesFileName));
    STOFileAdapter::write(valuesSampled, sampledCoordinatesFileName);

    // Print the path lengths and moment arms from the original coordinate
    // data to file.
    std::string pathLengthsFileName =
            SimTK::Pathname::getAbsolutePathname(
                    fmt::format("{}/{}_path_lengths.sto",
                            outputDir, model.getName()));
    std::string momentArmsFileName =
            SimTK::Pathname::getAbsolutePathname(
                    fmt::format("{}/{}_moment_arms.sto",
                            outputDir, model.getName()));
    log_info("");
    log_info("Printing the path lengths to '{}'...",
            pathLengthsFileName);
    STOFileAdapter::write(pathLengths, pathLengthsFileName);
    log_info("Printing the moment arms to '{}'...", momentArmsFileName);
    STOFileAdapter::write(momentArms, momentArmsFileName);

    // Print the path lengths and moment arms from the sampled coordinate
    // data to file.
    std::string pathLengthsSampledFileName =
            SimTK::Pathname::getAbsolutePathname(
                    fmt::format("{}/{}_path_lengths_sampled.sto",
                            outputDir, model.getName()));
    std::string momentArmsSampledFileName =
            SimTK::Pathname::getAbsolutePathname(
                    fmt::format("{}/{}_moment_arms_sampled.sto",
                            outputDir, model.getName()));

    log_info("");
    log_info("Printing the sampled path lengths to '{}'...",
            pathLengthsSampledFileName);
    STOFileAdapter::write(pathLengths, pathLengthsSampledFileName);
    log_info("Printing the sampled moment arms to '{}'...",
            momentArmsSampledFileName);
    STOFileAdapter::write(momentArms, momentArmsSampledFileName);

    // Print the fitted path lengths and moment arms using the original
    // coordinate data to file.
    std::string pathLengthsFittedFileName =
            SimTK::Pathname::getAbsolutePathname(
                    fmt::format("{}/{}_path_lengths_fitted.sto",
                            outputDir, model.getName()));
    std::string momentArmsFittedFileName =
            SimTK::Pathname::getAbsolutePathname(
                    fmt::format("{}/{}_moment_arms_fitted.sto",
                            outputDir, model.getName()));

    log_info("");
    log_info("Printing the fitted path lengths from the original coordinate "
             "values to '{}'...", pathLengthsFittedFileName);
    STOFileAdapter::write(pathLengthsFitted, pathLengthsFittedFileName);
    log_info("Printing the fitted moment arms from the original coordinate "
             "to '{}'...", momentArmsFittedFileName);
    STOFileAdapter::write(momentArmsFitted, momentArmsFittedFileName);

    // Print the fitted path lengths and moment arms using the sampled
    // coordinate data to file.
    std::string pathLengthsSampledFittedFileName =
            SimTK::Pathname::getAbsolutePathname(
                    fmt::format("{}/{}_path_lengths_sampled_fitted.sto",
                            outputDir, model.getName()));
    std::string momentArmsSampledFittedFileName =
            SimTK::Pathname::getAbsolutePathname(
                    fmt::format("{}/{}_moment_arms_sampled_fitted.sto",
                            outputDir, model.getName()));

    log_info("");
    log_info("Printing the fitted path lengths from the sampled coordinate "
             "values to '{}'...", pathLengthsSampledFittedFileName);
    STOFileAdapter::write(pathLengthsSampledFitted,
            pathLengthsSampledFittedFileName);
    log_info("Printing the fitted moment arms from the sampled coordinate "
             "values to '{}'...", momentArmsSampledFittedFileName);
    STOFileAdapter::write(momentArmsSampledFitted,
            momentArmsSampledFittedFileName);
}

TimeSeriesTable PolynomialPathFitter::loadCoordinateValuesAndValidateModel(
        const std::string& documentDir, TableProcessor tableProcessor,
        Model& model) {

    const auto pathList = model.getComponentList<AbstractGeometryPath>();
    int numPaths = (int)std::distance(pathList.begin(), pathList.end());
    OPENSIM_THROW_IF(!numPaths, Exception,
            "Expected the model to contain at least one AbstractGeometryPath, "
            "but it does not.")

    const auto fbPathList = model.getComponentList<FunctionBasedPath>();
    int numFunctionBasedPaths = (int)std::distance(fbPathList.begin(),
            fbPathList.end());
    OPENSIM_THROW_IF(numFunctionBasedPaths, Exception,
            "Expected the model to not contain any FunctionBasedPaths, but it "
            "does. Please remove all FunctionBasedPaths from the model before "
            "running the PolynomialPathFitter.")

    // Load the coordinate values.
    tableProcessor.append(TabOpConvertDegreesToRadians());
    tableProcessor.append(TabOpUseAbsoluteStateNames());
    tableProcessor.append(TabOpAppendCoupledCoordinateValues());
    TimeSeriesTable values = tableProcessor.process(documentDir, &model);
    log_info("Coordinate values table: {} columns, {} time points",
            values.getNumColumns(), values.getNumRows());

    // Validate the coordinate values table
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
            OPENSIM_THROW_IF(!values.hasColumn(valuePath), Exception,
                    fmt::format("Expected the coordinate values table to "
                                "contain a column for '{}' (this coordinate is "
                                "not locked), but it does not.",
                            coordinate.getAbsolutePathString()))
        }
    }

    // If we detected any joints to be welded, update the model.
    if (!jointsToWeld.empty()) {
        log_info("Welding the following locked joints (no data provided): ");
        for (const auto& jointName : jointsToWeld) {
            log_info("  {}", jointName);
        }
        auto modelProcessor = ModelProcessor(model) |
                              ModOpReplaceJointsWithWelds(jointsToWeld);
        model = modelProcessor.process();
    }

    // Delete any columns in the coordinate values table that are not in the
    // model.
    std::vector<std::string> columnsToDelete;
    for (const auto& columnLabel : values.getColumnLabels()) {
        // Check if the column label ends with "/value".
        if (columnLabel.substr(columnLabel.size() - 6) != "/value") {
            values.removeColumn(columnLabel);
            continue;
        }
        // Check if the column label contains a coordinate path.
        const std::string coordinatePath = columnLabel.substr(
                0, columnLabel.size() - std::string("/value").size());
        if (!model.hasComponent(coordinatePath)) {
            values.removeColumn(columnLabel);
        }
    }

    return values;
}

TimeSeriesTable PolynomialPathFitter::sampleCoordinateValues(
        const TimeSeriesTable& values) {
    // Mute the Latin hypercube sampling output, so it doesn't print out for
    // every time point.
    Logger::Level origLoggerLevel = Logger::getLevel();
    Logger::setLevel(Logger::Level::Warn);
    
    // Create a Latin hypercube design to sample the coordinate values.
    LatinHypercubeDesign lhs;
    lhs.setNumSamples(get_num_samples_per_frame());
    lhs.setNumVariables((int)values.getNumColumns());
    
    // Helper function for sampling the coordinate values between two time
    // indexes.
    auto sampleCoordinateValuesSubset = [this, lhs](
            std::vector<int>::iterator begin_iter,
            std::vector<int>::iterator end_iter,
            const TimeSeriesTable& values)
                -> SimTK::Matrix {
        
        SimTK::Matrix results(
                lhs.getNumSamples()*(int)std::distance(begin_iter, end_iter),
                (int)values.getNumColumns());
        SimTK::Matrix design(lhs.getNumSamples(), lhs.getNumVariables());
        int thisTimeIndex = 0;
        for (auto it = begin_iter; it != end_iter; ++it) {
            // Generate the design and shift its values between [-1, 1].
            if (m_useStochasticEvolutionaryLHS) {
                design = lhs.generateStochasticEvolutionaryDesign();
            } else {
                design = lhs.generateRandomDesign();
            }
            design.elementwiseSubtractFromScalarInPlace(0.5);
            design *= 2;
            
            int icol = 0;
            for (const std::string& label : values.getColumnLabels()) {
                const SimTK::VectorView column = 
                        values.getDependentColumn(label);
                const auto& bounds = m_coordinateBoundsMap.at(label);
                const auto& range = m_coordinateRangeMap.at(label);
                
                // Linearly transform the design to the specified bounds for 
                // each coordinate.
                double slope = 0.5*(bounds[1] - bounds[0]);
                SimTK::Vector candidateCol = slope*(design.col(icol) + 1.0) +
                        bounds[0] + column[*it];
                
                // Check that all elements in the column are within the range of
                // motion. If not, move the element inside the range of motion
                // that is closest to the original value.
                for (double& elt : candidateCol) {
                    if (elt < range[0]) {
                        elt = range[0];
                    } else if (elt > range[1]) {
                        elt = range[1];
                    }
                }
                design.updCol(icol) = candidateCol;
                ++icol;
            }
            
            // Store the results.
            results.updBlock(thisTimeIndex*lhs.getNumSamples(), 0, 
                    lhs.getNumSamples(), (int)values.getNumColumns()) = design;
            ++thisTimeIndex;
        }

        return results;
    };

    // Divide the sampling across multiple threads.
    std::vector<int> timeIndexes(values.getNumRows());
    std::iota(timeIndexes.begin(), timeIndexes.end(), 0);
    int stride = static_cast<int>(
            std::floor(values.getNumRows() / get_num_parallel_threads()));
    std::vector<std::future<SimTK::Matrix>> futures;
    int offset = 0;
    for (int thread = 0; thread < get_num_parallel_threads(); ++thread) {
        auto begin_iter = timeIndexes.begin() + offset;
        auto end_iter = (thread == get_num_parallel_threads()-1) ?
                timeIndexes.end() :
                timeIndexes.begin() + offset + stride;
        futures.push_back(std::async(std::launch::async,
                sampleCoordinateValuesSubset,
                begin_iter, end_iter, values));
        offset += stride;
    }

    // Wait for threads to finish and collect the results.
    std::vector<SimTK::Matrix> outputs;
    outputs.reserve(get_num_parallel_threads());
    for (int i = 0; i < get_num_parallel_threads(); ++i) {
        outputs.push_back(futures[i].get());
    }
    
    // Reset the logger.
    OpenSim::Logger::setLevel(origLoggerLevel);

    // Assemble the results into one TimeSeriesTable.
    int timeIdx = 0;
    const auto& times = values.getIndependentColumn();
    TimeSeriesTable valuesSampled;
    double dt = (times[1] - times[0]) / (get_num_samples_per_frame() + 2);
    for (int i = 0; i < get_num_parallel_threads(); ++i) {
        int numTimeIndexes = outputs[i].nrow() / get_num_samples_per_frame();
        for (int j = 0; j < numTimeIndexes; ++j) {
            // Append the original values.
            valuesSampled.appendRow(times[timeIdx], 
                    values.getRowAtIndex(timeIdx));

            // Update the time step, if possible. Otherwise, use the last time
            // step.
            if (timeIdx+1 < static_cast<int>(values.getNumRows())) {
                dt = (times[timeIdx+1] - times[timeIdx]) / 
                     (get_num_samples_per_frame() + 2);
            }

            // Append the sampled values.
            for (int irow = 0; irow < get_num_samples_per_frame(); ++irow) {
                valuesSampled.appendRow(times[timeIdx] + (irow + 1)*dt,
                        outputs[i].row(irow + j*get_num_samples_per_frame()));
            }
            ++timeIdx;
        }
    }
    valuesSampled.addTableMetaData<std::string>("inDegrees", "no");
    valuesSampled.setColumnLabels(values.getColumnLabels());

    return valuesSampled;
}

void PolynomialPathFitter::computePathLengthsAndMomentArms(
        const Model& model,
        const TimeSeriesTable& coordinateValues,
        int numThreads,
        TimeSeriesTable& pathLengths,
        TimeSeriesTable& momentArms) {

    // Create a StatesTrajectory from the coordinate values.
    auto statesTrajectory = StatesTrajectory::createFromStatesTable(
            model, coordinateValues, true, false, false);

    // Determine the maximum number of path and moment arm evaluations.
    const auto& paths = model.getComponentList<AbstractGeometryPath>();
    int numPaths = (int)std::distance(paths.begin(), paths.end());
    int numCoordinates = (int)coordinateValues.getNumColumns();
    int numColumns = numPaths + (numPaths * numCoordinates);

    // Define helper function for path length and moment arm computations.
    auto calcPathLengthsAndMomentArmsSubset =
            [numThreads, numColumns, numPaths](Model model, int thread,
            StatesTrajectory::IteratorRange subsetStates) -> SimTK::Matrix {
        model.initSystem();

        int numTimePoints = (int)std::distance(subsetStates.begin(),
                subsetStates.end());
        log_info("Thread {:2d}/{:2d}: computing values for times "
                 "{:1.2f}-{:1.2f} seconds...", thread+1, numThreads,
                 subsetStates.begin()->getTime(),
                 (subsetStates.end()-1)->getTime());

        SimTK::Matrix results(numTimePoints, numColumns);
        int row = 0;
        const auto& forces = model.getComponentList<Force>();
        for (const auto& state : subsetStates) {
            model.realizePosition(state);

            int ip = 0;
            int ima = 0;
            for (const auto& force : forces) {
                if (force.hasProperty("path")) {
                    const AbstractGeometryPath& path =
                        force.getPropertyByName<AbstractGeometryPath>("path")
                            .getValue();

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
            std::floor(coordinateValues.getNumRows() / numThreads));
    std::vector<std::future<SimTK::Matrix>> futures;
    int offset = 0;
    for (int thread = 0; thread < numThreads; ++thread) {
        auto begin_iter = statesTrajectory.begin() + offset;
        auto end_iter = (thread == numThreads-1) ?
                statesTrajectory.end() :
                statesTrajectory.begin() + offset + stride;
        futures.push_back(std::async(std::launch::async,
                calcPathLengthsAndMomentArmsSubset,
                model, thread,
                makeIteratorRange(begin_iter, end_iter)));
        offset += stride;
    }

    // Wait for threads to finish and collect results
    std::vector<SimTK::Matrix> outputs;
    outputs.reserve(numThreads);
    for (int i = 0; i < numThreads; ++i) {
        outputs.push_back(futures[i].get());
    }

    // Assemble results into one TimeSeriesTable
    std::vector<double> times = coordinateValues.getIndependentColumn();
    int itime = 0;
    for (int i = 0; i < numThreads; ++i) {
        for (int j = 0; j < outputs[i].nrow(); ++j) {
            pathLengths.appendRow(times[itime],
                    outputs[i].block(j, 0, 1, numPaths).getAsRowVector());
            momentArms.appendRow(times[itime], outputs[i].block(j, numPaths, 1,
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
    pathLengths.setColumnLabels(pathLengthLabels);
    momentArms.setColumnLabels(momentArmLabels);
}

void PolynomialPathFitter::filterSampledData(const Model& model,
        TimeSeriesTable& coordinateValues,
        TimeSeriesTable& pathLengths,
        TimeSeriesTable& momentArms,
        MomentArmMap& momentArmMap) {

    // Remove moment arm columns for coupled coordinates.
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
            bool removeColumn = col.normInf() < get_moment_arm_threshold();
            std::string path = label.substr(0, label.find("_moment_arm_"));
            std::string coordinate = label.substr(
                    label.find("_moment_arm_") + 12);

            if (removeColumn) {
                momentArms.removeColumn(label);
            } else {
                momentArmMap[path].push_back(coordinate);
                OPENSIM_THROW_IF_FRMOBJ(momentArmMap[path].size() > 6,
                        Exception,
                        "The path '{}' depends on more than 6 coordinates. "
                        "This is not supported.", path)
            }
        }
    }

    // Remove sample points that fall outside two standard deviations of the
    // data in each column.
    double threshold = 5.0;
    std::vector<double> rejectedTimePoints;
    auto rejectTimePoints = [this, threshold](
            const TimeSeriesTable& table,
            std::vector<double>& rejectedTimePoints) {
        const auto& times = table.getIndependentColumn();
        int increment = get_num_samples_per_frame() + 1;
        int numOriginalTimes = (int)table.getNumRows() / increment;
        for (const auto& label : table.getColumnLabels()) {
            SimTK::Vector column = table.getDependentColumn(label);

            SimTK::Vector std(numOriginalTimes, 0.0);
            for (int i = 0; i < numOriginalTimes; ++i) {
                // Compute the mean and standard deviation segments of the
                // column segment.
                SimTK::Vector segment = column.block(i*increment, 0,
                        increment, 1).getAsVector();
                double mean = segment.sum() / segment.size();
                segment.elementwiseSubtractFromScalarInPlace(mean);
                std[i] = std::sqrt(segment.normSqr() / segment.size());
            }

            // Compute the average standard deviation.
            double avgStd = std.sum() / std.size();

            // Find the time points that deviate too far from the nominal
            // values.
            int currentNominalIndex = 0;
            for (int i = 0; i < column.size(); ++i) {
                double nominal = column[currentNominalIndex];
                if (std::abs(column[i] - nominal) > threshold * avgStd) {
                    rejectedTimePoints.push_back(times[i]);
                }
                if (i % increment == 0) {
                    currentNominalIndex += increment;
                }
            }
        }
    };
    rejectTimePoints(pathLengths, rejectedTimePoints);
    rejectTimePoints(momentArms, rejectedTimePoints);

    // Remove duplicate time points.
    std::sort(rejectedTimePoints.begin(), rejectedTimePoints.end());
    rejectedTimePoints.erase(std::unique(rejectedTimePoints.begin(),
            rejectedTimePoints.end()), rejectedTimePoints.end());

    // Remove the rejected time points.
    if (!rejectedTimePoints.empty()) {
        double percentTotal = 100.0 * (int)rejectedTimePoints.size() /
                                      (int)coordinateValues.getNumRows();
        log_info("Removing {} samples ({:1.1f}% of total) that are larger than "
                 "{} standard deviations from nominal values...",
                rejectedTimePoints.size(), percentTotal, threshold);
        for (double time : rejectedTimePoints) {
            coordinateValues.removeRow(time);
            pathLengths.removeRow(time);
            momentArms.removeRow(time);
        }
    }
}

Set<FunctionBasedPath> PolynomialPathFitter::fitPolynomialCoefficients(
        const Model& model,
        const TimeSeriesTable& coordinateValues,
        const TimeSeriesTable& pathLengths,
        const TimeSeriesTable& momentArms,
        const MomentArmMap& momentArmMap) {

    // n-choose-k helper function.
    // stackoverflow.com/questions/15301885/best-way-of-calculating-n-choose-k
    std::function<int(int, int)> choose;
    choose = [&choose](int n, int k) -> int {
        if (k == 0) { return 1; }
        return (n * choose(n - 1, k - 1)) / k;
    };

    // Coordinate references.
    // ----------------------
    const CoordinateSet& coordinateSet = model.getCoordinateSet();
    const int numTimes = (int)coordinateValues.getNumRows();

    // Pre-compute variables.
    // ----------------------
    const auto forces = model.getComponentList<Force>();
    const int numForces = (int)std::distance(forces.begin(), forces.end());
    // Force indexes and paths.
    std::vector<int> forceIndexes;
    std::vector<std::string> forcePaths;
    forceIndexes.reserve(numForces);
    forcePaths.reserve(numForces);
    int forceIndex = 0;
    for (const auto& force : forces) {
        forceIndexes.push_back(forceIndex++);
        forcePaths.push_back(force.getAbsolutePathString());
    }

    // Build a FunctionBasedPath for each path-based force in the model.
    // -----------------------------------------------------------------
    // Solve A*x = b, where x is the vector of coefficients for the
    // FunctionBasedPath, A is a matrix of polynomial terms, and b is a vector
    // of path lengths and moment arms.
    auto fitForcePolynomialSubset = [&](
            std::vector<int>::iterator begin_iter,
            std::vector<int>::iterator end_iter,
            int thread) -> std::vector<std::unique_ptr<FunctionBasedPath>> {

        std::vector<std::unique_ptr<FunctionBasedPath>> thesePaths;
        thesePaths.reserve(std::distance(begin_iter, end_iter));
        for (auto it = begin_iter; it != end_iter; ++it) {
            int iforce = *it;
            const std::string& forcePath = forcePaths[iforce];

            // Check if the current force is dependent on any coordinates in the
            // model. If not, skip it.
            if (momentArmMap.find(forcePath) == momentArmMap.end()) {
                continue;
            }
            log_info("Thread {:2d}/{:2d}: fitting coefficients for force "
                     "'{}'...", thread+1, get_num_parallel_threads(),
                    forcePath);

            // The current force path and the number of coordinates it depends
            // on.
            std::vector<std::string> coordinatesNamesThisForce =
                    momentArmMap.at(forcePath);
            int numCoordinatesThisForce = (int)coordinatesNamesThisForce.size();
            std::vector<std::string> coordinatePathsThisForce;
            coordinatePathsThisForce.reserve(numCoordinatesThisForce);
            for (const auto& coordinateName : coordinatesNamesThisForce) {
                coordinatePathsThisForce.push_back(
                    coordinateSet.get(coordinateName).getAbsolutePathString());
            }

            // Initialize the 'b' vector. This is the same for all polynomial
            // orders.
            SimTK::Vector b(numTimes * (numCoordinatesThisForce + 1), 0.0);

            // The path lengths for this force. This is the first N elements of
            // the 'b' vector.
            b(0, numTimes) = pathLengths.getDependentColumn(
                    fmt::format("{}_length", forcePath));

            // The moment arms this force and coordinates associated with this
            // force. The moment arms are the remaining elements of the 'b'
            // vector.
            SimTK::Matrix coordinatesThisForce(
                    numTimes, numCoordinatesThisForce, 0.0);
            for (int ic = 0; ic < numCoordinatesThisForce; ++ic) {
                const std::string& coordinateName =
                        coordinatesNamesThisForce[ic];
                b((ic+1)*numTimes, numTimes) = momentArms.getDependentColumn(
                        fmt::format("{}_moment_arm_{}", forcePath,
                                coordinateName));

                const SimTK::VectorView coordinateValuesThisCoordinate =
                        coordinateValues.getDependentColumn(
                            fmt::format("{}/value",
                                        coordinatePathsThisForce[ic]));
                for (int itime = 0; itime < numTimes; ++itime) {
                    coordinatesThisForce.set(
                            itime, ic, coordinateValuesThisCoordinate[itime]);
                }
            }

            // Polynomial fitting.
            // -------------------
            SimTK::Vector coefficients;
            int order = get_minimum_polynomial_order();
            while (true) {
                // Initialize the multivariate polynomial function.
                int numCoefficients =
                        choose(numCoordinatesThisForce + order, order);
                SimTK::Vector dummyCoefficients(numCoefficients, 1.0);
                MultivariatePolynomialFunction dummyFunction(dummyCoefficients,
                        numCoordinatesThisForce, order);

                // Initialize the 'A' matrix.
                SimTK::Matrix A(numTimes * (numCoordinatesThisForce + 1),
                        numCoefficients, 0.0);

                // Fill in the A matrix. This contains the polynomial terms for
                // the path length and moment arms.
                for (int itime = 0; itime < numTimes; ++itime) {
                    A(itime, 0, 1, numCoefficients) =
                            dummyFunction.getTermValues(
                                coordinatesThisForce.row(itime).getAsVector());

                    for (int ic = 0; ic < numCoordinatesThisForce; ++ic) {
                        SimTK::Vector termDerivatives =
                            dummyFunction.getTermDerivatives({ic},
                                coordinatesThisForce.row(itime).getAsVector())
                                .negate();
                        A((ic+1)*numTimes + itime, 0, 1, numCoefficients) =
                                termDerivatives;
                    }
                }

                // Solve the least-squares problem.
                SimTK::FactorQTZ factor(A);
                factor.solve(b, coefficients);

                // Calculate the RMS error.
                SimTK::Vector b_fit = A * coefficients;
                SimTK::Vector error = b - b_fit;

                // If the fit achieves the path length and moment arm thresholds
                // we set, then exit the loop.
                SimTK::Vector pathLengthError = error.block(
                        0, 0, numTimes, 1).getAsVector();
                SimTK::Vector momentArmError = error.block(
                        numTimes, 0, numTimes * numCoordinatesThisForce,
                        1).getAsVector();
                double pathLengthRMSError = std::sqrt(
                        pathLengthError.normSqr() / pathLengthError.size());
                double momentArmRMSError = std::sqrt(
                        momentArmError.normSqr() / momentArmError.size());
                if ((pathLengthRMSError < get_path_length_tolerance() &&
                        momentArmRMSError < get_moment_arm_tolerance()) ||
                        order == get_maximum_polynomial_order()) {
                    break;
                }
                ++order;
            }

            // Create a FunctionBasedPath for the current path-based force.
            MultivariatePolynomialFunction lengthFunction;
            lengthFunction.setDimension(numCoordinatesThisForce);
            lengthFunction.setOrder(order);
            lengthFunction.setCoefficients(coefficients);
            auto functionBasedPath = make_unique<FunctionBasedPath>();
            functionBasedPath->setName(forcePath);
            functionBasedPath->setCoordinatePaths(coordinatePathsThisForce);
            functionBasedPath->setLengthFunction(lengthFunction);
            if (getIncludeMomentArmFunctions()) {
                for (int iq = 0; iq < numCoordinatesThisForce; ++iq) {
                    MultivariatePolynomialFunction momentArmFunction =
                            lengthFunction.generateDerivativeFunction(iq, true);
                    functionBasedPath->appendMomentArmFunction(
                            momentArmFunction);
                }
            }
            if (getIncludeLengtheningSpeedFunction()) {
                MultivariatePolynomialFunction lengtheningSpeedFunction = 
                        lengthFunction.generatePartialVelocityFunction();
                functionBasedPath->setLengtheningSpeedFunction(
                        lengtheningSpeedFunction);
            }

            // Save the FunctionBasedPath.
            thesePaths.push_back(std::move(functionBasedPath));
        }

        return thesePaths;
    };

    // Divide the polynomial fitting across multiple threads.
    std::vector<std::future<std::vector<std::unique_ptr<FunctionBasedPath>>>>
            futures;
    int stride = static_cast<int>(std::floor(
            numForces / get_num_parallel_threads()));
    int offset = 0;
    for (int thread = 0; thread < get_num_parallel_threads(); ++thread) {
        auto begin_iter = forceIndexes.begin() + offset;
        auto end_iter = (thread == get_num_parallel_threads()-1) ?
                forceIndexes.end() :
                forceIndexes.begin() + offset + stride;
        futures.push_back(std::async(std::launch::async,
                        fitForcePolynomialSubset,
                        begin_iter, end_iter, thread));
        offset += stride;
    }

    // Wait for threads to finish and collect the results.
    Set<FunctionBasedPath> functionBasedPaths;
    for (int thread = 0; thread < get_num_parallel_threads(); ++thread) {
        auto thesePaths = futures[thread].get();
        for (auto& path : thesePaths) {
            functionBasedPaths.adoptAndAppend(path.release());
        }
    }

    return functionBasedPaths;
}

//=============================================================================
// SETTINGS
//=============================================================================

void PolynomialPathFitter::setModel(ModelProcessor model) {
    set_model(std::move(model));
}

void PolynomialPathFitter::setCoordinateValues(TableProcessor values) {
    set_coordinate_values(std::move(values));
}

void PolynomialPathFitter::setMomentArmThreshold(double threshold) {
    set_moment_arm_threshold(threshold);
}

double PolynomialPathFitter::getMomentArmThreshold() const {
    return get_moment_arm_threshold();
}

void PolynomialPathFitter::setMinimumPolynomialOrder(int order) {
    set_minimum_polynomial_order(order);
}

int PolynomialPathFitter::getMinimumPolynomialOrder() const {
    return get_minimum_polynomial_order();
}

void PolynomialPathFitter::setMaximumPolynomialOrder(int order) {
    set_maximum_polynomial_order(order);
}

int PolynomialPathFitter::getMaximumPolynomialOrder() const {
    return get_maximum_polynomial_order();
}

void PolynomialPathFitter::setGlobalCoordinateSamplingBounds(SimTK::Vec2 bounds)
{
    set_global_coordinate_sampling_bounds(std::move(bounds));
}

SimTK::Vec2 PolynomialPathFitter::getGlobalCoordinateSamplingBounds() const {
    return get_global_coordinate_sampling_bounds();
}

void PolynomialPathFitter::appendCoordinateSamplingBounds(
        const std::string& coordinatePath, const SimTK::Vec2& bounds) {
    append_coordinate_sampling_bounds({coordinatePath, bounds});
}

void PolynomialPathFitter::setMomentArmTolerance(double tolerance) {
    set_moment_arm_tolerance(tolerance);
}

double PolynomialPathFitter::getMomentArmTolerance() const {
    return get_moment_arm_tolerance();
}

void PolynomialPathFitter::setPathLengthTolerance(double tolerance) {
    set_path_length_tolerance(tolerance);
}

double PolynomialPathFitter::getPathLengthTolerance() const {
    return get_path_length_tolerance();
}

void PolynomialPathFitter::setNumSamplesPerFrame(int numSamples) {
    set_num_samples_per_frame(numSamples);
}

int PolynomialPathFitter::getNumSamplesPerFrame() const {
    return get_num_samples_per_frame();
}

void PolynomialPathFitter::setNumParallelThreads(int numThreads) {
    set_num_parallel_threads(numThreads);
}

int PolynomialPathFitter::getNumParallelThreads() const {
    return get_num_parallel_threads();
}

void PolynomialPathFitter::setLatinHypercubeAlgorithm(
        std::string algorithm) {
    set_latin_hypercube_algorithm(std::move(algorithm));
}

std::string PolynomialPathFitter::getLatinHypercubeAlgorithm() const {
    return get_latin_hypercube_algorithm();
}

void PolynomialPathFitter::setOutputDirectory(std::string directory) {
    set_output_directory(std::move(directory));
}

std::string PolynomialPathFitter::getOutputDirectory() const {
    return get_output_directory();
}

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================

void PolynomialPathFitter::evaluateFunctionBasedPaths(Model model,
        TableProcessor trajectory,
        const std::string& functionBasedPathsFileName,
        double pathLengthTolerance, double momentArmTolerance) {

    // Initialize the original model.
    model.initSystem();

    // Create another model and add the FunctionBasedPaths to it.
    ModelProcessor modelProcessor = ModelProcessor(model) |
            ModOpReplacePathsWithFunctionBasedPaths(functionBasedPathsFileName);
    Model modelFitted = modelProcessor.process();

    // Create a moment arm map based on the fitted model paths.
    MomentArmMap momentArmMap;
    Set<FunctionBasedPath> functionBasedPaths(functionBasedPathsFileName);
    for (int i = 0; i < functionBasedPaths.getSize(); ++i) {
        const auto& path = functionBasedPaths.get(i);
        std::vector<std::string> coordinateNames;
        for (const auto& coordinatePath : path.getCoordinatePaths()) {
            const auto& coordinate =
                    model.getComponent<Coordinate>(coordinatePath);
            coordinateNames.push_back(coordinate.getName());
        }

        momentArmMap[path.getName()] = coordinateNames;
    }

    // Get the coordinate values table from the trajectory. This may contain
    // extra values, but that's okay.
    std::string currentDirectory = IO::getCwd();
    TimeSeriesTable coordinateValues = loadCoordinateValuesAndValidateModel(
            currentDirectory, std::move(trajectory), model);

    // Compute path lengths and moment arms for the original and fitted models.
    log_info("");
    log_info("Computing path lengths and moment arms for the original model..");
    TimeSeriesTable pathLengths;
    TimeSeriesTable momentArms;
    int numThreads = (int)std::thread::hardware_concurrency()-2;
    computePathLengthsAndMomentArms(model, coordinateValues, numThreads,
            pathLengths, momentArms);

    log_info("");
    log_info("Computing path lengths and moment arms for the fitted model..");
    TimeSeriesTable pathLengthsFitted;
    TimeSeriesTable momentArmsFitted;
    computePathLengthsAndMomentArms(modelFitted, coordinateValues, numThreads,
            pathLengthsFitted, momentArmsFitted);

    // Remove moment arm columns that are not in the map.
    removeMomentArmColumns(momentArms, momentArmMap);
    removeMomentArmColumns(momentArmsFitted, momentArmMap);

    // Compute the RMS errors.
    computeFittingErrors(modelFitted, pathLengths, momentArms,
            pathLengthsFitted, momentArmsFitted,
            pathLengthTolerance, momentArmTolerance);
}

void PolynomialPathFitter::removeMomentArmColumns(TimeSeriesTable& momentArms,
        const MomentArmMap& momentArmMap) {

    // Remove entries from the table that are not in the moment arm map.
    for (const auto& label : momentArms.getColumnLabels()) {
        std::string path = label.substr(0, label.find("_moment_arm_"));
        std::string coordinate = label.substr(
                label.find("_moment_arm_") + 12);
        if (momentArmMap.find(path) != momentArmMap.end()) {
            if (std::find(momentArmMap.at(path).begin(),
                        momentArmMap.at(path).end(), coordinate) ==
                    momentArmMap.at(path).end()) {
                momentArms.removeColumn(label);
            }
        } else {
            momentArms.removeColumn(label);
        }
    }
}

void PolynomialPathFitter::computeFittingErrors(const Model& modelFitted,
        const TimeSeriesTable& pathLengths, const TimeSeriesTable& momentArms,
        const TimeSeriesTable& pathLengthsFitted,
        const TimeSeriesTable& momentArmsFitted, double pathLengthTolerance,
        double momentArmTolerance) {

    // Convert the tolerances to centimeters.
    pathLengthTolerance *= 100.0;
    momentArmTolerance *= 100.0;

    // Helper function for printing warning messages.
    auto printWarningMessage = [](const std::string& pathLengthOrMomentArm,
                                double tolerance) {
        log_warn("-----------------------------------------------------------");
        log_warn(fmt::format("The {} RMS error is greater than {:g} cm.",
                pathLengthOrMomentArm, tolerance));
        log_warn("");
        log_warn("Consider increasing the number of samples per frame or the ");
        log_warn("polynomial order and re-fitting the model. If a re-fitting ");
        log_warn("is not successful, check that the original model produces ");
        log_warn("path lengths and moment arms that are free of ");
        log_warn("discontinuities or other irregularities.");
        log_warn("-----------------------------------------------------------");
    };

    // Check inputs.
    OPENSIM_ASSERT_ALWAYS(pathLengths.getNumRows() ==
            pathLengthsFitted.getNumRows());
    OPENSIM_ASSERT_ALWAYS(momentArms.getNumRows() ==
            momentArmsFitted.getNumRows());
    OPENSIM_ASSERT_ALWAYS(pathLengths.getNumColumns() ==
            pathLengthsFitted.getNumColumns());
    OPENSIM_ASSERT_ALWAYS(momentArms.getNumColumns() ==
            momentArmsFitted.getNumColumns());
    const auto& functionBasedPaths =
            modelFitted.getComponentList<FunctionBasedPath>();
    int numPaths = (int)std::distance(functionBasedPaths.begin(),
            functionBasedPaths.end());
    OPENSIM_THROW_IF(numPaths == 0, Exception,
            "Expected the model to contain 'FunctionBasedPath's, but none were "
            "found.");

    // Print the path length and moment arm RMS errors for each path.
    log_info("");
    log_info("Summary of path length and moment arm RMS errors");
    log_info("------------------------------------------------");
    SimTK::Vector pathLengthRMSErrors((int)pathLengths.getNumColumns());
    SimTK::Vector momentArmRMSErrors((int)momentArms.getNumColumns());
    int ip = 0;
    int ima = 0;
    for (const auto& path : modelFitted.getComponentList<FunctionBasedPath>()) {
        std::string pathName = path.getAbsolutePathString();
        pathName = pathName.substr(0, pathName.find("/path"));
        log_info("'{}' path errors:", pathName);

        SimTK::Vector pathLength = pathLengths.getDependentColumn(
                fmt::format("{}_length", pathName));
        SimTK::Vector pathLengthFitted = pathLengthsFitted.getDependentColumn(
                fmt::format("{}_length", pathName));
        SimTK::Vector pathLengthError = pathLength - pathLengthFitted;
        double pathLengthRMSError = 100.0 * std::sqrt(
                pathLengthError.normSqr() / pathLengthError.size());
        pathLengthRMSErrors[ip++] = pathLengthRMSError;
        log_info(" - path length RMSE: {:1.3f} cm", pathLengthRMSError);

        if (pathLengthRMSError > 10.0*pathLengthTolerance) {
            printWarningMessage("path length", pathLengthTolerance);
        }

        for (const auto& coordinatePath : path.getCoordinatePaths()) {
            const auto& coordinate =
                    modelFitted.getComponent<Coordinate>(coordinatePath);
            const std::string& coordinateName = coordinate.getName();

            SimTK::Vector momentArm = momentArms.getDependentColumn(
                    fmt::format("{}_moment_arm_{}", pathName, coordinateName));
            SimTK::Vector momentArmFitted = momentArmsFitted.getDependentColumn(
                    fmt::format("{}_moment_arm_{}", pathName, coordinateName));
            SimTK::Vector momentArmError = momentArm - momentArmFitted;
            double momentArmRMSError = 100.0 * std::sqrt(
                    momentArmError.normSqr() / momentArmError.size());
            momentArmRMSErrors[ima++] = momentArmRMSError;
            log_info(" - '{}' moment arm RMSE: {:1.3f} cm", coordinateName,
                    momentArmRMSError);

            if (momentArmRMSError > 10.0*momentArmTolerance) {
                printWarningMessage(
                        fmt::format("'{}' moment arm", coordinateName),
                        momentArmTolerance);
            }

        }
        log_info("");
    }

    // Print the average path length and moment arm RMS errors.
    double averagePathLengthError = pathLengthRMSErrors.sum() /
                                    pathLengthRMSErrors.size();
    double averageMomentArmError = momentArmRMSErrors.sum() /
                                   momentArmRMSErrors.size();
    log_info("Average path length RMSE = {:1.3f} cm", averagePathLengthError);
    if (averagePathLengthError > pathLengthTolerance) {
        printWarningMessage("average path length", pathLengthTolerance);
    }

    log_info("Average moment arm RMSE  = {:1.3f} cm", averageMomentArmError);
    if (averageMomentArmError > momentArmTolerance) {
        printWarningMessage("average moment arm", momentArmTolerance);
    }
}

void PolynomialPathFitter::constructProperties() {
    constructProperty_model(ModelProcessor());
    constructProperty_coordinate_values(TableProcessor());
    constructProperty_moment_arm_threshold(1e-3);
    constructProperty_moment_arm_tolerance(1e-4);
    constructProperty_path_length_tolerance(1e-4);
    constructProperty_minimum_polynomial_order(2);
    constructProperty_maximum_polynomial_order(6);
    constructProperty_num_parallel_threads(
            (int)std::thread::hardware_concurrency()-2);
    constructProperty_global_coordinate_sampling_bounds({-10.0, 10.0});
    constructProperty_coordinate_sampling_bounds();
    constructProperty_num_samples_per_frame(25);
    constructProperty_latin_hypercube_algorithm("random");
    constructProperty_output_directory("");
    constructProperty_include_moment_arm_functions(false);
    constructProperty_include_lengthening_speed_function(false);
}

std::string PolynomialPathFitter::getDocumentDirectory() const {
    std::string setupDir;
    {
        bool dontApplySearchPath;
        std::string fileName, extension;
        SimTK::Pathname::deconstructPathname(getDocumentFileName(),
                dontApplySearchPath, setupDir, fileName, extension);
    }
    return setupDir;
}
