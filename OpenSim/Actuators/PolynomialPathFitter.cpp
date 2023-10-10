/* -------------------------------------------------------------------------- *
 *                    OpenSim:  PolynomialPathFitter.h                        *
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

using namespace OpenSim;

//=============================================================================
// FUNCTION-BASED PATH FITTER BOUNDS
//=============================================================================
PolynomialPathFitterBounds::FunctionBasedPathFitterBounds() : Object()
{
    setAuthors("Nicholas Bianco");
    constructProperties();
}

PolynomialPathFitterBounds::FunctionBasedPathFitterBounds(
        const std::string& coordinatePath, const SimTK::Vec2& bounds) :
        FunctionBasedPathFitterBounds() {
    set_coordinate_path(coordinatePath);
    OPENSIM_THROW_IF_FRMOBJ(bounds[0] >= bounds[1], Exception,
            "Expected the lower bound to be less than the upper bound, but "
            "this is not the case.")
    set_bounds(bounds);
}

void PolynomialPathFitterBounds::constructProperties() {
    constructProperty_coordinate_path("");
    constructProperty_bounds({-5.0, 5.0});
}

//=============================================================================
// FUNCTION-BASED PATH FITTER
//=============================================================================

PolynomialPathFitter::FunctionBasedPathFitter() : Object()
{
    setAuthors("Nicholas Bianco");
    constructProperties();
}

PolynomialPathFitter::~FunctionBasedPathFitter() noexcept = default;

PolynomialPathFitter::FunctionBasedPathFitter(
        PolynomialPathFitter const&) = default;

PolynomialPathFitter& PolynomialPathFitter::operator=(
        const PolynomialPathFitter&) = default;

PolynomialPathFitter::FunctionBasedPathFitter(
        PolynomialPathFitter&& other) = default;

PolynomialPathFitter& PolynomialPathFitter::operator=(
        PolynomialPathFitter&& other) = default;

void PolynomialPathFitter::setModel(Model model) {
    //    set_model(std::move(model));
    m_model = std::move(model);
}
void PolynomialPathFitter::setCoordinateValues(TableProcessor values) {
    set_coordinate_values(std::move(values));
}

void PolynomialPathFitter::constructProperties() {
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

void PolynomialPathFitter::runFittingPipeline() {

    // Process the inputs.
    // -------------------
    // Load the model.
    Model model = m_model;
    //    Model model = get_model().process(getDocumentDirectory());
    model.initSystem();

    // Load the coordinate values.
    TableProcessor tableProcessor = get_coordinate_values();
    tableProcessor.append(TabOpUseAbsoluteStateNames());
    tableProcessor.append(TabOpAppendCoupledCoordinateValues());
    TimeSeriesTable values = tableProcessor.processAndConvertToRadians(
            getDocumentDirectory(), model);

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

    // Coordinate sampling bounds.
    // ---------------------------
    // Set the default bounds for all coordinates.
    SimTK::Vec2 defaultBounds = get_default_coordinate_sampling_bounds();
    m_coordinateBoundsMap.reserve(model.getNumCoordinates());
    m_coordinateRangeMap.reserve(model.getNumCoordinates());
    for (const auto& coordinate : model.getComponentList<Coordinate>()) {
        std::string valuePath = fmt::format("{}/value",
                coordinate.getAbsolutePathString());
        m_coordinateBoundsMap.insert({valuePath, defaultBounds});

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
        std::string valuePath = fmt::format("{}/value", coordinatePath);
        m_coordinateBoundsMap[valuePath] = bounds;
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
    TimeSeriesTable valuesSampled = sampleCoordinateValues(values);

    // Recompute the coupled coordinate values.
    auto tableProcessorSampled = TableProcessor(valuesSampled) |
                                 TabOpAppendCoupledCoordinateValues();
    valuesSampled = tableProcessor.process(&model);

    STOFileAdapter::write(valuesSampled, "coordinate_values_sampled.sto");

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

TimeSeriesTable PolynomialPathFitter::sampleCoordinateValues(
        const TimeSeriesTable& values) {
    // Create a Latin hypercube design to sample the coordinate values.
    LatinHypercubeDesign lhs;
    lhs.setNumSamples(get_num_samples_per_frame());
    lhs.setNumVariables((int)values.getNumColumns());

    // Store the coordinate bounds and ranges in vectors that we can pass to the
    // helper function below.
    std::vector<SimTK::Vec2> boundsVec;
    boundsVec.reserve(values.getNumColumns());
    std::vector<SimTK::Vec2> rangeVec;
    rangeVec.reserve(values.getNumColumns());
    for (const auto& label : values.getColumnLabels()) {
        boundsVec.push_back(m_coordinateBoundsMap[label]);
        rangeVec.push_back(m_coordinateRangeMap[label]);
    }

    // Helper function for sampling the coordinate values between two time
    // indexes.
    auto sampleCoordinateValuesSubset = [lhs, boundsVec, rangeVec](
                        std::vector<int>::iterator begin_iter,
                        std::vector<int>::iterator end_iter,
                        const SimTK::Matrix& valuesBlock) -> SimTK::Matrix {

        SimTK::Matrix results(
                lhs.getNumSamples()*(int)std::distance(begin_iter, end_iter),
                (int)valuesBlock.ncol());
        for (auto it = begin_iter; it != end_iter; ++it) {
            // Generate the design and shift between [-1, 1].
            SimTK::Matrix design = lhs.generateStochasticEvolutionaryDesign(5);
            design -= 0.5;
            design *= 2.0;

            // Linearly transform the design to the specified bounds for each
            // coordinate.
            for (int icol = 0; icol < design.ncol(); ++icol) {
                double slope = 0.5*(boundsVec[icol][1] - boundsVec[icol][0]);
                SimTK::Vector candidateCol = slope*(design.col(icol) + 1.0) +
                                             boundsVec[icol][0] + valuesBlock(*it, icol);

                // Check that all elements in the column are within the range of
                // motion. If not, move the element inside the range of motion
                // that is closest to the original value.
                for (double& elt : candidateCol) {
                    if (elt < rangeVec[icol][0]) {
                        elt = rangeVec[icol][0];
                    } else if (elt > rangeVec[icol][1]) {
                        elt = rangeVec[icol][1];
                    }
                }

                // Save the (potentially modified) column.
                design.updCol(icol) = candidateCol;
            }

            // Store the results.
            results.updBlock(
                    (int)std::distance(begin_iter, it)*lhs.getNumSamples(),
                    0, lhs.getNumSamples(), (int)valuesBlock.ncol()) = design;
        }

        return results;
    };

    // Divide the sampling across multiple threads.
    std::vector<int> timeIndexes(values.getNumRows());
    std::iota(timeIndexes.begin(), timeIndexes.end(), 0);
    int stride = static_cast<int>(
            std::floor(values.getNumRows() / get_parallel()));
    std::vector<std::future<SimTK::Matrix>> futures;
    int offset = 0;
    for (int ithread = 0; ithread < get_parallel(); ++ithread) {
        auto begin_iter = timeIndexes.begin() + offset;
        auto end_iter = (ithread == get_parallel()-1) ?
                                                        timeIndexes.end() :
                                                        timeIndexes.begin() + offset + stride;
        SimTK::Matrix valuesBlock = values.getMatrixBlock(*begin_iter, 0,
                (int)std::distance(begin_iter, end_iter),
                values.getNumColumns());
        futures.push_back(std::async(std::launch::async,
                sampleCoordinateValuesSubset,
                begin_iter, end_iter, valuesBlock));
        offset += stride;
    }

    // Wait for threads to finish and collect the results.
    std::vector<SimTK::Matrix> outputs(get_parallel());
    for (int i = 0; i < get_parallel(); ++i) {
        outputs[i] = futures[i].get();
    }

    // Assemble the results into one TimeSeriesTable.
    int timeIdx = 0;
    const auto& times = values.getIndependentColumn();
    double dt = (times[1] - times[0]) / (get_num_samples_per_frame() + 1);
    TimeSeriesTable valuesSampled;
    for (int i = 0; i < get_parallel(); ++i) {
        // Append the original values.
        valuesSampled.appendRow(times[timeIdx], values.getRowAtIndex(timeIdx));
        // Append the sampled values.
        for (int irow = 0; irow < outputs[i].nrow(); ++irow) {
            valuesSampled.appendRow(times[timeIdx] + (irow + 1)*dt,
                    outputs[i].row(irow));
        }
    }
    valuesSampled.addTableMetaData<std::string>("inDegrees", "no");
    valuesSampled.setColumnLabels(values.getColumnLabels());

    return valuesSampled;
}

void PolynomialPathFitter::computePathLengthsAndMomentArms(
        Model model,
        const TimeSeriesTable& coordinateValues,
        TimeSeriesTable& pathLengths,
        TimeSeriesTable& momentArms,
        std::map<std::string, std::vector<std::string>>& momentArmMap) {

    // Create a StatesTrajectory from the coordinate values.
    SimTK::State state = model.initSystem();
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

double PolynomialPathFitter::fitFunctionBasedPathCoefficients(
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