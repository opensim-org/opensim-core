#ifndef OPENSIM_POLYNOMIALPATHFITTER_H
#define OPENSIM_POLYNOMIALPATHFITTER_H
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

#include <OpenSim/Simulation/TableProcessor.h>
#include <OpenSim/Actuators/ModelProcessor.h>
#include <OpenSim/Simulation/Model/FunctionBasedPath.h>

namespace OpenSim {

/**
 * A helper class for specifying the minimum and maximum bounds for the
 * coordinate at `coordinate_path` during path fitting.
 *
 * The bounds are specified as a `SimTK::Vec2` in the property `bounds`, where
 * the first element is the minimum bound and the second element is the maximum
 * bound.
 */
class OSIMACTUATORS_API PolynomialPathFitterBounds : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(PolynomialPathFitterBounds, Object);

public:
    OpenSim_DECLARE_PROPERTY(coordinate_path, std::string,
            "The path to the coordinate in the model that is bounded during "
            "path fitting.")
    OpenSim_DECLARE_PROPERTY(bounds, SimTK::Vec2,
            "The bounds for the coordinate. The first element is the minimum "
            "bound and the second element is the maximum bound.")
    PolynomialPathFitterBounds();
    PolynomialPathFitterBounds(
            const std::string& coordinatePath, const SimTK::Vec2& bounds);
private:
    void constructProperties();
};

/**
 * A utility class for fitting a set of `FunctionBasedPath`s to existing
 * geometry-path in an OpenSim model using `MultivariatePolynomialFunction`s.
 *
 * The primary inputs to this class include a model containing path objects
 * derived from `AbstractGeometryPath` (e.g., `GeometryPath`) and a reference
 * trajectory containing coordinate values for all `Coordinate`s in the model.
 * The path fitting process samples coordinate values around the reference
 * trajectory, computes path lengths and moment arms from the geometry-based
 * paths in the model, and fits polynomial coefficients for
 * `MultivariatePolynomialFunction` objects based on the path length and moment
 * arm samples. The fitted paths are written to an XML file, along with the
 * modified coordinate values, sampled coordinate values, path lengths, and
 * moment arms for both the original and fitted paths.
 *
 * @note Each file name is prefixed with the name of the model, and the
 *       directory to which the results are written can be specified using the
 *      `setOutputDirectory` method.
 *
 * Settings
 * --------
 * Various settings can be adjusted to control the path fitting process. The
 * `setMomentArmsThreshold` method determines whether or not a path depends on a
 * model coordinate. In other words, the absolute value the moment arm of a with
 * respect to a particular coordinate must be greater than this value to be
 * included during path fitting. The `setMinimumPolynomialOrder` and
 * `setMaximumPolynomialOrder` methods specify the minimum and maximum order of
 * the polynomial used to fit each path. The `setGlobalCoordinateSamplingBounds`
 * property specifies the global bounds (in degrees) that determine the minimum
 * and maximum coordinate values sampled at each time point. The method
 * `appendCoordinateSamplingBounds` can be used to override the global bounds
 * for a specific coordinate. The `setMomentArmTolerance` and
 * `setPathLengthTolerance` methods specify the tolerance on the
 * root-mean-square (RMS) error (in meters) between the moment arms and path
 * lengths computed from the original model paths and the fitted polynomial
 * paths. The `setNumSamplesPerFrame` method specifies the number of samples
 * taken per time frame in the coordinate values table used to fit each path.
 * The `setParallel` method specifies the number of threads used to parallelize
 * the path fitting process. The `setLatinHypercubeAlgorithm` method specifies
 * the Latin hypercube sampling algorithm used to sample coordinate values for
 * path fitting.
 *
 * The default settings are as follows:
 *
 *    - Moment arm threshold: 1e-3 meters
 *    - Minimum polynomial order: 2
 *    - Maximum polynomial order: 6
 *    - Global coordinate sampling bounds: [-10, 10] degrees
 *    - Moment arm tolerance: 1e-4 meters
 *    - Path length tolerance: 1e-4 meters
 *    - Number of samples per frame: 25
 *    - Number of threads: (# of available hardware threads) - 2
 *    - Latin hypercube sampling algorithm: "random"
 *
 * @note The default settings were chosen based on testing with a human
 *       lower-extremity model. Different settings may be required for other
 *       models with larger or smaller anatomical measures (e.g., dinosaur
 *       models).
 *
 * Usage
 * -----
 * The most basic usage of `PolynomialPathFitter` requires the user to provide
 * a model and reference trajectory. The model should contain at least one path
 * object derived from `AbstractGeometryPath` and should not contain any
 * `FunctionBasedPath` objects. The reference trajectory must contain coordinate
 * values for all `Coordinate`s in the model:
 *
 * @code{.cpp}
 * PolynomialPathFitter fitter;
 * fitter.setModel(ModelProcessor("model.osim"));
 * fitter.setCoordinateValues(TableProcessor("values.sto"));
 * @endcode
 *
 * The additional settings can be adjusted using the various `set` methods
 * described above. For example, the global coordinate sampling bounds, bounds
 * for the coordinate at "/jointset/slider/position", and the number of samples
 * per frame can be set as follows:
 *
 * @code{.cpp}
 * fitter.setGlobalCoordinateSamplingBounds(SimTK::Vec2(-20.0, 20.0));
 * fitter.appendCoordinateSamplingBounds(
 *         "/jointset/slider/position", SimTK::Vec2(-5.0, 15.0));
 * fitter.setNumSamplesPerFrame(50);
 * @endcode
 *
 * The path fitting process can be run using the `run()` method:
 *
 * @code{.cpp}
 * fitter.run();
 * @endcode
 *
 * Recommendations
 * ---------------
 * Information from each step of the path fitting process is logged to the
 * console, provided that you have set the OpenSim::Logger to level "info" or
 * greater. Warnings are printed if the number of samples is likely insufficient
 * for the fitting process, or if the fit for a particular path did not meet the
 * specified tolerances.
 *
 * It is highly recommended to use the files printed to the output directory to
 * evaluate the quality of the fitted paths (see `setOutputDirectory()` for more
 * details). Depending on the quality of the original model, it may not be
 * possible to achieve a good fit for all paths (e.g., due to kinks or other
 * discontinuities in the path). Finally, the fitted paths should only be used
 * in simulations for which the coordinate values represent the expected range
 * of motion for the model. If you are unsure if a simulation you have created
 * with the fitted paths is valid, you can use the `evaluateFunctionBasedPaths`
 * static method to compare the fitted paths to the original model paths given a
 * new kinematic trajectory.
 *
 * @note The `evaluateFunctionBasedPaths` method can be used independently from
 *       the rest of this class, and does not require the `FunctionBasedPath`s
 *       in the model to use `MultivariatePolynomialFunction`s.
 */
class OSIMACTUATORS_API PolynomialPathFitter : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(PolynomialPathFitter, Object);

public:
    // CONSTRUCTION AND DESTRUCTION
    PolynomialPathFitter();
    ~PolynomialPathFitter() noexcept override;
    PolynomialPathFitter(const PolynomialPathFitter&);
    PolynomialPathFitter(PolynomialPathFitter&&);
    PolynomialPathFitter& operator=(const PolynomialPathFitter&);
    PolynomialPathFitter& operator=(PolynomialPathFitter&&);

    // MAIN INPUTS
    /**
     * The model containing geometry-based path objects to which
     * polynomial-based path objects will be fitted.
     *
     * The model should be provided using a `ModelProcessor` object. We expect
     * the model to contain at least one path object derived from
     * `AbstractGeometryPath` and does not already contain any
     * `FunctionBasedPath` objects. The bounds for clamped coordinates are
     * obeyed during the fitting process. Locked coordinates are unlocked if
     * data is provided for them, or replaced with WeldJoints if no data is
     * provided for them.
     */
    void setModel(ModelProcessor model);

    /**
     * The reference trajectory used to sample coordinate values for path
     * fitting.
     *
     * The reference trajectory should be provided using a `TableProcessor`
     * object. The reference trajectory must contain coordinate values for all
     * `Coordinate`s in the model. We assumed that the coordinate values meet
     * all the kinematic constraints in the model, except for
     * `CoordinateCouplerConstraint`s, since we automatically update the
     * coordinate trajectory to satisfy these constraints. The `TimeSeriesTable`
     * must contain the "inDegrees" metadata flag; the coordinate values are
     * automatically converted to radians if this flag is set to "yes".
     */
    void setCoordinateValues(TableProcessor coordinateValues);

    // RUN PATH FITTING
    /**
     * Run the path fitting process.
     *
     * The path fitting process consists of the following steps:
     *
     *     1. Load the model and reference coordinate values trajectory. The
     *        coordinate values table is modified to update the column labels
     *        based on the model coordinate paths, to update any coordinates
     *        dependent on `CoordinateCouplerConstraint`s, and to convert the
     *        coordinate values to radians if the "inDegrees" metadata flag is
     *        set to "yes".
     *
     *     2. Set sampling bounds for coordinates based on the global and
     *        coordinate-specific bounds properties.
     *
     *     3. Verify that the remaining user settings are valid.
     *
     *     4. Sample coordinate values around the reference trajectory using
     *        Latin hypercube sampling. The sampling is defined based on the
     *        coordinate bounds and range maps, the number of samples per frame,
     *        and the Latin hypercube sampling algorithm.
     *
     *     5. Compute path lengths and moment arms from the geometry-based paths
     *        in the input model.
     *
     *     6. Filter out bad coordinate samples and populate a map containing
     *        the coordinates that path is dependent on.
     *
     *     7. Fit the polynomial coefficients by finding a least-squares fit
     *        between the path lengths and moment arms computed from the
     *        geometry-based paths and the path lengths and moment arms
     *        computed from the fitted polynomial-based paths.
     *
     *     8. Print out a summary of the path fitting results, including
     *        information about the fitted polynomial functions and
     *        root-mean-square (RMS) errors between the original and fitted
     *        paths.
     *
     *     9. Write the fitted paths, modified coordinate values, sampled
     *        coordinate values, path lengths, and moment arms to files.
     *
     * @note Steps 4, 5, and 7 are parallelized using the number of threads
     *       specified via the `setParallel()` method.
     */
    void run();

    // SETTINGS
    /**
     * The directory to which the path fitting results are written.
     *
     * If the path fitting is successful, the fitted paths are written as a
     * `Set` of `FunctionBasedPath` objects (with path length functions defined
     * using `MultivariatePolynomialFunction` objects) to an XML file. Files
     * containing the modified coordinate values, sampled coordinate values,
     * path lengths, and moment arms for both the original and fitted paths are
     * also written to the output directory.
     *
     * @note By default, results are written to the current working directory.
     */
    void setOutputDirectory(std::string directory);
    std::string getOutputDirectory() const;

    /**
     * The moment arm threshold value that determines whether or not a path
     * depends on a model coordinate. In other words, the moment arm of a path
     * with respect to a particular coordinate must be greater than this value
     * to be included during path fitting.
     *
     * @note The default moment arm threshold is set to 1e-3 meters.
     */
    void setMomentArmThreshold(double threshold);
    /// @copydoc setMomentArmThreshold()
    double getMomentArmThreshold() const;

    /**
     * The minimum order of the polynomial used to fit each path. The order of
     * a polynomial is the highest power of the independent variable(s) in the
     * polynomial.
     *
     * @note The default minimum polynomial order is set to 2.
     */
    void setMinimumPolynomialOrder(int order);
    /// @copydoc setMinimumPolynomialOrder()
    int getMinimumPolynomialOrder() const;

    /**
     * The maximum order of the polynomial used to fit each path. The order of
     * a polynomial is the highest power of the independent variable(s) in the
     * polynomial.
     *
     * @note The default maximum polynomial order is set to 6.
     */
    void setMaximumPolynomialOrder(int order);
    /// @copydoc setMaximumPolynomialOrder()
    int getMaximumPolynomialOrder() const;

    /**
     * The global bounds that determine the minimum and maximum coordinate value
     * samples at each time point.
     *
     * The bounds are specified as a `SimTK::Vec2`, where the first element is
     * the minimum bound and the second element is the maximum bound. Rotational
     * coordinates are in degrees; translational coordinates in meters. The
     * maximum sample value at a particular time point is the nominal coordinate
     * value plus the maximum bound, and the minimum sample value is the
     * nominal coordinate value minus the minimum bound.
     *
     * @note The default global bounds are set to [-10, 10] degrees/meters.
     * If you have a model with paths that cross translational joints, you may
     * to specify smaller bounds for the translational coordinates (see
     * `appendCoordinateSamplingBounds()`).
     *
     * @note To override the default global bounds for a specific coordinate,
     *       use the `appendCoordinateSamplingBounds()` method.
     */
    void setGlobalCoordinateSamplingBounds(SimTK::Vec2 bounds);
    /// @copydoc setGlobalCoordinateSamplingBounds()
    SimTK::Vec2 getGlobalCoordinateSamplingBounds() const;

    /**
     * The bounds (in degrees) that determine the minimum and maximum coordinate
     * value samples at each time point for the coordinate at `coordinatePath`.
     *
     * The bounds are specified as a `SimTK::Vec2`, where the first element is
     * the minimum bound and the second element is the maximum bound. The
     * maximum sample value at a particular time point is the nominal coordinate
     * value plus the maximum bound, and the minimum sample value is the
     * nominal coordinate value minus the minimum bound. This overrides the
     * global bounds set by `setGlobalCoordinateSamplingBounds()` for this
     * coordinate.
     */
     void appendCoordinateSamplingBounds(
            const std::string& coordinatePath, const SimTK::Vec2& bounds);

    /**
     * The tolerance on the root-mean-square (RMS) error (in meters) between the
     * moment arms computed from an original model path and a fitted
     * polynomial-based path, which is used to determine the order of the
     * polynomial used in the fitted path.
     *
     * The moment arm RMS error must be less than the tolerance for the
     * polynomial order to be accepted. If the RMS error is greater than the
     * tolerance, the polynomial order is increased by one and the path is
     * refitted. This process is repeated until the RMS error is less than the
     * tolerance or the maximum polynomial order is reached.
     *
     * @note The default moment arm tolerance is set to 1e-4 meters.
     * @note The path length RMS error must also be less than the path length
     *       tolerance for the polynomial order to be accepted (see
     *       `setPathLengthTolerance`).
     */
    void setMomentArmTolerance(double tolerance);
    /// @copydoc setMomentArmTolerance()
    double getMomentArmTolerance() const;

    /**
     * The tolerance on the root-mean-square (RMS) error (in meters) between the
     * path lengths computed from an original model path and a fitted
     * polynomial-based path, which is used to determine the order of the
     * polynomial used in the fitted path.
     *
     * The path length RMS error must be less than the tolerance for the
     * polynomial order to be accepted. If the RMS error is greater than the
     * tolerance, the polynomial order is increased by one and the path is
     * refitted. This process is repeated until the RMS error is less than the
     * tolerance or the maximum polynomial order is reached.
     *
     * @note The default path length tolerance is set to 1e-4 meters.
     * @note The moment arm RMS error must also be less than the moment arm
     *       tolerance for the polynomial order to be accepted (see
     *      `setMomentArmTolerance`).
     */
    void setPathLengthTolerance(double tolerance);
    /// @copydoc setPathLengthTolerance()
    double getPathLengthTolerance() const;

    /**
     * The number of samples taken per time frame in the coordinate values table
     * used to fit each path.
     *
     * @note The default number of samples per frame is set to 25.
     */
    void setNumSamplesPerFrame(int numSamples);
    /// @copydoc setNumSamplesPerFrame()
    int getNumSamplesPerFrame() const;

    /**
     * The number of threads used to parallelize the path fitting process.
     *
     * This setting is used to divide the coordinate sampling, path length and
     * moment arm computations, and path fitting across multiple threads. The
     * number of threads must be greater than zero.
     *
     * @note The default number of threads is set to two fewer than the number
     *       of available hardware threads.
     */
    void setNumParallelThreads(int numThreads);
    /// @copydoc setParallel()
    int getNumParallelThreads() const;

    /**
     * The Latin hypercube sampling algorithm used to sample coordinate values
     * for path fitting.
     *
     * The Latin hypercube sampling algorithm is used to sample coordinate
     * values for path fitting. The algorithm can be set to either "random" or
     * "ESEA", which stands for the enhanced stochastic evolutionary algorithm
     * developed by Jin et al. 2005 (see class `LatinHypercubeDesign` for more
     * details). The "random" algorithm is used by default, and "ESEA" can be
     * used to improve the quality of the sampling at the expense of higher
     * computational cost. For most applications, the "random" algorithm is
     * likely sufficient.
     */
    void setLatinHypercubeAlgorithm(std::string algorithm);
    /// @copydoc setLatinHypercubeAlgorithm()
    std::string getLatinHypercubeAlgorithm() const;

    // HELPER FUNCTIONS
    /**
     * Print out a summary of the path fitting results, including information
     * about the fitted polynomial functions and root-mean-square (RMS) errors
     * between the original and fitted paths.
     *
     * The `trajectory` argument is a `TableProcessor` object containing the
     * simulation trajectory, specifically the set of coordinate values, used to
     * compute path lengths and moment arms. The `polynomialPathsFile` argument
     * is the path to an XML file containing the set of `FunctionBasedPath`s
     * fitted to the geometry-based paths in `model`. These paths can be defined
     * by `MultivariatePolynomialFunction`s generated by this class or any other
     * `Function` objects that approximate the original model paths.
     */
    static void evaluateFunctionBasedPaths(Model model,
            TableProcessor trajectory,
            const std::string& functionBasedPathsFile,
            double pathLengthTolerance = 1e-4,
            double momentArmTolerance = 1e-4);

private:
    // PROPERTIES
    OpenSim_DECLARE_PROPERTY(model, ModelProcessor,
            "The model containing geometry-based path objects to which "
            "polynomial-based path objects will be fitted.");
    OpenSim_DECLARE_PROPERTY(coordinate_values, TableProcessor,
            "The reference trajectory used to sample coordinate values for "
            "path fitting.");
    OpenSim_DECLARE_PROPERTY(output_directory, std::string,
            "The directory to which the path fitting results are written.");
    OpenSim_DECLARE_PROPERTY(moment_arm_threshold, double,
            "The moment arm threshold value that determines whether or not a "
            "path depends on a model coordinate. In other words, the moment "
            "arm of a path with respect to a coordinate must be greater than "
            "this value to be included during path fitting.");
    OpenSim_DECLARE_PROPERTY(minimum_polynomial_order, int,
            "The minimum order of the polynomial used to fit each path. The "
            "order of a polynomial is the highest power of the independent "
            "variable(s) in the polynomial.");
    OpenSim_DECLARE_PROPERTY(maximum_polynomial_order, int,
            "The maximum order of the polynomial used to fit each path. The "
            "order of a polynomial is the highest power of the independent "
            "variable(s) in the polynomial.");
    OpenSim_DECLARE_PROPERTY(global_coordinate_sampling_bounds, SimTK::Vec2,
            "The global bounds (in degrees) that determine the minimum and "
            "maximum coordinate value samples at each time point.");
    OpenSim_DECLARE_LIST_PROPERTY(
            coordinate_sampling_bounds, PolynomialPathFitterBounds,
            "The bounds (in degrees) that determine the minimum and maximum "
            "coordinate value samples at each time point for specific "
            "coordinates. These bounds override the default coordinate "
            "sampling bounds.");
    OpenSim_DECLARE_PROPERTY(moment_arm_tolerance, double,
            "The tolerance on the root-mean-square (RMS) error (in meters) "
            "between the moment arms computed from an original model path and "
            "a fitted polynomial-based path, which is used to determine the "
            "order of the polynomial used in the fitted path (default: 1e-4).");
    OpenSim_DECLARE_PROPERTY(path_length_tolerance, double,
            "The tolerance on the root-mean-square (RMS) error (in meters) "
            "between the path lengths computed from an original model path and "
            "a fitted polynomial-based path, which is used to determine the "
            "order of the polynomial used in the fitted path (default: 1e-4).");
    OpenSim_DECLARE_PROPERTY(num_samples_per_frame, int,
            "The number of samples taken per time frame in the coordinate "
            "values table used to fit each path (default: 25).");
    OpenSim_DECLARE_PROPERTY(num_parallel_threads, int,
            "The number of threads used to parallelize the path fitting "
            "process (default: two fewer than the number of available "
            "hardware threads).");
    OpenSim_DECLARE_PROPERTY(
            latin_hypercube_algorithm, std::string,
            "The Latin hypercube sampling algorithm used to sample coordinate "
            "values for path fitting (default: 'random').");

    void constructProperties();

    // PATH FITTING PIPELINE
    /**
     * Type alias for the moment arm map. The keys are the paths in the model
     * and the values are vectors containing the names of coordinates on which
     * the paths depend.
     */
    typedef std::unordered_map<std::string, std::vector<std::string>>
            MomentArmMap;

    /**
     * Helper function to load the reference coordinate values trajectory and
     * validate the model. The coordinate values table is modified to update the
     * column labels based on the model coordinate paths, to update any
     * coordinates dependent on `CoordinateCouplerConstraint`s, and to convert
     * the coordinate values to radians if the "inDegrees" metadata flag is set
     * to "yes".
     */
    static TimeSeriesTable loadCoordinateValuesAndValidateModel(
            const std::string& documentDir,
            TableProcessor tableProcessor,
            Model& model);

    /**
     * Helper function to sample coordinate values around the user-provided
     * coordinate trajectory contained in the `values` input table. The
     * sampling is defined based on the coordinate bounds and range maps,
     * the number of samples per frame, and the Latin hypercube sampling
     * algorithm.
     */
    TimeSeriesTable sampleCoordinateValues(const TimeSeriesTable& values);

    /**
     * Helper function to compute path lengths and moment arms for the
     * geometry-based paths in the model. The path lengths and moment arms
     * are computed using the coordinate values in the `coordinateValues`
     * table. The `numThreads` argument specifies the number of threads used
     * to parallelize the computations.
     */
    static void computePathLengthsAndMomentArms(const Model& model,
            const TimeSeriesTable& coordinateValues, int numThreads,
            TimeSeriesTable& pathLengths, TimeSeriesTable& momentArms);

    /**
     * Helper function to filter out bad coordinate value samples and determine
     * which coordinates each path is dependent on. Bad samples are defined as
     * coordinate values that produce path length and/or moment are values that
     * deviate by a set number of standard deviations away from the nominal
     * trajectories. The `momentArmMap` argument is a map containing the
     * coordinates each path is dependent on. Columns in the `momentArms` table
     * are removed if they do not correspond to entries in the `momentArmMap`.
     */
    void filterSampledData(const Model& model,
            TimeSeriesTable& coordinateValues, TimeSeriesTable& pathLengths,
            TimeSeriesTable& momentArms, MomentArmMap& momentArmMap);

    /**
     * Helper function to fit polynomial coefficients to the path lengths and
     * moment arms computed from the geometry-based paths in the model. The
     * `coordinateValues`, `pathLengths`, and `momentArms` table arguments are
     * the result of previous model sampling and data filtering steps. The
     * `momentArmMap` argument is a map containing the coordinates each path is
     * dependent on, which determines the number of independent coordinates
     * that each `MultivariatePolynomialFunction` contains to approximate the
     * original path.
     */
    Set<FunctionBasedPath> fitPolynomialCoefficients(const Model& model,
            const TimeSeriesTable& coordinateValues,
            const TimeSeriesTable& pathLengths,
            const TimeSeriesTable& momentArms,
            const MomentArmMap& momentArmMap);

    // HELPER FUNCTIONS
    /**
     * Get the (canonicalized) absolute directory containing the file from
     * which this tool was loaded. If the `FunctionBasedPathFitter` was not
     * loaded from a file, this returns an empty string.
     */
    std::string getDocumentDirectory() const;

    /**
     * Remove columns from the `momentArms` table that do not correspond to
     * entries in the `momentArmMap`.
     */
    static void removeMomentArmColumns(TimeSeriesTable& momentArms,
            const MomentArmMap& momentArmMap);

    /**
     * Get the RMS errors between two sets of path lengths and moment arms
     * computed from a model with FunctionBasedPaths and the original model. The
     * `modelFitted` argument must be the model with the FunctionBasedPaths.
     */
    static void computeFittingErrors(const Model& modelFitted,
            const TimeSeriesTable& pathLengths,
            const TimeSeriesTable& momentArms,
            const TimeSeriesTable& pathLengthsFitted,
            const TimeSeriesTable& momentArmsFitted,
            double pathLengthTolerance, double momentArmTolerance);

    // MEMBER VARIABLES
    std::unordered_map<std::string, SimTK::Vec2> m_coordinateBoundsMap;
    std::unordered_map<std::string, SimTK::Vec2> m_coordinateRangeMap;
    bool m_useStochasticEvolutionaryLHS = false;
};

} // namespace OpenSim

#endif // OPENSIM_POLYNOMIALPATHFITTER_H
