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
 * coordinate at `coordinate_path` during path fitting. The bounds are
 * specified as a `SimTK::Vec2` in the property `bounds`, where the first
 * element is the minimum bound and the second element is the maximum bound.
 */
class OSIMACTUATORS_API PolynomialPathFitterBounds : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(PolynomialPathFitterBounds, Object);

public:
    OpenSim_DECLARE_PROPERTY(coordinate_path, std::string,
            "The path to the bounded coordinate in the model.")
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
 * A class for fitting a set of `FunctionBasedPath`s to paths in an OpenSim
 * model.
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
    // TODO 1) explain locked and clamped coordinates (i.e., based on the
    //         property values)
    void setModel(ModelProcessor model);

    // TODO 1) assume that coordinate values satisfy kinematic constraints
    //         in the model (except for CoordinateCouplerConstraints)
    //      2) use absolute state names and convert to radians
    //      3) table must have the "inDegrees" flag
    void setCoordinateValues(TableProcessor coordinateValues);

    // RUN PATH FITTING
    void run();

    // SETTINGS
    /**
     * The moment arm threshold value that determines whether or not a path
     * depends on a model coordinate. In other words, the moment arm of a path
     * with respect to a particular coordinate must be greater than this value
     * to be included during path fitting.
     */
    void setMomentArmThreshold(double threshold);
    /// @copydoc setMomentArmThreshold()
    double getMomentArmThreshold() const;

    /**
     * The minimum order of the polynomial used to fit each path. The order of
     * a polynomial is the highest power of the independent variable(s) in the
     * polynomial.
     */
    void setMinimumPolynomialOrder(int order);
    /// @copydoc setMinimumPolynomialOrder()
    int getMinimumPolynomialOrder() const;

    /**
     * The maximum order of the polynomial used to fit each path. The order of
     * a polynomial is the highest power of the independent variable(s) in the
     * polynomial.
     */
    void setMaximumPolynomialOrder(int order);
    /// @copydoc setMaximumPolynomialOrder()
    int getMaximumPolynomialOrder() const;

    /**
     * The default bounds (in degrees) that determine the minimum and maximum
     * coordinate value samples at each time point.
     *
     * The bounds are specified as a `SimTK::Vec2`, where the first element is
     * the minimum bound and the second element is the maximum bound. The .
     * maximum sample value at a particular time point is the nominal coordinate
     * value plus the maximum bound, and the minimum sample value is the
     * nominal coordinate value minus the minimum bound.
     */
    void setDefaultCoordinateSamplingBounds(SimTK::Vec2 bounds);
    /// @copydoc setDefaultCoordinateSamplingBounds()
    SimTK::Vec2 getDefaultCoordinateSamplingBounds() const;

    /**
     * The bounds (in degrees) that determine the minimum and maximum coordinate
     * value samples at each time point for the coordinate at `coordinatePath`.
     *
     * The bounds are specified as a `SimTK::Vec2`, where the first element is
     * the minimum bound and the second element is the maximum bound. The .
     * maximum sample value at a particular time point is the nominal coordinate
     * value plus the maximum bound, and the minimum sample value is the
     * nominal coordinate value minus the minimum bound.
     *
     * @note This overrides the default coordinate sampling bounds for the
     *       coordinate at `coordinatePath`.
     */
     void appendCoordinateSamplingBounds(
            const std::string& coordinatePath, const SimTK::Vec2& bounds);

    void setMomentArmTolerance(double tolerance);
    double getMomentArmTolerance() const;

    void setPathLengthTolerance(double tolerance);
    double getPathLengthTolerance() const;

    void setNumSamplesPerFrame(int numSamples);
    int getNumSamplesPerFrame() const;

    void setParallel(int numThreads);
    int getParallel() const;

    void setLatinHypercubeAlgorithm(std::string algorithm);
    std::string getLatinHypercubeAlgorithm() const;

    void setOutputDirectory(std::string directory);
    std::string getOutputDirectory() const;

    // HELPER FUNCTIONS
    static void evaluateFittedPaths(Model model,
            TableProcessor trajectory,
            const std::string& functionBasedPathsFileName);

private:
    // PROPERTIES
    OpenSim_DECLARE_PROPERTY(model, ModelProcessor, "TODO.");
    OpenSim_DECLARE_PROPERTY(coordinate_values, TableProcessor, "TODO.");
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
    OpenSim_DECLARE_PROPERTY(default_coordinate_sampling_bounds, SimTK::Vec2,
            "The default bounds (in degrees) that determine the minimum and "
            "maximum coordinate value samples at each time point.");
    OpenSim_DECLARE_LIST_PROPERTY(
            coordinate_sampling_bounds, PolynomialPathFitterBounds,
            "The bounds (in degrees) that determine the minimum and maximum "
            "coordinate value samples at each time point for specific "
            "coordinates. These bounds override the default coordinate "
            "sampling bounds.");
    OpenSim_DECLARE_PROPERTY(moment_arm_tolerance, double, "TODO.");
    OpenSim_DECLARE_PROPERTY(path_length_tolerance, double, "TODO.");
    OpenSim_DECLARE_PROPERTY(num_samples_per_frame, int, "TODO.");
    OpenSim_DECLARE_PROPERTY(parallel, int, "TODO.");
    OpenSim_DECLARE_PROPERTY(
            latin_hypercube_algorithm, std::string, "TODO.");
    OpenSim_DECLARE_PROPERTY(output_directory, std::string, "TODO.");

    void constructProperties();

    // PATH FITTING PIPELINE
    typedef std::unordered_map<std::string, std::vector<std::string>>
            MomentArmMap;

    TimeSeriesTable sampleCoordinateValues(const TimeSeriesTable& values);

    static void computePathLengthsAndMomentArms(const Model& model,
            const TimeSeriesTable& coordinateValues, int numThreads,
            TimeSeriesTable& pathLengths, TimeSeriesTable& momentArms);

    void filterSampledData(const Model& model,
            TimeSeriesTable& coordinateValues, TimeSeriesTable& pathLengths,
            TimeSeriesTable& momentArms, MomentArmMap& momentArmMap);

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
            const TimeSeriesTable& momentArmsFitted);

    // MEMBER VARIABLES
    std::unordered_map<std::string, SimTK::Vec2> m_coordinateBoundsMap;
    std::unordered_map<std::string, SimTK::Vec2> m_coordinateRangeMap;
    bool m_useStochasticEvolutionaryLHS = false;
};

} // namespace OpenSim

#endif // OPENSIM_POLYNOMIALPATHFITTER_H
