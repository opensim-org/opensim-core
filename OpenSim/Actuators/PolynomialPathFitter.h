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

namespace OpenSim {

/**
 * A helper class for storing coordinate bounds.
 */
class OSIMACTUATORS_API PolynomialPathFitterBounds : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(PolynomialPathFitterBounds, Object);

public:
    OpenSim_DECLARE_PROPERTY(coordinate_path, std::string, "TODO.")
    OpenSim_DECLARE_PROPERTY(bounds, SimTK::Vec2, "TODO.")
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

    // GET AND SET
    // TODO 1) explain locked and clamped coordinates (i.e., based on the
    //         property values)
    void setModel(ModelProcessor model);

    // TODO 1) assume that coordinate values satisfy kinematic constraints
    //         in the model (except for CoordinateCouplerConstraints)
    //      2) use absolute state names and convert to radians
    void setCoordinateValues(TableProcessor coordinateValues);

    void setMomentArmTolerance(double momentArmTolerance) {
        set_moment_arm_tolerance(momentArmTolerance);
    }

    /** Get the (canonicalized) absolute directory containing the file from
     * which this tool was loaded. If the `FunctionBasedPathFitter` was not
     * loaded from a file, this returns an empty string.
     */
    std::string getDocumentDirectory() const;

    void runFittingPipeline();

private:
    // PROPERTIES
    OpenSim_DECLARE_PROPERTY(model, ModelProcessor, "TODO.");
    OpenSim_DECLARE_PROPERTY(coordinate_values, TableProcessor, "TODO.");
    OpenSim_DECLARE_PROPERTY(moment_arm_tolerance, double, "TODO.");
    OpenSim_DECLARE_PROPERTY(minimum_polynomial_order, int, "TODO.");
    OpenSim_DECLARE_PROPERTY(maximum_polynomial_order, int, "TODO.");
    OpenSim_DECLARE_PROPERTY(parallel, int, "TODO.");
    OpenSim_DECLARE_PROPERTY(
            default_coordinate_sampling_bounds, SimTK::Vec2, "TODO.");
    OpenSim_DECLARE_LIST_PROPERTY(
            coordinate_sampling_bounds, PolynomialPathFitterBounds, "TODO.");
    OpenSim_DECLARE_PROPERTY(num_samples_per_frame, int, "TODO.");

    void constructProperties();

    // FITTING PIPELINE FUNCTIONS

    TimeSeriesTable sampleCoordinateValues(const TimeSeriesTable& values);

    // TODO: 0) should any of the TODOs below be handled in this function?
    //          (probably not, just throw exceptions)
    //       1) how to handle locked coordinates?
    //       2) coordinate values must have number of columns equal to
    //          number of independent coordinates (Appends coupled coordinate
    //          values, if missing)
    //       3) Updates the coordinates table to use absolute state names
    void computePathLengthsAndMomentArms(Model model,
            const TimeSeriesTable& coordinateValues,
            TimeSeriesTable& pathLengths, TimeSeriesTable& momentArms,
            std::map<std::string, std::vector<std::string>>& momentArmMap);

    // TODO: 1) expects length and moment arm column names in specific format
    //       2) returns average RMS error
    double fitFunctionBasedPathCoefficients(Model model,
            const TimeSeriesTable& coordinateValues,
            const TimeSeriesTable& pathLengths,
            const TimeSeriesTable& momentArms,
            const std::map<std::string, std::vector<std::string>>& momentArmMap,
            std::string outputPath, const int minOrder = 2,
            const int maxOrder = 6);

    // MEMBER VARIABLES
    std::unordered_map<std::string, SimTK::Vec2> m_coordinateBoundsMap;
    std::unordered_map<std::string, SimTK::Vec2> m_coordinateRangeMap;
};

} // namespace OpenSim

#endif // OPENSIM_POLYNOMIALPATHFITTER_H
