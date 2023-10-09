#ifndef OPENSIM_FUNCTIONBASEDPATH_H
#define OPENSIM_FUNCTIONBASEDPATH_H
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  FunctionBasedPath.h                         *
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

#include "OpenSim/Common/Function.h"
#include "OpenSim/Common/TimeSeriesTable.h"
#include "OpenSim/Simulation/Model/AbstractPath.h"
#include "OpenSim/Simulation/TableProcessor.h"
#include "OpenSim/Actuators/ModelProcessor.h"

namespace OpenSim {

//=============================================================================
//                           FUNCTION-BASED PATH
//=============================================================================
/**
 * A concrete class representing a path for muscles, ligaments, etc., based on
 * `Function` objects. This class can be used when efficient computations of
 * path length, lengthening speed, and moment arms are a priority (e.g., a
 * direct collocation problem).
 *
 * Each instance of this class requires both a list of model coordinates that
 * the path is dependent on and a function, \f$ l(q) \f$, that computes the
 * length of the path as a function of the coordinate values, \f$ q \f$.
 * Optionally, functions can be provided to compute the moment arms of the path
 * as a function of the coordinate values and a function that computes the speed
 * of the path as a function of the coordinate values and speeds.
 *
 * If the moment arm and/or speed functions are not provided, they will be
 * computed from the length function based on van den Bogert et al. (2013) and
 * Meyer et al. (2017):
 *
 * \f[
 * r_i = -\frac{\partial l}{\partial q_i} \quad \forall q_i \in Q
 * \f]
 *
 * \f[
 * \dot{l} = \frac{dl}{dt}
 *         = \sum_i \frac{\partial l}{\partial q_i} \frac{dq_i}{dt}
 *         = \sum_i -r_i \dot{q}_i
 * \f]
 *
 * Where,
 * - \f$ r_i \f$: the moment arm of the path with respect to coordinate
 * \f$q_i\f$.
 * - \f$ l \f$: the length of the path.
 * - \f$ \dot{l} \f$: the lengthening speed of the path.
 * - \f$ q_i \f$: the i-th coordinate value.
 * - \f$ \dot{q}_i \f$: the i-th coordinate value derivative.
 * - \f$ Q \f$: the set of all coordinates.
 *
 * @note The moment arm expression above assumes that all constraints in the
 * model are workless (i.e., no `MovingPathPoint`s). Other path types (e.g.,
 * `GeometryPath`) compute moment arms based on the generalized force applied to
 * joint given a unit force along the path, which does not rely on this
 * assumption (see `MomentArmSolver`). Please keep this in mind when providing
 * the path functions. See Sherman et al. (2013) for more details.
 *
 * @note \f$ \dot{q}_i \f$ is usually, but not necessarily, the same as the
 * generalized speed of the coordinate, \f$ u_i \f$ (e.g., see `BallJoint`).
 * Please keep this in mind when providing a speed function. See Sherman et al.
 * (2013) for more details.
 *
 * The length function and (if provided) the moment arm functions must have the
 * same number of arguments as the number of coordinates, where the order of the
 * arguments matches the order in the `coordinates` property. Each moment arm
 * function corresponds to a single coordinate, and the order of the functions
 * in the `moment_arm_functions` property must match the order in `coordinates`.
 * The speed function (if provided) must have twice as many arguments as the
 * number of coordinates, where the first half of the arguments are the
 * coordinate values and the second half are the coordinate speeds. Again, the
 * order of the value and speed arguments must match the order in the
 * `coordinates` property.
 *
 * The forces applied to the model by the path are computed by multiplying the
 * tension in the path by the moment arms. Therefore, this class only applies
 * mobility (i.e., generalized) forces to the model.
 *
 * References
 * ----------
 * - [1] Meyer AJ, Patten C, Fregly BJ (2017) "Lower extremity EMG-driven
 *       modeling of walking with automated adjustment of musculoskeletal
 *       geometry." PLoS ONE 12(7): e0179698.
 *       https://doi.org/10.1371/journal.pone.0179698
 * - [2] van den Bogert, A.J., Geijtenbeek, T., Even-Zohar, O. et al. (2013) "A
 *       real-time system for biomechanical analysis of human movement and
 *       muscle function." Med Biol Eng Comput 51, 1069–1077 (2013).
 *       https://doi.org/10.1007/s11517-013-1076-z
 * - [3] Sherman MA, Seth A, Delp SL. (2013) "What is a Moment Arm? Calculating
 *       Muscle Effectiveness in Biomechanical Models Using Generalized
 *       Coordinates." ASME. International Design Engineering Technical
 *       Conferences and Computers and Information in Engineering Conference,
 *       Volume 7B: 9th International Conference on Multibody Systems,
 *       Nonlinear Dynamics, and Control.
 */
class OSIMSIMULATION_API FunctionBasedPath : public AbstractPath {
OpenSim_DECLARE_CONCRETE_OBJECT(FunctionBasedPath, AbstractPath);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(coordinate_paths, std::string,
            "The list of paths to the model coordinates that are used as "
            "arguments to the length, lengthening speed, and moment arm "
            "functions. You must provide a least one coordinate path, and "
            "the order of the paths must match the order of the arguments "
            "passed to `length_function` (and `moment_arm_functions` and "
            "`lengthening_speed_function`, if provided).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(length_function, Function,
            "(Required) The OpenSim::Function object that computes the length "
            "of the path as a function of the coordinate values. The function "
            "arguments must match the order in `coordinate_paths`.");
    OpenSim_DECLARE_LIST_PROPERTY(moment_arm_functions, Function,
            "(Optional) The list of OpenSim::Function objects that compute the "
            "moment arms of the path as a function of the coordinate values. "
            "The function arguments must match the order in "
            "`coordinate_paths`.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(lengthening_speed_function, Function,
            "(Optional) The OpenSim::Function object that computes the speed "
            "of the path as a function of the coordinate values and speeds. "
            "The function arguments must be the coordinate values followed by "
            "coordinate speeds, both matching the order in "
            "`coordinate_paths`.");

//=============================================================================
// METHODS
//=============================================================================

    // CONSTRUCTION AND DESTRUCTION
    FunctionBasedPath();

    // GET AND SET
    /// Set the list of paths to the model coordinate that are used as arguments
    /// to the length and, if provided, lengthening speed and moment functions.
    /// The order of the coordinates must match the order of the function
    /// arguments.
    void setCoordinatePaths(const std::vector<std::string>& coordinatePaths);
    void appendCoordinatePath(const std::string& coordinatePath);
    std::vector<std::string> getCoordinatePaths() const;

    /// Set the function that computes the length of the path as a function of
    /// the coordinate values. The function must have the same number of
    /// arguments as the number of coordinates.
    void setLengthFunction(const Function& lengthFunction);
    const Function& getLengthFunction() const;

    /// Set the list of functions that compute the moment arms of the path as a
    /// function of the coordinate values. The order of the functions must match
    /// the order of the coordinates.
    void setMomentArmFunctions(const std::vector<Function>& momentArmFunctions);
    void appendMomentArmFunction(const Function& momentArmFunction);
    const Function& getMomentArmFunction(
            const std::string& coordinatePath) const;
    const SimTK::Vector& getMomentArms(const SimTK::State& s) const;

    /// Set the function that computes the speed of the path as a function of
    /// the coordinate values and speeds. The function must have the same number
    /// of arguments as the number of coordinate values and speeds.
    void setLengtheningSpeedFunction(const Function& speedFunction);
    const Function& getLengtheningSpeedFunction() const;

    // ABSTRACT PATH INTERFACE
    double getLength(const SimTK::State& s) const override;
    double getLengtheningSpeed(const SimTK::State& s) const override;
    /// @note This must check if the path is dependent on the provided
    /// coordinate, which requires a search through the list of coordinates.
    /// To retrieve the moment arms directly from the cache variable in the
    /// SimTK::State, use `getMomentArms()` instead.
    double computeMomentArm(const SimTK::State& s,
            const Coordinate& coord) const override;
    void addInEquivalentForces(const SimTK::State& state,
            const double& tension,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& mobilityForces) const override;
    bool isVisualPath() const override { return false; }

private:
    // MODEL COMPONENT INTERFACE
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    // CONVENIENCE METHODS
    void constructProperties();
    SimTK::Vector computeCoordinateValues(const SimTK::State& s) const;
    SimTK::Vector computeCoordinateDerivatives(const SimTK::State& s) const;
    void computeLength(const SimTK::State& s) const;
    void computeMomentArms(const SimTK::State& s) const;
    void computeLengtheningSpeed(const SimTK::State& s) const;

    // MEMBER VARIABLES
    std::vector<SimTK::ReferencePtr<const Coordinate>> _coordinates;
    std::unordered_map<std::string, int> _coordinateIndices;
    bool _computeMomentArms = false;
    bool _computeLengtheningSpeed = false;

    // CACHE VARIABLES
    mutable CacheVariable<double> _lengthCV;
    mutable CacheVariable<SimTK::Vector> _momentArmsCV;
    mutable CacheVariable<double> _lengtheningSpeedCV;
};

//=============================================================================
//                       FUNCTION-BASED PATH FITTER
//=============================================================================

/**
 * A helper class for storing coordinate bounds.
 */
class OSIMSIMULATION_API FunctionBasedPathFitterBounds : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(FunctionBasedPathFitterBounds, Object);
public:
    OpenSim_DECLARE_PROPERTY(coordinate_path, std::string, "TODO.")
    OpenSim_DECLARE_PROPERTY(bounds, SimTK::Vec2, "TODO.")
    FunctionBasedPathFitterBounds();
    FunctionBasedPathFitterBounds(
            const std::string& coordinatePath, const SimTK::Vec2& bounds);
private:
    void constructProperties();
};

/**
 * A class for fitting a set of `FunctionBasedPath`s to paths in an OpenSim
 * model.
 */
class OSIMSIMULATION_API FunctionBasedPathFitter : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(FunctionBasedPathFitter, Object);
public:
    // CONSTRUCTION AND DESTRUCTION
    FunctionBasedPathFitter();
    ~FunctionBasedPathFitter() noexcept override;
    FunctionBasedPathFitter(const FunctionBasedPathFitter&);
    FunctionBasedPathFitter(FunctionBasedPathFitter&&);
    FunctionBasedPathFitter& operator=(const FunctionBasedPathFitter&);
    FunctionBasedPathFitter& operator=(FunctionBasedPathFitter&&);

    // GET AND SET
    // TODO 1) explain locked and clamped coordinates (i.e., based on the
    //         property values)
    void setModel(ModelProcessor model) {
        set_model(std::move(model));
    }

    // TODO 1) assume that coordinate values satisfy kinematic constraints
    //         in the model (except for CoordinateCouplerConstraints)
    //      2) use absolute state names and convert to radians
    void setCoordinateValues(TableProcessor coordinateValues) {
        set_coordinate_values(std::move(coordinateValues));
    }

    void setMomentArmTolerance(double momentArmTolerance) {
        set_moment_arm_tolerance(momentArmTolerance);
    }

    /** Get the (canonicalized) absolute directory containing the file from
     * which this tool was loaded. If the `FunctionBasedPathFitter` was not
     * loaded from a file, this returns an empty string.
     */
    std::string getDocumentDirectory() const;


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
            coordinate_sampling_bounds, FunctionBasedPathFitterBounds,
            "TODO.");
    OpenSim_DECLARE_PROPERTY(num_samples_per_frame, int, "TODO.");

    void constructProperties();


    // FITTING PIPELINE FUNCTIONS
    void runFittingPipeline();


    void sampleAndAppendCoordinateValues(
            TimeSeriesTable& values);


    // TODO: 0) should any of the TODOs below be handled in this function?
    //          (probably not, just throw exceptions)
    //       1) how to handle locked coordinates?
    //       2) coordinate values must have number of columns equal to
    //          number of independent coordinates (Appends coupled coordinate
    //          values, if missing)
    //       3) Updates the coordinates table to use absolute state names
    void computePathLengthsAndMomentArms(
            Model model,
            const TimeSeriesTable& coordinateValues,
            TimeSeriesTable& pathLengths,
            TimeSeriesTable& momentArms,
            std::map<std::string, std::vector<std::string>>& momentArmMap);

    // TODO: 1) expects length and moment arm column names in specific format
    //       2) returns average RMS error
    double fitFunctionBasedPathCoefficients(
            Model model,
            const TimeSeriesTable& coordinateValues,
            const TimeSeriesTable& pathLengths,
            const TimeSeriesTable& momentArms,
            const std::map<std::string, std::vector<std::string>>& momentArmMap,
            std::string outputPath,
            const int minOrder = 2, const int maxOrder = 6);

    // MEMBER VARIABLES
    std::unordered_map<std::string, SimTK::Vec2> m_coordinateBoundsMap;


};

} // namespace OpenSim

#endif // OPENSIM_FUNCTIONBASEDPATH_H
