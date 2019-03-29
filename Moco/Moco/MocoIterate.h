#ifndef MOCO_MOCOITERATE_H
#define MOCO_MOCOITERATE_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoIterate.h                                                *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "osimMocoDLL.h"

#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/StatesTrajectory.h>

namespace OpenSim {

class MocoProblem;
class MocoProblemRep;

/// This exception is thrown if you try to invoke most methods on MocoIterate
/// while the iterate is sealed.
class OSIMMOCO_API MocoIterateIsSealed : public Exception {
public:
    MocoIterateIsSealed(
            const std::string& file, size_t line, const std::string& func)
            : Exception(file, line, func) {
        addMessage("This iterate is sealed, to force you to acknowledge the "
                   "solver failed; call unseal() to gain access.");
    }
};

/** The values of the variables in an optimal control problem.
This can be used for specifying an initial guess, or holding the solution
returned by a solver.

The file format for reading and writing a MocoIterate is comprised of a
file header followed by a row of column names and the stored data. The file
header contains the number of states, controls, Lagrange multipliers (for
kinematic constraints), derivatives (non-zero if the dynamics mode is implict),
slacks (for special solver implmentations), and parameters (order does
not matter). Order does matter for the column names and corresponding data
columns. The columns *must* follow this order: time, states, controls,
multipliers, derivatives, slacks, parameters.
@note Slack columns may contain real number or NaN values, depending on their
use. For example, values for velocity correction variables used in problems
with model kinematic constraints are defined only at the midpoint of a Hermite-
Simpson mesh interval. The non-midpoint variables are returned as NaN in the
slack variable data structure.
@note For parameter columns, the value of the parameter is stored in
the first row of the column, while the rest of the rows are filled with
NaNs.
@samplefile
num_controls=<number-of-control-variables>
num_derivatives=<number-of-derivative-variables>
num_multipliers=<number-of-multiplier-variables>
num_parameters=<number-of-parameter-variables>
num_slacks=<number-of-slack-variables>
num_states=<number-of-state-variables>
time,<state-0-name>,...,<control-0-name>,...,<multiplier-0-name>,..., \
        <derivative-0-name>,...,<slack-0-name>,...,<parameter-0-name>,...
<#>,<#>,...,<#>,...,<#>,...,<#>,...,<#-or-NaN>,...,<#>  ,...
<#>,<#>,...,<#>,...,<#>,...,<#>,...,<#-or-NaN>,...,<NaN>,...
 : , : ,..., : ,..., : ,..., : ,...,    :     ,...,  :  ,...
<#>,<#>,...,<#>,...,<#>,...,<#>,...,<#-or-NaN>,...,<NaN>,...
@endsamplefile
(If stored in a STO file, the delimiters are tabs, not commas.)

Column labels starting with "lambda" are Lagrange multiplier, and columns
starting with "gamma" are slack variables (probably velocity corrections at
certain collocation points).

@par Matlab and Python
Many of the functions in this class have variants ending with "Mat" that
provide convenient access to the data directly in Matlab or Python (NumPy).
In Python, the constructors can also accept NumPy matrices in addition to
arguments of type SimTK::Matrix.
@code
iterate.getStateMat("<state-name>")
iterate.getStatesTrajectoryMat()
@endcode



@par Implicit dynamics model
If the solver uses an implicit dynamics mode, then there are "control"
variables ("adjunct" variables in tropter's terminology) for the generalized
accelerations. These are stored in the iterate as derivative variables. */
// Not using three-slash doxygen comments because that messes up verbatim.
class OSIMMOCO_API MocoIterate {
public:
    MocoIterate() = default;
    MocoIterate(const SimTK::Vector& time, std::vector<std::string> state_names,
            std::vector<std::string> control_names,
            std::vector<std::string> multiplier_names,
            std::vector<std::string> parameter_names,
            const SimTK::Matrix& statesTrajectory,
            const SimTK::Matrix& controlsTrajectory,
            const SimTK::Matrix& multipliersTrajectory,
            const SimTK::RowVector& parameters);
    /// This constructor is for use with the implicit dynamics mode, and
    /// allows specifying a derivativesTrajectory.
    MocoIterate(const SimTK::Vector& time, std::vector<std::string> state_names,
            std::vector<std::string> control_names,
            std::vector<std::string> multiplier_names,
            std::vector<std::string> derivative_names,
            std::vector<std::string> parameter_names,
            const SimTK::Matrix& statesTrajectory,
            const SimTK::Matrix& controlsTrajectory,
            const SimTK::Matrix& multipliersTrajectory,
            const SimTK::Matrix& derivativesTrajectory,
            const SimTK::RowVector& parameters);
#ifndef SWIG
    /// This constructor allows you to control which
    /// data you provide for the iterate. The possible keys for continuousVars
    /// are "states", "controls", "multipliers", and "derivatives". The names
    /// and data are grouped together, and you can leave out any of the keys.
    template <typename T>
    using NamesAndData = std::pair<std::vector<std::string>, T>;
    MocoIterate(const SimTK::Vector& time,
            const std::map<std::string, NamesAndData<SimTK::Matrix>>&
                    continuousVars,
            const NamesAndData<SimTK::RowVector>& parameters = {});
#endif
    /// Read a MocoIterate from a data file (e.g., STO, CSV). See output of
    /// write() for the correct format.
    explicit MocoIterate(const std::string& filepath);

    virtual ~MocoIterate() = default;

    /// Returns a dynamically-allocated copy of this iterate. You must manage
    /// the memory for return value.
    /// @note This works even if the iterate is sealed.
    virtual MocoIterate* clone() const { return new MocoIterate(*this); }

    bool empty() const {
        ensureUnsealed();
        return !(m_time.size() || m_states.nelt() || m_controls.nelt() ||
                 m_multipliers.nelt() || m_derivatives.nelt() ||
                 m_slacks.nelt() || m_parameters.nelt() ||
                 m_state_names.size() || m_control_names.size() ||
                 m_multiplier_names.size() || m_derivative_names.size() ||
                 m_slack_names.size() || m_parameter_names.size());
    }

    /// @name Change the length of the trajectory
    /// @{

    /// Resize the time vector and the time dimension of the states, controls,
    /// multipliers, and derivatives trajectories, and set all times, states,
    /// controls, multipliers, and derivatives to NaN.
    /// @note Parameters are NOT set to NaN.
    // TODO rename to setNumPoints(), setNumNodes(), setNumTimePoints().
    void setNumTimes(int numTimes) {
        ensureUnsealed();
        m_time.resize(numTimes);
        m_time.setToNaN();
        m_states.resize(numTimes, m_states.ncol());
        m_states.setToNaN();
        m_controls.resize(numTimes, m_controls.ncol());
        m_controls.setToNaN();
        m_multipliers.resize(numTimes, m_multipliers.ncol());
        m_multipliers.setToNaN();
        m_derivatives.resize(numTimes, m_derivatives.ncol());
        m_derivatives.setToNaN();
        m_slacks.resize(numTimes, m_slacks.ncol());
        m_slacks.setToNaN();
    }
    /// Uniformly resample (interpolate) the iterate so that it retains the
    /// same initial and final times but now has the provided number of time
    /// points.
    /// Resampling is done by creating a 5-th degree GCV spline of the states
    /// and controls and evaluating the spline at the `numTimes` time points.
    /// The degree is reduced as necessary if getNumTimes() < 6, and
    /// resampling is not possible if getNumTimes() < 2.
    /// @returns the resulting time interval between time points.
    double resampleWithNumTimes(int numTimes);
    /// Uniformly resample (interpolate) the iterate to try to achieve the
    /// provided time interval between mesh points, while preserving the
    /// initial and final times. The resulting time interval may be shorter
    /// than what you request (in order to preserve initial and
    /// final times), and is returned by this function.
    /// Resampling is done by creating a 5-th degree GCV spline of the states
    /// and controls and evaluating the spline at the new time points.
    /// The degree is reduced as necessary if getNumTimes() < 6, and
    /// resampling is not possible if getNumTimes() < 2.
    double resampleWithInterval(double desiredTimeInterval);
    /// Uniformly resample (interpolate) the iterate to try to achieve the
    /// provided frequency of time points per second of the trajectory, while
    /// preserving the initial and final times. The resulting frequency may be
    /// higher than what you request (in order to preserve initial and final
    /// times), and is returned by this function.
    /// Resampling is done by creating a 5-th degree GCV spline of the states
    /// and controls and evaluating the spline at the new time points.
    /// The degree is reduced as necessary if getNumTimes() < 6, and
    /// resampling is not possible if getNumTimes() < 2.
    double resampleWithFrequency(double desiredNumTimePointsPerSecond);
    /// Resample (interpolate) the data in this iterate at the provided times.
    /// If all times have the same value (e.g., 0.0), then the value of each
    /// variable for all time is its previous value at the initial time.
    /// @throws Exception if new times are not within existing initial and final
    /// times, if the new times are decreasing, or if getNumTimes() < 2.
    void resample(SimTK::Vector newTime);
    /// @}

    /// @name Set the data
    /// @{

    /// Set the time vector. The provided vector must have the same number of
    /// elements as the pre-existing time vector; use setNumTimes() or the
    /// "resample..." functions to change the number of times.
    /// @note Using `setTime({5, 10})` uses the initializer list overload
    /// below; it does *not* construct a 5-element vector with the value 10.
    void setTime(const SimTK::Vector& time);
    /// Set the value of a single state variable across time. The provided
    /// vector must have length getNumTimes().
    /// @note Using `setState(name, {5, 10})` uses the initializer list
    /// overload below; it does *not* construct a 5-element vector with the
    /// value 10.
    void setState(const std::string& name, const SimTK::Vector& trajectory);
    /// Set the value of a single control variable across time. The provided
    /// vector must have length getNumTimes().
    /// @note Using `setControl(name, {5, 10})` uses the initializer list
    /// overload below; it does *not* construct a 5-element vector with the
    /// value 10.
    void setControl(const std::string& name, const SimTK::Vector& trajectory);
    /// Set the value of a single Lagrange multiplier variable across time. The
    /// provided vector must have length getNumTimes().
    /// @note Using `setMultiplier(name, {5, 10})` uses the initializer list
    /// overload below; it does *not* construct a 5-element vector with the
    /// value 10.
    void setMultiplier(
            const std::string& name, const SimTK::Vector& trajectory);

    /// Set the value of a single parameter variable. This value is invariant
    /// across time.
    void setParameter(const std::string& name, const SimTK::Real& value);

    /// Set the time vector. The provided vector must have the same number of
    /// elements as the pre-existing time vector; use setNumTimes() or the
    /// "resample..." functions to change the number of times.
    /// This variant supports use of an initializer list. Example:
    /// @code{.cpp}
    /// iterate.setTime({0, 0.5, 1.0});
    /// @endcode
    void setTime(std::initializer_list<double> time) {
        ensureUnsealed();
        SimTK::Vector v((int)time.size());
        int i = 0;
        for (auto it = time.begin(); it != time.end(); ++it, ++i) v[i] = *it;
        setTime(v);
    }
    /// Set the value of a single state variable across time. The provided
    /// vector must have length getNumTimes().
    /// This variant supports use of an initializer list:
    /// @code{.cpp}
    /// iterate.setState("/jointset/knee/flexion/value", {0, 0.5, 1.0});
    /// @endcode
    void setState(
            const std::string& name, std::initializer_list<double> trajectory) {
        ensureUnsealed();
        SimTK::Vector v((int)trajectory.size());
        int i = 0;
        for (auto it = trajectory.begin(); it != trajectory.end(); ++it, ++i)
            v[i] = *it;
        setState(name, v);
    }
    /// Set the value of a single control variable across time. The provided
    /// vector must have length getNumTimes().
    /// This variant supports use of an initializer list:
    /// @code{.cpp}
    /// iterate.setControl("/forceset/soleus", {0, 0.5, 1.0});
    /// @endcode
    void setControl(
            const std::string& name, std::initializer_list<double> trajectory) {
        ensureUnsealed();
        SimTK::Vector v((int)trajectory.size());
        int i = 0;
        for (auto it = trajectory.begin(); it != trajectory.end(); ++it, ++i)
            v[i] = *it;
        setControl(name, v);
    }
    /// Set the value of a single Lagrange multiplier variable across time. The
    /// provided vector must have length getNumTimes().
    /// This variant supports use of an initializer list:
    /// @code{.cpp}
    /// iterate.setMultiplier("lambda_cid0_p0", {0, 0.5, 1.0});
    /// @endcode
    void setMultiplier(
            const std::string& name, std::initializer_list<double> trajectory) {
        ensureUnsealed();
        SimTK::Vector v((int)trajectory.size());
        int i = 0;
        for (auto it = trajectory.begin(); it != trajectory.end(); ++it, ++i)
            v[i] = *it;
        setMultiplier(name, v);
    }

    /// Set the states trajectory. The provided data is interpolated at the
    /// times contained within this iterate. The controls trajectory is not
    /// altered. If the table only contains a subset of the states in the
    /// iterate (and allowMissingColumns is true), the unspecified states
    /// preserve their pre-existing values.
    ///
    /// This function might be helpful if you generate a guess using a
    /// forward simulation; you can access the forward simulation's states
    /// trajectory using Manager::getStateStorage() or
    /// Manager::getStatesTable().
    ///
    /// @param states
    ///     The column labels of the table should match the state
    ///     names (see getStateNames()). By default, the table must provide all
    ///     state variables. Any data outside the time range of this guess's
    ///     times are ignored.
    /// @param allowMissingColumns
    ///     If false, an exception is thrown if there are states in the
    ///     iterate that are not in the table.
    /// @param allowExtraColumns
    ///     If false, an exception is thrown if there are states in the
    ///     table that are not in the iterate.
    /// @see createFromStatesControlsTables.
    // TODO add tests in testMocoInterface.
    // TODO add setStatesTrajectory(const StatesTrajectory&)
    // TODO handle rotational coordinates specified in degrees.
    void setStatesTrajectory(const TimeSeriesTable& states,
            bool allowMissingColumns = false, bool allowExtraColumns = false);
    /// @}

    /// @name Accessors
    /// @{

    int getNumTimes() const {
        ensureUnsealed();
        return m_time.size();
    }
    const SimTK::Vector& getTime() const {
        ensureUnsealed();
        return m_time;
    }
    /// The first time in the time vector.
    /// @throws Exception If numTimes is 0.
    double getInitialTime() const;
    /// The last time in the time vector.
    /// @throws Exception If numTimes is 0.
    double getFinalTime() const;

    // TODO inconsistent plural "state names" vs "states trajectory"
    const std::vector<std::string>& getStateNames() const {
        ensureUnsealed();
        return m_state_names;
    }
    const std::vector<std::string>& getControlNames() const {
        ensureUnsealed();
        return m_control_names;
    }
    const std::vector<std::string>& getMultiplierNames() const {
        ensureUnsealed();
        return m_multiplier_names;
    }
    const std::vector<std::string>& getDerivativeNames() const {
        ensureUnsealed();
        return m_derivative_names;
    }
    const std::vector<std::string>& getParameterNames() const {
        ensureUnsealed();
        return m_parameter_names;
    }
    SimTK::VectorView_<double> getState(const std::string& name) const;
    SimTK::VectorView_<double> getControl(const std::string& name) const;
    SimTK::VectorView_<double> getMultiplier(const std::string& name) const;
    SimTK::VectorView_<double> getDerivative(const std::string& name) const;
    const SimTK::Real& getParameter(const std::string& name) const;
    const SimTK::Matrix& getStatesTrajectory() const {
        ensureUnsealed();
        return m_states;
    }
    const SimTK::Matrix& getControlsTrajectory() const {
        ensureUnsealed();
        return m_controls;
    }
    const SimTK::Matrix& getMultipliersTrajectory() const {
        ensureUnsealed();
        return m_multipliers;
    }
    const SimTK::Matrix& getDerivativesTrajectory() const {
        ensureUnsealed();
        return m_derivatives;
    }
    const SimTK::RowVector& getParameters() const {
        ensureUnsealed();
        return m_parameters;
    }

    /// @}

    /// @name Comparisons
    /// @{

    /// Do the state and control names in this iterate match those in the
    /// problem? This may not catch all possible incompatibilities.
    /// The problem and this iterate can still be compatible even if the iterate
    /// contains no derivative columns.
    bool isCompatible(const MocoProblemRep&, bool throwOnError = false) const;
    /// Check if this iterate is numerically equal to another iterate.
    /// This uses SimTK::Test::numericallyEqual() internally.
    /// Accordingly, the tolerance is both a relative and absolute tolerance
    /// (depending on the magnitude of quantities being compared).
    bool isNumericallyEqual(const MocoIterate& other,
            double tol =
                    SimTK::NTraits<SimTK::Real>::getDefaultTolerance()) const;
    /// Compute the root-mean-square error between the continuous variables of
    /// this iterate and another. The RMS is computed by numerically integrating
    /// the sum of squared error across
    /// states,
    /// controls,
    /// Lagrange multipliers, and
    /// derivatives and dividing by the number of columns and the larger of the
    /// two time ranges. The calculation can be expressed as follows:
    /// \f[
    ///     \epsilon_{\textrm{RMS}} =
    ///     \sqrt{\frac{1}{N(t_f - t_i)} \int_{t_i}^{t_f} \left(
    ///         \sum_{ \textrm{i \in states} } \epsilon_i(t)^2 +
    ///         \sum_{ \textrm{i \in controls} } \epsilon_i(t)^2 +
    ///         \sum_{ \textrm{i \in mult} } \epsilon_i(t)^2 +
    ///         \sum_{ \textrm{i \in deriv} } \epsilon_i(t)^2
    ///     \right) dt  },
    /// \f]
    /// where \f$N\f$ is the number of columns and \f$ \epsilon \f$ indicates
    /// an error.
    ///
    /// When one iterate does not cover the same time range as
    /// the other, we assume values of 0 for the iterate with "missing" time.
    /// Numerical integration is performed using the trapezoidal rule. By
    /// default, all states, controls, and multipliers are compared, and it is
    /// expected that both iterates have the same states, controls, and
    /// multipliers. Alternatively, you can specify the specific
    /// states,
    /// controls,
    /// multipliers, and
    /// derivatives to compare as keys for `columnsToUse`.
    /// Values are an empty vector to compare all columns for that key,
    /// `{"none"}` (single-entry vector with value "none") to compare none of
    /// the columns for that key, or a vector of column labels to compare all
    /// all columns for that key. Leaving out a key means no columns for that
    /// key are compared.
    /// Both iterates must have at least 6 time nodes.
    /// If the number of columns to compare is 0, this returns 0.
    double compareContinuousVariablesRMS(const MocoIterate& other,
            std::map<std::string, std::vector<std::string>> columnsToUse = {}) const;
    /// Compute the root-mean-square error between the parameters in this
    /// iterate and another. The RMS is computed by dividing the the sum of the
    /// squared errors between corresponding parameters and then dividing by the
    /// number of parameters compared.
    /// By default, all parameters are compared, and it is expected that both
    /// iterates have the same parameters. Alternatively, you can specify the
    /// specific parameters to compare.
    double compareParametersRMS(const MocoIterate& other,
            std::vector<std::string> parameterNames = {}) const;
    /// @}

    /// @name Convert to other formats
    /// @{

    /// Save the iterate to file(s). Use a ".sto" file extension.
    void write(const std::string& filepath) const;

    /// The Storage can be used in the OpenSim GUI to visualize a motion, or
    /// as input to OpenSim's conventional tools (e.g., AnalyzeTool).
    ///
    /// Controls are not carried over to the states storage.
    Storage exportToStatesStorage() const;
    /// Same as exportToStatesStorage() except using TimeSeriesTable.
    TimeSeriesTable exportToStatesTable() const;
    /// Controls are not carried over to the StatesTrajectory.
    /// The MocoProblem is necessary because we need the underlying Model to
    /// order the state variables correctly.
    StatesTrajectory exportToStatesTrajectory(const MocoProblem&) const;
    /// @}

    /// @name Convert from other formats
    /// @{

    /// (Experimental) Create an iterate from a states trajectory and controls
    /// trajectory (i.e, from Manager::getStatesTable() and
    /// Model::getControlsTable()). The time columns from the two tables must
    /// match exactly. The times in the iterate will be those from the tables.
    /// This does not (yet) handle parameters.
    static MocoIterate createFromStatesControlsTables(const MocoProblemRep&,
            const TimeSeriesTable& statesTrajectory,
            const TimeSeriesTable& controlsTrajectory);
/// @}

// User interaction with slack variables is limited to using previous
// solution slack trajectories as initial guesses for subsequent problems.
// Therefore, these methods are hidden from doxygen and the bindings to
// discourage use.
#ifndef SWIG
    /// @cond
    void setSlack(const std::string& name, const SimTK::Vector& trajectory);
    void setSlack(
            const std::string& name, std::initializer_list<double> trajectory) {
        ensureUnsealed();
        SimTK::Vector v((int)trajectory.size());
        int i = 0;
        for (auto it = trajectory.begin(); it != trajectory.end(); ++it, ++i)
            v[i] = *it;
        setSlack(name, v);
    }
    const std::vector<std::string>& getSlackNames() const {
        ensureUnsealed();
        return m_slack_names;
    }
    const SimTK::Matrix& getSlacksTrajectory() const {
        ensureUnsealed();
        return m_slacks;
    }
    SimTK::VectorView_<double> getSlack(const std::string& name) const;
    // This additional mutator is necessary since no constructor exists to
    // update the slack variable data member variables.
    void appendSlack(const std::string& name, const SimTK::Vector& trajectory);

/// @endcond
#endif

protected:
    void setSealed(bool sealed) { m_sealed = sealed; }
    bool isSealed() const { return m_sealed; }
    /// @throws MocoIterateIsSealed if the iterate is sealed.
    void ensureUnsealed() const;

private:
    TimeSeriesTable convertToTable() const;
    double compareContinuousVariablesRMSInternal(const MocoIterate& other,
            std::vector<std::string> stateNames = {},
            std::vector<std::string> controlNames = {},
            std::vector<std::string> multiplierNames = {},
            std::vector<std::string> derivativeNames = {}) const;
    // TODO std::string m_name;
    SimTK::Vector m_time;
    std::vector<std::string> m_state_names;
    std::vector<std::string> m_control_names;
    std::vector<std::string> m_multiplier_names;
    std::vector<std::string> m_derivative_names;
    std::vector<std::string> m_slack_names;
    std::vector<std::string> m_parameter_names;
    // Dimensions: time x states
    SimTK::Matrix m_states;
    // Dimensions: time x controls
    SimTK::Matrix m_controls;
    // Dimensions: time x multipliers
    SimTK::Matrix m_multipliers;
    // Dimensions: time x derivatives
    SimTK::Matrix m_derivatives;
    // Dimensions: time x slacks
    SimTK::Matrix m_slacks;
    // Dimensions: 1 x parameters
    SimTK::RowVector m_parameters;

    // We use "seal" instead of "lock" because locks have a specific meaning
    // with threading (e.g., std::unique_lock()).
    bool m_sealed = false;
};

/// Return type for MocoTool::solve(). Use success() to check if the solver
/// succeeded. You can also use this object as a boolean in an if-statement:
/// @code
/// auto solution = moco.solve();
/// if (solution) {
///     std::cout << solution.getStatus() << std::endl;
/// }
/// @endcode
/// You can use getStatus() to get more details about the return status of
/// the optimizer.
/// If the solver was not successful, then this object is "sealed", which
/// means you cannot do anything with it until calling `unseal()`. This
/// prevents you from silently proceeding with a failed solution.
class OSIMMOCO_API MocoSolution : public MocoIterate {
public:
    /// Returns a dynamically-allocated copy of this solution. You must manage
    /// the memory for return value.
    /// @note This works even if the iterate is sealed.
    virtual MocoSolution* clone() const { return new MocoSolution(*this); }
    /// Was the problem solved successfully? If not, then you cannot access
    /// the solution until you call unlock().
    bool success() const { return m_success; }
    double getObjective() const { return m_objective; }
    /// Same as success().
    explicit operator bool() const { return success(); }
    /// Obtain a solver-dependent string describing the return status of the
    /// optimization.
    const std::string& getStatus() const { return m_status; }
    /// Number of solver iterations at which this solution was obtained
    /// (-1 if not set).
    int getNumIterations() const { return m_numIterations; }
    /// Get the amount of time (clock time, not CPU time) spent within solve().
    /// Units: seconds.
    double getSolverDuration() const { return m_solverDuration; }

    /// @name Access control
    /// @{

    /// If the solver did not succeed, call this to enable read and write
    /// access to the (failed) solution. If the solver succeeded, then the
    /// solution is already unsealed.
    /// @note In Python, you must invoke this function on a separate line:
    /// @code
    /// solution = moco.solve()
    /// solution.unseal()
    /// @endcode
    /// Otherwise, Moco will cause a crash.
    MocoSolution& unseal() {
        MocoIterate::setSealed(false);
        return *this;
    }
    MocoSolution& seal() {
        MocoIterate::setSealed(true);
        return *this;
    }
    bool isSealed() const { return MocoIterate::isSealed(); }
    /// @}

    // TODO num_iterations
    // TODO store the optimizer settings that were used.
private:
    using MocoIterate::MocoIterate;
    void setSuccess(bool success) {
        if (!success) setSealed(true);
        m_success = success;
    }
    void setObjective(double objective) { m_objective = objective; }
    void setStatus(std::string status) { m_status = std::move(status); }
    void setNumIterations(int numIterations) {
        m_numIterations = numIterations;
    };
    void setSolverDuration(double duration) { m_solverDuration = duration; }
    bool m_success = true;
    double m_objective = -1;
    std::string m_status;
    int m_numIterations = -1;
    double m_solverDuration = -1;
    // Allow solvers to set success, status, and construct a solution.
    friend class MocoSolver;
};

} // namespace OpenSim

#endif // MOCO_MOCOITERATE_H
