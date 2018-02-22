#ifndef MUSCOLLO_MUCOITERATE_H
#define MUSCOLLO_MUCOITERATE_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoIterate.h                                           *
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

#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Common/Storage.h>

#include "osimMuscolloDLL.h"

namespace OpenSim {

class MucoProblem;

/// This exception is thrown if you try to invoke most methods on MucoIterate
/// while the iterate is sealed.
class OSIMMUSCOLLO_API MucoIterateIsSealed : public Exception {
public:
    MucoIterateIsSealed(const std::string& file, size_t line,
            const std::string& func) : Exception(file, line, func) {
        addMessage("This iterate is sealed, to force you to acknowledge the "
                "solver failed; call unseal() to gain access.");
    }
};

/// The values of the variables in an optimal control problem.
/// This can be used for specifying an initial guess, or holding the solution
/// returned by a solver. 
///
/// The file format for reading and writing a MucoIterate is comprised of a 
/// file header followed by a row of column names and the stored data. The file
/// header contains the number of states, controls, and parameters (order does
/// not matter). Order does matter for the column names and corresponding data 
/// columns. The columns *must* follow this order: time, states, controls, 
/// parameters. For parameter columns, the value of the parameter is stored in 
/// the first row of the column, while the rest of the rows are filled with 
/// NaNs.
/// @samplefile
/// num_controls=<number-of-control-variables>
/// num_parameters=<number-of-parameter-variables>
/// num_states=<number-of-state-variables>
/// time,<state-0-name>,...,<control-0-name>,...,<parameter-0-name>,...
/// <#>,<#>,...,<#>,...,<#>,...
/// <#>,<#>,...,<#>,...,<NaN>,...
///  : , : ,..., : ,...,  :  ,...
/// <#>,<#>,...,<#>,...,<NaN>,...
/// @endsamplefile
/// (If stored in a STO file, the delimiters are tabs, not commas.)
class OSIMMUSCOLLO_API MucoIterate {
public:
    MucoIterate() = default;
    MucoIterate(const SimTK::Vector& time,
            std::vector<std::string> state_names,
            std::vector<std::string> control_names,
            std::vector<std::string> parameter_names,
            const SimTK::Matrix& statesTrajectory,
            const SimTK::Matrix& controlsTrajectory,
            const SimTK::RowVector& parameters);
    /// Read a MucoIterate from a data file (e.g., STO, CSV). See output of
    /// write() for the correct format.
    // TODO describe format.
    explicit MucoIterate(const std::string& filepath);
    /// Returns a dynamically-allocated copy of this iterate. You must manage
    /// the memory for return value.
    /// @note This works even if the iterate is sealed.
    virtual MucoIterate* clone() const { return new MucoIterate(*this); }

    bool empty() const {
        ensureUnsealed();
        return !(m_time.size() || m_states.nelt() || m_controls.nelt() ||
                m_parameters.nelt() || m_state_names.size() || 
                m_control_names.size() || m_parameter_names.size());
    }

    /// @name Change the length of the trajectory
    /// @{

    /// setNumTimes() -> setNumNodes().

    /// Resize the time vector and the time dimension of the states and controls
    /// trajectories, and set all times, states, and controls to NaN.
    /// @note Parameters are NOT set to NaN.
    // TODO rename to setNumPoints() or setNumTimePoints().
    void setNumTimes(int numTimes)
    {
        ensureUnsealed();
        m_time.resize(numTimes);
        m_time.setToNaN();
        m_states.resize(numTimes, m_states.ncol());
        m_states.setToNaN();
        m_controls.resize(numTimes, m_controls.ncol());
        m_controls.setToNaN();
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
        for (auto it = time.begin(); it != time.end(); ++it, ++i)
            v[i] = *it;
        setTime(v);
    }
    /// Set the value of a single state variable across time. The provided
    /// vector must have length getNumTimes().
    /// This variant supports use of an initializer list:
    /// @code{.cpp}
    /// iterate.setState("knee/flexion/value", {0, 0.5, 1.0});
    /// @endcode
    void setState(const std::string& name,
            std::initializer_list<double> trajectory) {
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
    /// iterate.setControl("soleus", {0, 0.5, 1.0});
    /// @endcode
    void setControl(const std::string& name,
            std::initializer_list<double> trajectory) {
        ensureUnsealed();
        SimTK::Vector v((int)trajectory.size());
        int i = 0;
        for (auto it = trajectory.begin(); it != trajectory.end(); ++it, ++i)
            v[i] = *it;
        setControl(name, v);
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
    // TODO add tests in testMuscolloInterface.
    // TODO add setStatesTrajectory(const StatesTrajectory&)
    // TODO handle rotational coordinates specified in degrees.
    void setStatesTrajectory(const TimeSeriesTable& states,
            bool allowMissingColumns = false, bool allowExtraColumns = false);
    /// @}

    /// @name Accessors
    /// @{

    int getNumTimes() const
    {   ensureUnsealed(); return m_time.size(); }
    const SimTK::Vector& getTime() const
    {   ensureUnsealed(); return m_time; }
    // TODO inconsistent plural "state names" vs "states trajectory"
    const std::vector<std::string>& getStateNames() const
    {   ensureUnsealed(); return m_state_names; }
    const std::vector<std::string>& getControlNames() const
    {   ensureUnsealed(); return m_control_names; }
    const std::vector<std::string>& getParameterNames() const
    {   ensureUnsealed(); return m_parameter_names; }
    SimTK::VectorView_<double> getState(const std::string& name) const;
    SimTK::VectorView_<double> getControl(const std::string& name) const;
    const SimTK::Real& getParameter(const std::string& name) const;
    const SimTK::Matrix& getStatesTrajectory() const
    {   ensureUnsealed(); return m_states; }
    const SimTK::Matrix& getControlsTrajectory() const
    {   ensureUnsealed(); return m_controls; }
    const SimTK::RowVector& getParameters() const
    {   ensureUnsealed(); return m_parameters; }

    /// @}

    /// @name Comparisons
    /// @{

    /// Do the state and control names in this iterate match those in the
    /// problem? This may not catch all possible incompatibilities.
    bool isCompatible(const MucoProblem&, bool throwOnError = false) const;
    /// Check if this iterate is numerically equal to another iterate.
    /// This uses SimTK::Test::numericallyEqual() internally.
    /// Accordingly, the tolerance is both a relative and absolute tolerance
    /// (depending on the magnitude of quantities being compared).
    bool isNumericallyEqual(const MucoIterate& other,
            double tol = SimTK::NTraits<SimTK::Real>::getDefaultTolerance())
            const;
    /// Compute the root-mean-square error between this iterate and another.
    /// The RMS is computed by numerically integrating the sum of squared
    /// error across states and controls and dividing by the larger of the
    /// two time ranges. When one iterate does not cover the same time range as
    /// the other, we assume values of 0 for the iterate with "missing" time.
    /// Numerical integration is performed using the trapezoidal rule.
    /// By default, all states and controls are compared, and it is expected
    /// that both iterates have the same states and controls. Alternatively,
    /// you can specify the specific states and controls to compare. To skip
    /// over all states, specify a single element of "none" for stateNames;
    /// likewise for controlNames.
    /// Both iterates must have at least 6 time nodes.
    double compareStatesControlsRMS(const MucoIterate& other,
            std::vector<std::string> stateNames = {},
            std::vector<std::string> controlNames = {}) const;
    /// Compute the root-mean-square error between the parameters in this
    /// iterate and another. The RMS is computed by dividing the the sum of the
    /// squared errors between corresponding parameters and then dividing by the
    /// number of parameters compared.
    /// By default, all parameters are compared, and it is expected that both
    /// iterates have the same parameters. Alternatively, you can specify the
    /// specific parameters to compare.
    double compareParametersRMS(const MucoIterate& other,
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
    // TODO use TimeSeriesTable instead?
    Storage exportToStatesStorage() const;
    /// Controls are not carried over to the StatesTrajectory.
    /// The MucoProblem is necessary because we need the underlying Model to
    /// order the state variables correctly.
    StatesTrajectory exportToStatesTrajectory(const MucoProblem&) const;
    /// @}

    /// @name Convert from other formats
    /// @{

    /// (Experimental) Create an iterate from a states trajectory and controls
    /// trajectory (i.e, from Manager::getStatesTable() and
    /// Model::getControlsTable()). The time columns from the two tables must
    /// match exactly. The times in the iterate will be those from the tables.
    /// This does not (yet) handle parameters.
    static MucoIterate createFromStatesControlsTables(
            const MucoProblem&,
            const TimeSeriesTable& statesTrajectory,
            const TimeSeriesTable& controlsTrajectory);
    /// @}

protected:
    void setSealed(bool sealed) { m_sealed = sealed; }
    bool isSealed() const { return m_sealed; }
    /// @throws MucoIterateIsSealed if the iterate is sealed.
    void ensureUnsealed() const;

private:
    TimeSeriesTable convertToTable() const;
    // TODO std::string m_name;
    SimTK::Vector m_time;
    std::vector<std::string> m_state_names;
    std::vector<std::string> m_control_names;
    std::vector<std::string> m_parameter_names;
    // Dimensions: time x states
    SimTK::Matrix m_states;
    // Dimensions: time x controls
    SimTK::Matrix m_controls;
    // Dimensions: 1 x parameters
    SimTK::RowVector m_parameters;

    // We use "seal" instead of "lock" because locks have a specific meaning
    // with threading (e.g., std::unique_lock()).
    bool m_sealed = false;
};

/// Return type for MucoTool::solve(). Use success() to check if the solver
/// succeeded. You can also use this object as a boolean in an if-statement:
/// @code
/// auto solution = muco.solve();
/// if (solution) {
///     std::cout << solution.getStatus() << std::endl;
/// }
/// @endcode
/// You can use getStatus() to get more details about the return status of
/// the optimizer.
/// If the solver was not successful, then this object is "sealed", which
/// means you cannot do anything with it until calling `unseal()`. This
/// prevents you from silently proceeding with a failed solution.
class OSIMMUSCOLLO_API MucoSolution : public MucoIterate {
public:
    /// Returns a dynamically-allocated copy of this solution. You must manage
    /// the memory for return value.
    /// @note This works even if the iterate is sealed.
    virtual MucoSolution* clone() const { return new MucoSolution(*this); }
    /// Was the problem solved successfully? If not, then you cannot access
    /// the solution until you call unlock().
    bool success() const { return m_success; }
    /// Same as success().
    explicit operator bool() const { return success(); }
    /// Obtain a solver-dependent string describing the return status of the
    /// optimization.
    const std::string& getStatus() const { return m_status; }
    /// Number of solver iterations at which this solution was obtained
    /// (-1 if not set).
    int getNumIterations() const { return m_numIterations; }

    /// @name Access control
    /// @{

    /// If the solver did not succeed, call this to enable read and write
    /// access to the (failed) solution. If the solver succeeded, then the
    /// solution is already unsealed.
    MucoSolution& unseal() { MucoIterate::setSealed(false); return *this; }
    MucoSolution& seal() { MucoIterate::setSealed(true); return *this; }
    bool isSealed() const { return MucoIterate::isSealed(); }
    /// @}

    // TODO num_iterations
    // TODO store the optimizer settings that were used.
private:
    using MucoIterate::MucoIterate;
    void setSuccess(bool success) {
        if (!success) setSealed(true);
        m_success = success;
    }
    void setStatus(std::string status) { m_status = std::move(status); }
    void setNumIterations(int numIterations)
    {   m_numIterations = numIterations; };
    bool m_success = true;
    std::string m_status;
    int m_numIterations = -1;
    // Allow solvers to set success, status, and construct a solution.
    friend class MucoSolver;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOITERATE_H
