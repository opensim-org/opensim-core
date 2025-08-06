#ifndef OPENSIM_MANAGER_H_
#define OPENSIM_MANAGER_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Manager.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 * Contributor(s): Ajay Seth, Carmichael Ong, Nicholas Bianco                 *
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

/* Note: This code was originally developed by Realistic Dynamics Inc.
 * Author: Frank C. Anderson
 */

#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>

namespace SimTK {
class Integrator;
class State;
class System;
class TimeStepper;
}

namespace OpenSim {

class Model;
class Storage;
class ControllerSet;

/**
 * \section Manager
 * A class that manages the execution of a simulation.
 *
 * This class uses a SimTK::Integrator and SimTK::TimeStepper to perform the
 * simulation. By default, a Runge-Kutta-Merson integrator is used, but can be
 * changed by using setIntegratorMethod().
 *
 * \subsection man_usage Usage
 * The basic usage of a Manager is to construct it with a Model and then call
 * initialize() and integrate():
 *
 * @code
 * SimTK::State state = model.initSystem();
 * Manager manager(model);
 * manager.initialize(state);
 * state = manager.integrate(1.0);
 * @endcode
 *
 * See the documentation for integrate() for more usage examples.
 *
 * \subsection man_int_step SimTK::Integrator and SimTK::TimeStepper consistency
 * Upon construction, a Manager creates a SimTK::Integrator (defaulted to
 * SimTK::RungeKuttaMersonIntegrator) and a SimTK::TimeStepper. These will only
 * be changed if setIntegratorMethod() is called. Therefore, any changes to the
 * SimTK::Integrator (e.g., via setIntegratorAccuracy()) should be made after
 * calling setIntegratorMethod(). Otherwise, these settings will be overridden
 * when setIntegratorMethod() is called.
 *
 * \subsection man_init Initializing a Manager
 * After configuring the SimTK::Integrator, call the initialize() method to
 * initialize the Manager. This method initializes the internal
 * SimTK::TimeStepper and prepares data structures to record simulation
 * trajectory (e.g., if setRecordStatesTrajectory() or setWriteToStorage()
 * are set to true).
 *
 * Manager::initialize() must be called before calling Manager::integrate(). A
 * Manager may be reinitialized by calling initialize() with a different
 * initial State. Note that this effectively resets the simulation: the internal
 * SimTK::TimeStepper is reinitialized and the internal data structures are
 * cleared.
 *
 * \subsection man_results Obtaining simulation results
 * Results from a simulation can be obtained by enabling the appropriate
 * settings before calling Manager::integrate(). Manager::setPerformAnalyses()
 * enables the execution of all analyses registered with the Model during the
 * simulation via AnalysisSet::step() and is enabled by default.
 * Manager::setWriteToStorage() enables the writing of the simulation results to
 * a Storage object and is also enabled by default.
 * Manager::setRecordStatesTrajectory() enables the recording of a full
 * StatesTrajectory during a simulation; this setting is disabled by default
 * since it can have a significant impact on performance.
 *
 * When setWriteToStorage(true) is called, use getStateStorage() or
 * getStatesTable() to obtain the simulation trajectory. When
 * setRecordStatesTrajectory(true) is called, use getStatesTrajectory() to
 * obtain the simulation trajectory.
 */
class OSIMSIMULATION_API Manager {
public:
    // CONSTRUCTION
    /**
     * Model-only constructor.
     *
     * You must call Model::initSystem() before passing the model to this
     * constructor. The internal SimTK::Integrator is initialized to
     * SimTK::RungeKuttaMersonIntegrator with default settings
     * (accuracy, constraint tolerance, etc.).
     */
    Manager(Model& model);

    /**
     * Convenience constructor for creating and initializing a Manager.
     *
     * Manager::initialize() is called internally using the provided
     * SimTK::State. If integrator settings are changed after construction
     * (e.g., via setIntegratorMethod()), then initialize() must be called
     * again before calling integrate().
     */
    Manager(Model& model, const SimTK::State& state);

    // The default constructor is explicitly deleted. The Manager must be
    // constructed with a Model using one of the convenience constructors above.
    Manager() = delete;

    // This class would not behave properly if copied (we would need to write a
    // complex custom copy constructor, etc.), so don't allow copies.
    Manager(const Manager&) = delete;
    void operator=(const Manager&) = delete;
    // Similarly, disallow move construction and assignment.
    Manager(Manager&&) = delete;
    Manager& operator=(Manager&&) = delete;

    // DESTRUCTION
    ~Manager() = default;

    /**
     * @name Session configuration
     * @{ */
    /**
     * Set the session name of this Manager instance.
     */
    void setSessionName(const std::string &name);

    /**
     * Get the session name of this Manager instance.
     */
    const std::string& getSessionName() const;

    /**
     * %Set whether to perform analyses during the simulation.
     *
     * Enabling this setting will trigger the execution of all analyses
     * registered with the Model during the simulation via AnalysisSet::step().
     *
     * @see AnalysisSet::step()
     * @note This setting is enabled by default. If you do not need to perform
     *       analyses, you can disable this setting to improve performance.
     * @note When a Manager is reinitialized, the internal step count used for
     *       calling AnalysisSet::step() is reset to zero.
     */
    void setPerformAnalyses(bool performAnalyses);

    /**
     * %Set whether to write the simulation results to a Storage object.
     *
     * If this setting is enabled, the Manager will create a Storage object
     * and write the states and controls to it during the simulation.
     *
     * @note This setting is enabled by default. If you do not need to write
     *       results to a Storage object, you can disable this setting to
     *       improve performance.
     */
    void setWriteToStorage(bool writeToStorage);

    /**
     * %Set whether to record each SimTK::State from the integration into a
     * StatesTrajectory trajectory.
     *
     * This option provides an alternative to setWriteToStorage(), which records
     * state variable values at each step of the integration but not other
     * information contained in the SimTK::State (e.g., Lagrange multipliers,
     * constraint errors, etc.).
     *
     * @note This setting is disabled by default. Enabling this setting will
     *       have a significant impact on performance, greater than the speed
     *       reduction incurred from setWriteToStorage(). Therefore, it is
     *       recommend to disable this setting when generating many simulations
     *       (e.g., optimizing a controller).
     */
    void setRecordStatesTrajectory(bool recordStatesTrajectory);

    /** @} */

    /**
     * Supported integrator methods.
     *
     * @note For MATLAB, int's must be used rather than enum's (see example in
     * setIntegratorMethod(IntegratorMethod)). */
    enum class IntegratorMethod {
        ExplicitEuler      = 0, ///< 0 : For details, see SimTK::ExplicitEulerIntegrator.
        RungeKutta2        = 1, ///< 1 : For details, see SimTK::RungeKutta2Integrator.
        RungeKutta3        = 2, ///< 2 : For details, see SimTK::RungeKutta3Integrator.
        RungeKuttaFeldberg = 3, ///< 3 : For details, see SimTK::RungeKuttaFeldbergIntegrator.
        RungeKuttaMerson   = 4, ///< 4 : For details, see SimTK::RungeKuttaMersonIntegrator.
        SemiExplicitEuler2 = 5, ///< 5 : For details, see SimTK::SemiExplicitEuler2Integrator.
        Verlet             = 6, ///< 6 : For details, see SimTK::VerletIntegrator.
        CPodes             = 7, ///< 7 : For details, see SimTK::CPodesIntegrator.

        // No error control, reqiures a fixed step size argument on construction.
        // SemiExplicitEuler  = 8  ///< 8 : For details, see SimTK::SemiExplicitEulerIntegrator.
    };

    /**
     * @name Configuring the SimTK::Integrator
     * @note These methods should be called before calling Manager::initialize().
     * @{ */

    /**
     * %Set the integrator method used via the IntegratorMethod enum.
     *
     * A new SimTK::Integrator and SimTK::TimeStepper with default values are
     * created after each call to this method. Therefore, integrator settings
     * (e.g., setIntegratorAccuracy()) must be set _after_ calling this method,
     * and you should call this method before Manager::initialize().
     *
     * <b>C++ example</b>
     * \code{.cpp}
     * auto manager = Manager(model);
     * manager.setIntegratorMethod(Manager::IntegratorMethod::SemiExplicitEuler2);
     * \endcode
     *
     * <b>Python example</b>
     * \code{.py}
     * import opensim
     * manager = opensim.Manager(model)
     * manager.setIntegratorMethod(opensim.Manager.IntegratorMethod_SemiExplicitEuler2)
     * \endcode
     *
     * <b>MATLAB example</b>
     * \code{.m}
     * import org.opensim.modeling.*
     * manager = Manager(model);
     * manager.setIntegratorMethod(5);
     * \endcode
     */
    void setIntegratorMethod(IntegratorMethod integMethod);

    /**
     * Get the IntegratorMethod enum.
     */
    IntegratorMethod getIntegratorMethod() const;

    /**
     * %Set the accuracy of the internal SimTK::Integrator.
     *
     * @see SimTK::Integrator::setAccuracy(SimTK::Real)
     * @note If a new integrator is set via setIntegratorMethod(), this
     *       setting will be cleared and set to the default value.
     */
    void setIntegratorAccuracy(double accuracy);

    /**
     * %Set the minimum step size of the internal SimTK::Integrator.
     *
     * @see SimTK::Integrator::setMinimumStepSize(SimTK::Real)
     * @note If a new integrator is set via setIntegratorMethod(), this
     *       setting will be cleared and set to the default value.
     * @note The integrators supported by Manager by default use error-controlled
     *       adaptive stepping, meaning that the step size is adjusted
     *       automatically during integration to maintain the desired accuracy.
     *       By calling this method, you set a lower bound on the step size,
     *       which may degrade accuracy and even perfomance.
     */
    void setIntegratorMinimumStepSize(double hmin);

    /**
     * %Set the maximum step size of the internal SimTK::Integrator.
     *
     * @see SimTK::Integrator::setMaximumStepSize(SimTK::Real)
     * @note If a new integrator is set via setIntegratorMethod(), this
     *       setting will be cleared and set to the default value.
     * @note The integrators supported by Manager by default use error-controlled
     *       adaptive stepping, meaning that the step size is adjusted
     *       automatically during integration to maintain the desired accuracy.
     *       By calling this method, you set an upper bound on the step size,
     *       which may degrade performance.
     */
    void setIntegratorMaximumStepSize(double hmax);

    /**
     * %Set the limit of steps the integrator can take per call of stepTo().
     *
     * @see SimTK::Integrator::setInternalStepLimit(int).
     * @note If a new integrator is set via setIntegratorMethod(), this
     *       setting will be cleared and set to the default value.
     */
    void setIntegratorInternalStepLimit(int nSteps);

    /**
     * %Set a fixed step size for the internal SimTK::Integrator.
     *
     * @see SimTK::Integrator::setFixedStepSize(SimTK::Real)
     * @note If a new integrator is set via setIntegratorMethod(), this
     *       setting will be cleared and set to the default value.
     * @note The integrators supported by Manager by default use error-controlled
     *       adaptive stepping, meaning that the step size is adjusted
     *       automatically during integration to maintain the desired accuracy.
     *       By calling this method, you override this behavior and set a fixed
     *       step size for the integrator, which may degrade performance and
     *       accuracy. If you need a fixed reporting interval, consider using
     *       a Reporter which will preserve internal adaptive stepping.
     */
    void setIntegratorFixedStepSize(double stepSize);

    /**
     * (Advanced) %Set the final time for the internal SimTK::Integrator.
     *
     * This setting causes the integration to stop when the final time is
     * reached, even if the integrator has not reached the final time provided
     * to Manager::integrate().
     *
     * @see SimTK::Integrator::setFinalTime(SimTK::Real)
     * @note If a new integrator is set via setIntegratorMethod(), this
     *       setting will be cleared and set to the default value.
     */
    void setIntegratorFinalTime(double finalTime);

    /**
     * (Advanced) Use infinity norm (maximum absolute value) instead of default
     * RMS norm to evaluate whether accuracy has been achieved for states and
     * for constraint tolerance for the internal SimTK::Integrator.
     *
     * The infinity norm is more strict but may permit use of a looser accuracy
     * request.
     *
     * @see SimTK::Integrator::setUseInfinityNorm(bool)
     * @note If a new integrator is set via setIntegratorMethod(), this
     *       setting will be cleared and set to the default value.
     */
    void setIntegratorUseInfinityNorm(bool useInfinityNorm);

    /**
     * (Advanced) %Set the tolerance within which constraints must be satisfied
     * for the internal SimTK::Integrator.
     *
     * @see SimTK::Integrator::setConstraintTolerance(SimTK::Real)
     * @note If a new integrator is set via setIntegratorMethod(), this
     *       setting will be cleared and set to the default value.
     */
    void setIntegratorConstraintTolerance(double tol);

    /**
     * Get the internal SimTK::Integrator.
     */
    SimTK::Integrator& getIntegrator() const;

    /** @} */

    /**
     * @name Executing a simulation
     * @{ */

    /**
     * Initialize the Manager.
     *
     * This must be called after configuring the integrator and before calling
     * Manager::integrate(). If the integrator is changed via
     * setIntegratorMethod(), this method must be called again, otherwise an
     * exception is thrown. Subsequent changes to the State object passed in
     * here will not affect the simulation.
     */
    void initialize(const SimTK::State& s);

    /**
     * Integrate the equations of motion for the specified model, given the
     * current state (at which the integration will start) and a final time. You
     * must call Manager::initialize() before calling this function.
     *
     * If you must update the SimTK::State in a loop, you must call
     * Manager::initialize() before each call to integrate(). This is because
     * discontinuous changes are considered "events" and cannot be handled
     * during integration of the otherwise continuous system. If you make
     * changes to the state and continue integrating without re-initializing,
     * these changes will be ignored. If you make changes to the model and
     * continue integrating without re-initializing, an exception will be thrown.
     *
     * If the provided final time is greater than or equal to the current time,
     * an exception is thrown.
     *
     * @note The proper way to handle the simulation of a discontinuous system
     *       is to create a SimTK::EventHandler.
     *
     * Example: Integrating from time = 1s to time = 2s
     * @code
     * SimTK::State state = model.initSystem();
     * Manager manager(model);
     * state.setTime(1.0);
     * manager.initialize(state);
     * state = manager.integrate(2.0);
     * @endcode
     *
     * Example: Integrating from time = 1s to time = 2s using the
     *          convenience constructor
     * @code
     * SimTK::State state = model.initSystem();
     * state.setTime(1.0);
     * Manager manager(model, state);
     * state = manager.integrate(2.0);
     * @endcode
     *
     * Example: Integrate from time = 0s to time = 10s, in 2s increments
     * @code
     * dTime = 2.0;
     * finalTime = 10.0;
     * int n = int(round(finalTime/dTime));
     * state.setTime(0.0);
     * manager.initialize(state);
     * for (int i = 1; i <= n; ++i) {
     *     state = manager.integrate(i*dTime);
     * }
     * @endcode
     *
     * Example: Integrate from time = 0s to time = 10s, updating the state
     *          (e.g., the model's first coordinate value) every 2s
     * @code
     * dTime = 2.0;
     * finalTime = 10.0;
     * int n = int(round(finalTime/dTime));
     * state.setTime(0.0);
     * for (int i = 0; i < n; ++i) {
     *     model.getCoordinateSet().get(0).setValue(state, 0.1*i);
     *     state.setTime(i*dTime);
     *     manager.initialize(state);
     *     state = manager.integrate((i+1)*dTime);
     * }
     * @endcode
     *
     * Example: Integrate using a specified sequence of time steps
     * @code
     * state.setTime(0.0);
     * std::vector<double> times = {0.02, 0.04, 0.05, 0.06, 0.08, 0.1, 0.2};
     * for (double time : times) {
     *     state = manager.integrate(time);
     * }
     * @endcode
     */
    const SimTK::State& integrate(double finalTime);

    /** @} */

    /**
     * @name Obtaining simulation results
     * @{ */

    /**
     * Get the current SimTK::State from the SimTK::Integrator associated with
     * this Manager.
     */
    const SimTK::State& getState() const;

    /**
     * Get a Storage object containing the integration states.
     *
     * @note If setWriteToStorage(false) was called, this will return an empty
     *       Storage object.
     */
    Storage getStateStorage() const;

    /**
     * Get a TimeSeriesTable containing the integration states.
     *
     * The returned TimeSeriesTable is converted from the Storage object
     * populated during the simulation when setWriteToStorage() is enabled
     * (which is the default setting).
     *
     * @note If setWriteToStorage(false) was called, this will return an empty
     *       TimeSeriesTable.
     */
    TimeSeriesTable getStatesTable() const;

    /**
     * Get a StatesTrajectory containing the integration states.
     *
     * @note If setRecordStatesTrajectory(false) was called, this will return
     *       an empty StatesTrajectory.
     */
    StatesTrajectory getStatesTrajectory() const;

    /** @} */

private:
    // MEMBER VARIABLES
    std::string _sessionName;
    bool _recordStatesTrajectory;
    bool _performAnalyses;
    bool _writeToStorage;

    SimTK::ReferencePtr<Model> _model;

    std::unique_ptr<SimTK::TimeStepper> _timeStepper;
    std::unique_ptr<SimTK::Integrator> _integ;
    IntegratorMethod _integMethod = IntegratorMethod::RungeKuttaMerson;

    std::unique_ptr<StatesTrajectory> _statesTraj;
    std::unique_ptr<Storage> _stateStore;

    // HELPER METHOD
    // Helper method to record state and analysis values at integration steps.
    // `step = 0` denotes the first step, and `step = -1` denotes the final step.
    void record(const SimTK::State& state, int step);

};  // END of class Manager

} // namespace OpenSim

#endif // OPENSIM_MANAGER_H_
