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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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
 
// INCLUDES
#include <OpenSim/Common/Array.h>
#include "OpenSim/Common/TimeSeriesTable.h"
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <SimTKcommon/internal/ReferencePtr.h>
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
 * A class that manages the execution of a simulation. This class uses a
 * SimTK::Integrator and SimTK::TimeStepper to perform the simulation. By
 * default, a Runge-Kutta-Merson integrator is used, but can be changed by
 * using setIntegratorMethod().
 * 
 * In order to prevent an inconsistency between the Integrator and TimeStepper,
 * we only create a TimeStepper once, specifically when we call
 * initialize(). To ensure this, the Manager will throw
 * an exception if initialize() is called more than once. Note
 * that editing the SimTK::State after calling initialize()
 * will not affect the simulation.
 *
 * Note that this interface means that you cannot "reinitialize" a Manager.
 * If you make changes to the SimTK::State, a new Manager must be created
 * before integrating again.
 */
class OSIMSIMULATION_API Manager {

//=============================================================================
// DATA
//=============================================================================
private:
    /** Simulation session name. */
    std::string _sessionName;
    /** Model for which the simulation is performed. */
    SimTK::ReferencePtr<Model> _model;
    SimTK::State _state;

    /** Integrator. */
    std::unique_ptr<SimTK::Integrator> _integ;

    /** TimeStepper */
    std::unique_ptr<SimTK::TimeStepper> _timeStepper;

    /** Storage for the states. */
    std::unique_ptr<Storage> _stateStore;

    /** Flag for signaling a desired halt. */
    bool _halt;

    /** Flag to indicate whether or not specified integration time steps
    should be used.  The specified integration time steps are held in _tVec.
    If _tVec does not contain time steps appropriate for the integration,
    an exception is thrown. */
    bool _specifiedDT;
    /** Flag to indicate whether or not constant (fixed) integration time
    steps should be used.  The constant integration time step is set using
    setDT(). */
    bool _constantDT;
    /** Constant integration time step. */
    double _dt;
    /** Vector of integration time steps. */
    Array<double> _tArray;
    /** Vector of integration time step deltas. */
    Array<double> _dtArray;

    /** Name to be shown by the UI */
    static std::string _displayName;

    /** flag indicating if manager should call Analyses after each step */
    bool _performAnalyses;

    /** flag indicating if manager should write to storage  each step */
    bool _writeToStorage;

    /** controllerSet used for the integration */
    SimTK::ReferencePtr<ControllerSet> _controllerSet;


//=============================================================================
// METHODS
//=============================================================================
public:
    /** Constructor that takes a model only and internally uses a
     * SimTK::RungeKuttaMersonIntegrator with default settings (accuracy,
     * constraint tolerance, etc.). */
    Manager(Model& model);

    /** Convenience constructor for creating and initializing a Manager (by
     * calling initialize()).
     * Do not use this constructor if you intend to change integrator settings;
     * changes to the integrator may not take effect after initializing. */
    Manager(Model& model, const SimTK::State& state);

    /** <b>(Deprecated)</b> A Constructor that does not take a model or
     * controllerSet. This constructor also does not set an integrator; you
     * must call setIntegrator() on your own. You should use one of the other
     * constructors. */
    DEPRECATED_14("There will be no replacement for this constructor.")
    Manager();

    // This class would not behave properly if copied (we would need to write a
    // complex custom copy constructor, etc.), so don't allow copies.
    Manager(const Manager&) = delete;
    void operator=(const Manager&) = delete;

private:
    /** Set all member variables to their null values. */
    void setNull();

    /** Construct the storage utility. */
    bool constructStorage();
    
public:
    // GET AND SET
    /** Set the model. and initializes other entities that depend on it. */
    void setModel(Model& model);

    /** Set the session name of this Manager instance. */
    void setSessionName(const std::string &name);

    /** Get the session name of this Manager instance. */
    const std::string& getSessionName() const;

    /**
     * Get the name to be shown for this object in Simtk-model tree
     */
    const std::string& toString() const;

    void setPerformAnalyses(bool performAnalyses)
    { _performAnalyses =  performAnalyses; }
    void setWriteToStorage(bool writeToStorage)
    { _writeToStorage =  writeToStorage; }

    /** @name Configure the Integrator
     * @note Call these functions before calling `Manager::initialize()`.
     * @{ */

    // Integrator
    /** Supported integrator methods. For MATLAB, int's must be used rather
     *  than enum's (see example in setIntegratorMethod(IntegratorMethod)). */
    enum class IntegratorMethod {
        ExplicitEuler      = 0, ///< 0 : For details, see SimTK::ExplicitEulerIntegrator.
        RungeKutta2        = 1, ///< 1 : For details, see SimTK::RungeKutta2Integrator.
        RungeKutta3        = 2, ///< 2 : For details, see SimTK::RungeKutta3Integrator. 
        RungeKuttaFeldberg = 3, ///< 3 : For details, see SimTK::RungeKuttaFeldbergIntegrator.
        RungeKuttaMerson   = 4, ///< 4 : For details, see SimTK::RungeKuttaMersonIntegrator.
        SemiExplicitEuler2 = 5, ///< 5 : For details, see SimTK::SemiExplicitEuler2Integrator.
        Verlet             = 6, ///< 6 : For details, see SimTK::VerletIntegrator.
        CPodes             = 7, ///< 7 : For details, see SimTK::CPodesIntegrator.

        // Not included
        //SemiExplicitEuler, no error ctrl, requires fixed stepSize arg on construction
    };

    /** Sets the integrator method used via IntegratorMethod enum. The 
     * integrator will be set to its default options, even if the caller
     * requests the same integrator method. Note that this function must
     * be called before `Manager::initialize()`.
      
      <b>C++ example</b>
      \code{.cpp}
      auto manager = Manager(model);
      manager.setIntegratorMethod(Manager::IntegratorMethod::SemiExplicitEuler2);
      \endcode
      
      <b>Python example</b>
      \code{.py}
      import opensim
      manager = opensim.Manager(model)
      manager.setIntegratorMethod(opensim.Manager.IntegratorMethod_SemiExplicitEuler2)
      \endcode

      <b>MATLAB example</b>
      \code{.m}
      import org.opensim.modeling.*
      manager = Manager(model);
      manager.setIntegratorMethod(5);
      \endcode
      */
    void setIntegratorMethod(IntegratorMethod integMethod);

    /** Get the integrator. */
    SimTK::Integrator& getIntegrator() const;

    /** Sets the accuracy of the integrator. 
     * For more details, see `SimTK::Integrator::setAccuracy(SimTK::Real)`. */
    void setIntegratorAccuracy(double accuracy);

    /** Sets the minimum step size of the integrator.
     * For more details, see `SimTK::Integrator::setMinimumStepSize(SimTK::Real)`. */
    void setIntegratorMinimumStepSize(double hmin);

    /** Sets the maximum step size of the integrator.
     * For more details, see `SimTK::Integrator::setMaximumStepSize(SimTK::Real)`. */
    void setIntegratorMaximumStepSize(double hmax);

    /** Sets the limit of steps the integrator can take per call of `stepTo()`.
     * Note that Manager::integrate() calls `stepTo()` for each interval when a fixed
     * step size is used.
     * For more details, see SimTK::Integrator::setInternalStepLimit(int). */
    void setIntegratorInternalStepLimit(int nSteps);

    //void setIntegratorFixedStepSize(double stepSize);
   
    /** @} */

    // SPECIFIED DT
    /**
     * Set whether or not to take a specified sequence of deltas during an
     * integration. The time deltas are obtained from what's stored in the
     * vector dt vector (@see setDTVector()).  In order to execute an
     * integration in this manner, the sum of the deltas must cover any
     * requested integration interval.  If not, an exception will be thrown
     * at the beginning of an integration.
     *
     * @param aTrueFalse If true, a specified dt's will be used.
     * If set to false, a variable-step integration or a constant step integration
     * will be used.
     * When set to true, the flag used to indicate whether or not a constant
     * time step is used is set to false.
     *
     * @see setDTVector()
     * @see getUseConstantDT()
     */
    void setUseSpecifiedDT(bool aTrueFalse);

    /**
     * Get whether or not to take a specified sequence of deltas during an
     * integration.
     * The time deltas are obtained from what's stored in the dt vector
     * (@see setDTVector()).  In order to execute an
     * integration in this manner, the sum of the deltas must cover any
     * requested integration interval.  If not, an exception will be thrown
     * at the beginning of an integration.
     *
     * @return If true, a specified time step will be used if possible.
     * If false, a variable-step integration will be performed or a constant
     * time step will be taken.
     *
     * @see getUseConstantDT()
     * @see getDT()
     * @see getTimeVector()
     */
    bool getUseSpecifiedDT() const;

    // CONSTANT DT
    /**
     * Set whether or not to take a constant integration time step. The size of
     * the constant integration time step can be set using setDT().
     *
     * @param aTrueFalse If true, constant time steps are used.
     * When set to true, the flag used to indicate whether or not to take
     * specified time steps is set to false.
     * If set to false, a variable-step integration or a constant integration
     * time step will be used.
     *
     * @see setDT()
     * @see setUseSpecifiedDT()
     * @see getUseSpecifiedDT();
     */
    void setUseConstantDT(bool aTrueFalse);

    // DT ARRAY
    /**
     * Get whether or not to use a constant integration time step. The
     * constant integration time step can be set using setDT().
     *
     * @return If true, constant time steps are used.  If false, either specified
     * or variable time steps are used.
     *
     * @see setDT()
     * @see getUseSpecifiedDTs();
     */
    bool getUseConstantDT() const;

    /**
     * Get the time deltas used in the last integration.
     *
     * @return Constant reference to the dt array.
     */
    const Array<double>& getDTArray();

    /**
     * Set the deltas held in the dt array.  These deltas will be used
     * if the integrator is set to take a specified set of deltas.  In order to
     * integrate using a specified set of deltas, the sum of deltas must cover
     * the requested integration time interval, otherwise an exception will be
     * thrown at the beginning of an integration.
     *
     * Note that the time vector is reconstructed in order to check that the
     * sum of the deltas covers a requested integration interval.
     *
     * @param aN Number of deltas.
     * @param aDT Array of deltas.
     * @param aTI Initial time.  If not specified, 0.0 is assumed.
     * @see getUseSpecifiedDT()
     */
    void setDTArray(const SimTK::Vector_<double>& aDT, double aTI = 0.0);

    /**
     * Get the delta used for a specified integration step.
     * For step aStep, the delta returned is the delta used to go from
     * step aStep to step aStep+1.
     *
     * @param aStep Index of the desired step.
     * @return Delta.  SimTK::Nan is returned on error.
     */
    double getDTArrayDT(int aStep);

    /**
     * Return the step size when the integrator is taking fixed step sizes.
     *
     * @param tArrayStep Step number
     */
    double getFixedStepSize(int tArrayStep) const;

    /**
     * Print the dt array.
     */
    void printDTArray(const char *aFileName=NULL);

    // TIME VECTOR
    /**
     * Get the sequence of time steps taken in the last integration.
     */
    const Array<double>& getTimeArray();

    /**
     * Get the integration step (index) that occurred prior to or at
     * a specified time.
     *
     * @param aTime Time of the integration step.
     * @return Step that occurred prior to or at aTime.  0 is returned if there
     * is no such time stored.
     */
    int getTimeArrayStep(double aTime);

    /**
     * Get the next time in the time array
     *
     * @param aTime Time of the integration step.
     * @return next time
     */
    double getNextTimeArrayTime(double aTime);

    /**
     * Get the time of a specified integration step.
     *
     * @param aStep Index of the desired step.
     * @return Time of integration step aStep. SimTK::NaN is returned on error.
     */
    double getTimeArrayTime(int aStep);

    /**
     * Print the time array.
     *
     * @param aFileName Name of the file to which to print.  If the time array
     * cannot be written to a file of the specified name, the time array is
     * written to standard out.
     */
    void printTimeArray(const char *aFileName=NULL);

    /**
     * Reset the time and dt arrays so that all times after the specified time
     * and the corresponding deltas are erased.
     *
     * @param aTime Time after which to erase the entries in the time and dt
     * vectors.
     */
    void resetTimeAndDTArrays(double aTime);

    // EXECUTION
    /**
     * Initializes the Manager by creating and initializing the underlying 
     * SimTK::TimeStepper. This must be called before calling 
     * Manager::integrate() Subsequent changes to the State object passed in 
     * here will not affect the simulation. Calling this function multiple 
     * times with the same Manager will trigger an Exception.
     *
     * Changes to the integrator (e.g., setIntegratorAccuracy()) after calling
     * initialize() may not have any effect.
     */
    void initialize(const SimTK::State& s);
    
    /**
     * Integrate the equations of motion for the specified model, given the current
     * state (at which the integration will start) and a finalTime. You must call
     * Manager::initialize() before calling this function.
     *
     * If you must update states or controls in a loop, you must recreate the 
     * manager within the loop (such discontinuous changes are considered "events"
     * and cannot be handled during integration of the otherwise continuous system).
     * The proper way to handle this situation is to create a SimTK::EventHandler.
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
     *     Manager manager(model);
     *     state.setTime(i*dTime);
     *     manager.initialize(state);
     *     state = manager.integrate((i+1)*dTime);
     * }
     * @endcode
     *
     */
    SimTK::State integrate(double finalTime);

    /** Get the current State from the Integrator associated with this 
      * Manager. */
    const SimTK::State& getState() const;
    

    // STATE STORAGE
    /** Set the Storage object to be used for storing states. The Manager takes
     * ownership of the passed-in Storage. */
    void setStateStorage(Storage& aStorage);

    /** Get the Storage object containing the integration states. */
    Storage& getStateStorage() const;

    /** Get whether there is a Storage object for the integration states. */
    bool hasStateStorage() const;
    
    /** Get a TimeSeriesTable containing the integration states. */
    TimeSeriesTable getStatesTable() const;

   // INTERRUPT
   /**
    * Halt an integration.
    *
    * If an integration is pending or executing, the value of the interrupt
    * flag is set to true.
    */
   void halt();

   /**
    * Clear the halt flag.
    *
    * The value of the interrupt flag is set to false.
    */
   void clearHalt();

   /**
    * Check for a halt request.
    *
    * The value of the halt flag is simply returned.
    */
   bool checkHalt();

private:
    // Helper to initialize storage and analyses.
    void initializeStorageAndAnalyses(const SimTK::State& s);

    // Helper to record state and analysis values at integration steps.
    // step = 0 is the beginning, step = -1 used to denote the end/final step
    void record(const SimTK::State& s, const int& step);

};  // END of class Manager

} // namespace OpenSim

#endif // OPENSIM_MANAGER_H_
